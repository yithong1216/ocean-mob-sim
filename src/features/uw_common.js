// ============================================================================
// Underwater view + search-and-rescue gear (uw_*): shared state, water optics, GUI
// ============================================================================
// uw_common.js   shared uniforms (camera-local water plane, inherent optical properties), GUI folder
// uw_gear.js     lifebuoy, life raft, debris, rescue vessel, fluorescein dye patch
// uw_underwater.js  underwater camera, surface seen from below, water-volume pass, god rays, caustics,
//                   marine snow, bubbles, half-over/half-under split view
Object.assign(params, {
  uwAllowUnder: true,     // orbit camera may go below the surface
  uwSplit: false,         // camera floats at the surface (half over / half under)
  uwClarity: 1.0,         // underwater visibility multiplier
  uwGodRays: 1.0, uwCaustics: 1.0, uwSnow: true, uwBubbles: true,
  uwBuoy: true, uwRaft: true, uwDebris: true, uwVessel: true, uwDye: false,
});

// uniforms shared by reference between the ocean shader, the person/gear shaders and the underwater pass
const uwU = {
  // camera-local water surface: height above the surface of point p = dot(xyz, p) + w (from the camera probe)
  uUwPlane: { value: new THREE.Vector4(0, 1, 0, 0) },
  // x: underwater rendering enabled, yz: camera xz the plane belongs to, w: half-width of the straddle band (m)
  uUwInfo: { value: new THREE.Vector4(0, 0, 0, 0.3) },
  uUwSigT: { value: new THREE.Vector3(0.5, 0.11, 0.1) },     // extinction coefficient (1/m)
  uUwSigS: { value: new THREE.Vector3(0.04, 0.05, 0.06) },   // scattering coefficient (1/m)
  uUwKd: { value: new THREE.Vector3(0.46, 0.08, 0.06) },     // diffuse attenuation of downwelling light (1/m)
  uUwSunW: { value: new THREE.Vector3(0, 1, 0) },            // towards the refracted sun, in the water
  uUwSunE: { value: new THREE.Vector3() },                   // refracted solar beam irradiance just below the surface
  uUwSunT: { value: 0 },                                     // that irradiance relative to the beam in air
  uUwTime: { value: 0 },
};
const UW = {
  mode: 0,            // 0 above water, 1 camera under water, 2 straddling the surface (split view)
  camS: 10,           // camera height above the local surface (m)
  camH: 0,            // water height under the camera (m)
  sx: 0, sz: 0,       // surface slope at the camera
};
const UW_WATER0 = new THREE.Color('#2c646d');   // default water colour: the tint of the underwater scattering is relative to it

// Inherent optical properties of the water column (moderately clear oceanic water, Jerlov II-ish)
function uwUpdateOptics() {
  const t = params.turbidity, k = 1 / Math.max(params.uwClarity, 0.05);
  const wc = new THREE.Color(params.waterColor);
  const tint = [wc.r / UW_WATER0.r, wc.g / UW_WATER0.g, wc.b / UW_WATER0.b].map(v => clamp(v, 0.2, 3));
  const a = [0.43 + 0.03 * t, 0.055 + 0.02 * t, 0.035 + 0.035 * t];         // pure water + CDOM/particles
  const b = [0.028 * t * tint[0], 0.05 * t * tint[1], 0.058 * t * tint[2]]; // particle scattering
  uwU.uUwSigS.value.set(b[0] * k, b[1] * k, b[2] * k);
  uwU.uUwSigT.value.set((a[0] + b[0]) * k, (a[1] + b[1]) * k, (a[2] + b[2]) * k);
  // Kd ~ (a + bb) / mu_d with mu_d ~ 0.85 (bb ~ 2% of b); kept below c so the in-scatter integral stays finite
  uwU.uUwKd.value.set(...a.map((v, i) => Math.min((v + 0.02 * b[i]) / 0.85 * k, 0.92 * (v + b[i]) * k)));
}

// sun direction refracted into the water and the beam irradiance just below the surface
function uwUpdateSun() {
  const L = lightDir, ci = Math.max(L.y, 0.02);
  const si = Math.sqrt(1 - ci * ci), st = si / 1.333, ct = Math.sqrt(1 - st * st);
  const hl = Math.hypot(L.x, L.z) || 1;
  uwU.uUwSunW.value.set(L.x / hl * st, ct, L.z / hl * st);
  // Fresnel loss at the air-water interface (Schlick on the air side); beam compression cos(i)/cos(t)
  const F = 0.02 + 0.98 * Math.pow(1 - ci, 5);
  const E = oceanU.uSunE.value, up = L.y > 0 ? (1 - F) * ci / ct : 0;
  uwU.uUwSunE.value.set(E.x * up, E.y * up, E.z * up);
  uwU.uUwSunT.value = up;
}

// camera-local water plane from the camera probe (sampled at this frame's time, at last frame's camera xz)
function uwUpdatePlane() {
  const s = probe(CAM_PROBE), qx = queryData[CAM_PROBE * 4], qz = queryData[CAM_PROBE * 4 + 1];
  const cx = camera.position.x, cz = camera.position.z;
  // ripple slopes are only valid very close to the camera
  const sx = clamp(s.sx, -0.6, 0.6), sz = clamp(s.sz, -0.6, 0.6);
  const h = s.h + sx * (cx - qx) + sz * (cz - qz);
  UW.camH = h; UW.sx = sx; UW.sz = sz;
  UW.camS = camera.position.y - h;
  uwU.uUwPlane.value.set(-sx, 1, -sz, sx * cx + sz * cz - h);
  // the near plane can cut the surface only inside this band (near-plane half diagonal + margin)
  const halfH = camera.near * Math.tan(camera.fov * D2R / 2);
  const band = halfH * Math.sqrt(1 + camera.aspect * camera.aspect) + 0.12;
  uwU.uUwInfo.value.set(params.uwAllowUnder ? 1 : 0, cx, cz, band);
  UW.mode = !params.uwAllowUnder ? 0 : UW.camS < -band ? 1 : UW.camS < band ? 2 : 0;
}

const uwGui = gui.addFolder('Underwater & SAR gear');
