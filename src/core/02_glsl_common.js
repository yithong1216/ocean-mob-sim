// ============================================================================
// Shared GLSL
// ============================================================================
const ATMO_GLSL = /* glsl */`
const float PI = 3.14159265358979;
const float RG = 6360.0;
const float RT = 6460.0;
const vec3 RAY_S = vec3(5.802e-3, 13.558e-3, 33.1e-3);
const float MIE_S = 3.996e-3;
const float MIE_E = 4.40e-3;
const vec3 OZO_A = vec3(0.650e-3, 1.881e-3, 0.085e-3);
const float SUN_ANG_R = 0.004675;
const float TRANS_W = 256.0;
const float TRANS_H = 64.0;
const float MS_SIZE = 32.0;
const float SKY_W = 256.0;
const float SKY_H = 144.0;
struct Medium { vec3 ext; vec3 rayS; float mieS; };
Medium medium(float h, float haze) {
  float hh = max(h, 0.0);
  float dR = exp(-hh / 8.0);
  float dM = exp(-hh / 1.2);
  float dO = max(0.0, 1.0 - abs(hh - 25.0) / 15.0);
  Medium m;
  m.rayS = RAY_S * dR;
  m.mieS = MIE_S * dM * haze;
  m.ext = m.rayS + vec3(MIE_E * dM * haze) + OZO_A * dO;
  return m;
}
vec2 raySphere(vec3 ro, vec3 rd, float R) {
  float l = length(ro);
  float b = dot(ro, rd);
  float c = (l - R) * (l + R);
  float d = b * b - c;
  if (d < 0.0) return vec2(-1.0);
  float s = sqrt(d);
  return vec2(-b - s, -b + s);
}
float rhoOf(float r) { return sqrt(max(0.0, (r - RG) * (r + RG))); }
vec2 transRMuToUV(float r, float mu) {
  float H = sqrt(RT * RT - RG * RG);
  float rho = rhoOf(r);
  float disc = r * r * (mu * mu - 1.0) + RT * RT;
  float d = max(0.0, -r * mu + sqrt(max(disc, 0.0)));
  float dMin = RT - r;
  float dMax = rho + H;
  return vec2((d - dMin) / (dMax - dMin), rho / H);
}
vec2 transUVToRMu(vec2 uv) {
  float H = sqrt(RT * RT - RG * RG);
  float rho = H * uv.y;
  float r = sqrt(rho * rho + RG * RG);
  float dMin = RT - r;
  float dMax = rho + H;
  float d = dMin + uv.x * (dMax - dMin);
  float mu = 1.0;
  if (d > 0.0) mu = (H * H - rho * rho - d * d) / (2.0 * r * d);
  return vec2(r, clamp(mu, -1.0, 1.0));
}
float fromUnitToSubUvs(float u, float res) { return (u + 0.5 / res) * (res / (res + 1.0)); }
float fromSubUvsToUnit(float u, float res) { return (u - 0.5 / res) * (res / (res - 1.0)); }
float phaseRayleigh(float c) { return 3.0 / (16.0 * PI) * (1.0 + c * c); }
float phaseMie(float c, float g) {
  float g2 = g * g;
  return 3.0 / (8.0 * PI) * ((1.0 - g2) * (1.0 + c * c)) / ((2.0 + g2) * pow(max(1.0 + g2 - 2.0 * g * c, 1e-4), 1.5));
}
float phaseHG(float c, float g) {
  float g2 = g * g;
  return (1.0 - g2) / (4.0 * PI * pow(max(1.0 + g2 - 2.0 * g * c, 1e-4), 1.5));
}
bool sunBelowHorizon(float r, float mu) { return mu < -rhoOf(r) / r; }
`;

const NOISE_GLSL = /* glsl */`
uint pcgHash(uint v) {
  uint s = v * 747796405u + 2891336453u;
  uint w = ((s >> ((s >> 28u) + 4u)) ^ s) * 277803737u;
  return (w >> 22u) ^ w;
}
vec2 hashGrad(ivec2 i) {
  uint h = pcgHash(uint(i.x) * 1664525u + pcgHash(uint(i.y) + 1013904223u));
  float a = float(h) * (6.28318530718 / 4294967296.0);
  return vec2(cos(a), sin(a));
}
float gnoise(vec2 p) {
  ivec2 i = ivec2(floor(p));
  vec2 f = p - floor(p);
  vec2 u = f * f * f * (f * (f * 6.0 - 15.0) + 10.0);
  float a = dot(hashGrad(i), f);
  float b = dot(hashGrad(i + ivec2(1, 0)), f - vec2(1.0, 0.0));
  float c = dot(hashGrad(i + ivec2(0, 1)), f - vec2(0.0, 1.0));
  float d = dot(hashGrad(i + ivec2(1, 1)), f - vec2(1.0, 1.0));
  return mix(mix(a, b, u.x), mix(c, d, u.x), u.y);
}
int imod(int a, int b) { return a - b * int(floor(float(a) / float(b))); }
float pnoise(vec2 p, int period) {
  ivec2 i = ivec2(floor(p));
  vec2 f = p - floor(p);
  vec2 u = f * f * f * (f * (f * 6.0 - 15.0) + 10.0);
  ivec2 i00 = ivec2(imod(i.x, period), imod(i.y, period));
  ivec2 i10 = ivec2(imod(i.x + 1, period), i00.y);
  ivec2 i01 = ivec2(i00.x, imod(i.y + 1, period));
  ivec2 i11 = ivec2(i10.x, i01.y);
  float a = dot(hashGrad(i00), f);
  float b = dot(hashGrad(i10), f - vec2(1.0, 0.0));
  float c = dot(hashGrad(i01), f - vec2(0.0, 1.0));
  float d = dot(hashGrad(i11), f - vec2(1.0, 1.0));
  return mix(mix(a, b, u.x), mix(c, d, u.x), u.y);
}
float pfbm(vec2 p, int period, int oct) {
  float s = 0.0, a = 0.5;
  vec2 q = p;
  int per = period;
  for (int o = 0; o < 8; o++) {
    if (o >= oct) break;
    s += a * pnoise(q, per);
    q *= 2.0; per *= 2; a *= 0.5;
  }
  return s;
}
`;

// ---- thermal (LWIR 8-14 um) radiance model; every shader emits band radiance in R when uThermal = 1
// band radiance, inverse, sea/foam emissivity (complex Fresnel), sky (Idso), path, person surface model
// (written for this file by a helper agent; see the notes inside)
const THERMAL_GLSL = /* glsl */`
// ===================== LWIR physics (8-14 um) =====================
const float LWIR_C1 = 1.191042972e8;   // 2hc^2   [W m^-2 sr^-1 um^4]  (CODATA)
const float LWIR_C2 = 14387.7688;      // hc/k_B  [um K]
float lwirPlanckLam(float lam, float T) {                 // spectral radiance [W m^-2 sr^-1 um^-1]
  return LWIR_C1 / (lam * lam * lam * lam * lam * (exp(LWIR_C2 / (lam * T)) - 1.0));
}
// 1) Band radiance: 4-point Gauss-Legendre over 8-14 um (nodes 11 +- 3*x_i, weights 3*w_i).
// The integrand is smooth and single-humped, so GL4 is exact to < 3e-6 relative over 180-360 K
// (< 3e-5 for the 5772 K Sun) against a 20000-step trapezoid reference. 4 exp() per call.
float planckBand(float T) {
  T = max(T, 100.0);
  return 1.0435645354 * (lwirPlanckLam(8.4165910652, T) + lwirPlanckLam(13.5834089348, T))
       + 1.9564354646 * (lwirPlanckLam(9.9800568693, T) + lwirPlanckLam(12.0199431307, T));
}
// Inverse: Sakuma-Hattori form L = C / (exp(c2 / (A T + B)) - 1), fitted (minimax over 200-350 K)
// to planckBand(). Closed form, no iteration: |error| < 0.01 K over 200-350 K (< 0.05 K 180-400 K).
const float LWIR_SH_A = 9.3734;   // um
const float LWIR_SH_B = 173.725;  // um K
const float LWIR_SH_C = 6747.59;  // W m^-2 sr^-1
float bandTemp(float L) {
  return (LWIR_C2 / log(LWIR_SH_C / max(L, 1e-4) + 1.0) - LWIR_SH_B) / LWIR_SH_A;
}
// Surface-leaving radiance (opaque, Kirchhoff: reflectance = 1 - emissivity).
float surfaceRadianceLWIR(float T, float eps, float Lrefl) {
  return eps * planckBand(T) + (1.0 - eps) * Lrefl;
}

// 2) Sea-surface emissivity: exact Fresnel for an absorbing medium, unpolarised = (Rs+Rp)/2
// (Born & Wolf; a,b = Re/Im of the refracted wave term). Effective index n=1.242, k=0.154 was
// fitted so this single evaluation reproduces, within 0.001 for 0-85 deg, the Planck(290 K)-
// weighted band average of spectral Fresnel emissivity from Hale & Querry (1973) pure-water
// optical constants (e.g. 10 um n=1.218 k=0.051; 11 um 1.153/0.097; 12 um 1.111/0.199;
// 13 um 1.123/0.305). Seawater differs by < 0.5% (Friedman 1969). Result: 0.984 nadir,
// 0.945 at 60 deg, 0.65 at 80 deg -- consistent with Masuda et al. (1988, RSE 24:313).
// Wave-slope roughening is NOT built in: evaluate per facet with the shaded normal (the ocean
// shader's normal maps do that), which reproduces Masuda's wind dependence to first order.
const float LWIR_WATER_N = 1.242;
const float LWIR_WATER_K = 0.154;
float waterEmissivity(float cosTheta) {
  float c = clamp(cosTheta, 1e-3, 1.0);
  float s2 = 1.0 - c * c;
  float n = LWIR_WATER_N, k = LWIR_WATER_K;
  float A = n * n - k * k - s2;
  float r = sqrt(A * A + 4.0 * n * n * k * k);
  float a = sqrt(0.5 * (r + A));
  float b2 = 0.5 * (r - A);
  float Rs = ((a - c) * (a - c) + b2) / ((a + c) * (a + c) + b2);
  float st = s2 / c;                                        // sin(theta) * tan(theta)
  float Rp = Rs * ((a - st) * (a - st) + b2) / ((a + st) * (a + st) + b2);
  return 1.0 - 0.5 * (Rs + Rp);
}
// Foam/whitecap: bubble layer is a near-black volume emitter, ~0.99 and almost angle-independent
// (Niclos et al. 2007; Salisbury et al. 1993 report 0.99+ at nadir); mild drop at grazing angles.
float foamEmissivity(float cosTheta) {
  float g = 1.0 - clamp(cosTheta, 0.0, 1.0);
  return 0.99 - 0.06 * g * g * g * g;
}

// 3) Downwelling sky radiance. Vapour pressure: Magnus/Bolton (1980) e_s [hPa].
float lwirVapourPressure(float Tair, float rh) {                        // [hPa]
  float tc = Tair - 273.15;
  return clamp(rh, 0.0, 1.0) * 6.112 * exp(17.67 * tc / (tc + 243.5));
}
float lwirVapourDensity(float Tair, float rh) {                         // [g m^-3]
  return 216.7 * lwirVapourPressure(Tair, rh) / Tair;
}
// Clear-sky 8-14 um zenith emissivity relative to planckBand(Tair). Hemispheric value from
// Idso (1981, Water Resour. Res. 17:295) 8-14 um equation eps_h = 0.24 + 2.98e-8 e^2 exp(3000/T)
// (e hPa; 0.24 = CO2/O3 dry floor, e^2 = water-vapour self-continuum), capped at 0.92 because
// the fit was made in dry/temperate air and overshoots in tropical humidity. Converted to zenith
// with the Elsasser diffusivity factor 1.66: 1-eps_h = (1-eps_z)^1.66.
float lwirZenithEmissivity(float Tair, float rh) {
  float e = lwirVapourPressure(Tair, rh);
  float eh = min(0.24 + 2.98e-8 * e * e * exp(3000.0 / Tair), 0.92);
  return 1.0 - pow(1.0 - eh, 1.0 / 1.66);
}
// Slant path: plane-parallel optical depth ~ 1/sin(el); eps -> 1 (air temperature) at horizon.
float lwirSkyEmissivity(float elevSin, float Tair, float rh) {
  return 1.0 - pow(1.0 - lwirZenithEmissivity(Tair, rh), 1.0 / max(elevSin, 0.01));
}
float lwirAtmTransmittance(float elevSin, float Tair, float rh) {       // slant, to space
  return 1.0 - lwirSkyEmissivity(elevSin, Tair, rh);
}
// cloudFrac: cloud opacity along the ray (e.g. alpha of cloudLayers()). Water clouds are black
// in LWIR (eps ~1 for LWP > ~30 g m^-2); thin cirrus should pass a reduced cloudFrac. The cloud
// is seen through the sub-cloud clear path (transmittance 1-eps); above-cloud emission ignored.
// Tair..Tair-15 K for low stratus, ~230 K for cirrus; see cloudBaseTempLWIR().
float skyRadianceLWIR(float elevSin, float Tair, float rh, float cloudFrac, float Tcloud) {
  float eps = lwirSkyEmissivity(elevSin, Tair, rh);
  return eps * planckBand(Tair) + clamp(cloudFrac, 0.0, 1.0) * (1.0 - eps) * planckBand(Tcloud);
}
float cloudBaseTempLWIR(float Tair, float baseHeightM) {   // ~6.5 K/km mean tropospheric lapse
  return Tair - 0.0065 * baseHeightM;
}

// 4) Horizontal path. Window extinction is dominated by the water-vapour continuum: self term
// ~ rho_v^2 with exp(1800(1/T - 1/296)) temperature dependence (Roberts, Selby & Biberman 1976,
// Appl. Opt. 15:2085; their C_s ~ 6-14 g^-1 cm^2 atm^-1 across 8-12.5 um gives 0.14-0.32 km^-1 at
// 13 g m^-3 -- our band mean 0.17), plus a linear term (foreign continuum + weak H2O lines near
// 8 um) and a 0.05 km^-1 floor (CO2 15 um band edge at 13-14 um, maritime aerosol). Result:
// 0.08 km^-1 (cold dry) .. 0.3 km^-1 (20 C, 75% RH) .. 0.75 km^-1 (tropical). Beer's law on a band
// mean slightly overstates loss at long range (saturated lines); add fog/rain extinction as needed.
float lwirExtinctionPerKm(float rh, float Tair) {
  float rv = lwirVapourDensity(Tair, rh);
  return 0.05 + 0.0075 * rv + 0.00095 * rv * rv * exp(1800.0 * (1.0 / Tair - 1.0 / 296.0));
}
float pathTransmittanceLWIR(float distM, float rh, float Tair) {
  return exp(-lwirExtinctionPerKm(rh, Tair) * max(distM, 0.0) * 1e-3);
}
// x = tau, y = path radiance  =>  L_seen = L_target * r.x + r.y. Path assumed at Tair (isothermal).
// Do NOT apply to sky pixels: skyRadianceLWIR() already contains the whole atmospheric column.
vec2 pathTauRadLWIR(float distM, float rh, float Tair) {
  float tau = pathTransmittanceLWIR(distM, rh, Tair);
  return vec2(tau, (1.0 - tau) * planckBand(Tair));
}
float applyPathLWIR(float Ltarget, float distM, float rh, float Tair) {
  vec2 p = pathTauRadLWIR(distM, rh, Tair);
  return Ltarget * p.x + p.y;
}

// 6) Sun glint and opacity. Solar 8-14 um irradiance at TOA = Omega_sun (6.794e-5 sr) x
// planckBand(5772 K) = 1.51 W m^-2 (~0.1% of the solar constant; the Sun's real 10 um brightness
// temperature is ~5000 K, so this is an upper bound). After the slant path it is ~1 W m^-2, versus
// ~20-50 W m^-2 sr^-1 of thermal scene radiance and water reflectance 0.015 near nadir: LWIR glint
// is weak, mostly visible as faint sparkle at grazing sun/view geometry where R -> 0.3+.
// specTerm = the shader's microfacet lobe D*G/(4 cos_o) [sr^-1] WITHOUT Fresnel (what the visible
// shader multiplies sun irradiance by); fresnelR = 1 - waterEmissivity(dot(h, v)).
const float LWIR_SUN_E_TOA = 1.508;    // W m^-2, 8-14 um, at normal incidence
float sunGlintLWIR(float fresnelR, float specTerm, float sunElevSin, float Tair, float rh) {
  if (sunElevSin <= 0.0) return 0.0;
  return fresnelR * specTerm * LWIR_SUN_E_TOA * lwirAtmTransmittance(sunElevSin, Tair, rh);
}
// LWIR does NOT penetrate water: absorption coefficient ~ 4 pi k / lambda ~ 1e3 cm^-1, i.e.
// 1/e depth ~ 10-20 um. The sea surface is an opaque emitter/reflector in thermal mode: no
// refraction, subsurface or transmitted term, and anything below the waterline (submerged limbs,
// torso) must be completely invisible -- depth-test against the water surface, never blend.

// 5) Materials. Emissivities: human skin 0.98 (Steketee 1973; Sanchez-Marin et al. 2009), wet hair
// 0.97, wet fabric 0.95 (water film dominated), PU/PVC-coated nylon 0.88 (coated textiles 0.85-0.90).
const int LWIR_MAT_WATER = 0;
const int LWIR_MAT_FOAM = 1;
const int LWIR_MAT_FACE = 2;        // exposed facial skin, wet
const int LWIR_MAT_HAIR = 3;        // wet scalp hair
const int LWIR_MAT_HANDS = 4;       // wet hands / forearms above water
const int LWIR_MAT_WET_CLOTH = 5;   // soaked clothing at/just above the waterline
const int LWIR_MAT_LIFEJACKET = 6;  // inflated coated-nylon lifejacket, mostly dry upper part
float materialEmissivity(int m) {
  if (m == LWIR_MAT_WATER) return 0.984;   // nadir; use waterEmissivity(cos) per pixel
  if (m == LWIR_MAT_FOAM) return 0.99;
  if (m == LWIR_MAT_FACE || m == LWIR_MAT_HANDS) return 0.98;
  if (m == LWIR_MAT_HAIR) return 0.97;
  if (m == LWIR_MAT_WET_CLOTH) return 0.95;
  return 0.88;                              // lifejacket
}
// Sea skin temperature: cool-skin offset vs bulk (Donlon et al. 2002, J. Climate 15:353):
// dT = -0.14 - 0.30 exp(-u/3.7) K  (-0.44 K calm .. -0.16 K at 10 m/s).
float waterSkinTemp(float Twater, float windSpeed) {
  return Twater - 0.14 - 0.30 * exp(-max(windSpeed, 0.0) / 3.7);
}
// Steady surface energy balance, linearised (one equation for every person material):
//   K (Tdeep - T) + (hc + hr)(Tair - T) + hw (Twater - T) + alpha f S - Qsky
//     - w * 16.5 hc [ (1 - RH) e_s(Tair) + s (T - Tair) ] = 0
// K: tissue/insulation conductance to a deep temperature, hc = max(3, 8.3 u^0.6) forced
// convection (whole body, ISO 9920 / de Dear 1997), hr = 5.5 linearised radiation, hw = wave-wash
// coupling to water (80 W m^-2 K^-1 at the waterline -> 0 at 0.25 m), w = wetted fraction with
// Lewis relation 16.5 K/kPa, s = de_s/dT, RH assumed 0.8 (marine), Qsky = 30 W m^-2 net extra
// longwave loss to a cold sky for a ~50% sky view. f = orientation-averaged solar projection.
// Assumptions: core 37 C, person < ~1 h in water (no deep hypothermia), sunIrr = global
// horizontal irradiance (0 at night). Tuned outcomes at Tw 15 C / Ta 20 C / 5 m/s / night:
// face ~28 C (30 C at 2 m/s), hands ~21 C (cold vasoconstriction), wet hair ~18.5 C (below air,
// evaporation), wet cloth at waterline Tw + ~2 K, dry lifejacket Ta - 1 K (+5 K at 800 W m^-2).
float lwirEnergyBalance(float K, float Tdeep, float Tair, float hw, float Twater,
                        float w, float absSolar, float windSpeed) {
  const float lwirRH = 0.8;
  float hc = max(3.0, 8.3 * pow(max(windSpeed, 0.01), 0.6));
  float hcr = hc + 5.5;
  float tc = Tair - 273.15;
  float es = 0.6112 * exp(17.67 * tc / (tc + 243.5));                   // kPa
  float s = es * 17.67 * 243.5 / ((tc + 243.5) * (tc + 243.5));         // kPa/K
  float Le = w * 16.5 * hc;
  float num = K * Tdeep + hcr * Tair + hw * Twater + absSolar - 30.0
            - Le * (1.0 - lwirRH) * es + Le * s * Tair;
  return num / (K + hcr + hw + Le * s);
}
// heightAboveWaterline [m] (<0 = submerged), distToHeadCentre [m] (head radius ~0.1 m).
float personSurfaceTemp(int m, float heightAboveWaterline, float distToHeadCentre,
                        float Twater, float Tair, float windSpeed, float sunIrr) {
  float h = heightAboveWaterline;
  if (m == LWIR_MAT_WATER || h < 0.0) return waterSkinTemp(Twater, windSpeed); // opaque water wins
  if (m == LWIR_MAT_FOAM) return Twater + 0.05;   // foam destroys the cool skin: ~bulk
  float hw = 80.0 * (1.0 - smoothstep(0.0, 0.25, h));
  float S = max(sunIrr, 0.0);
  if (m == LWIR_MAT_FACE) {                        // well perfused, little vasoconstriction
    float T = lwirEnergyBalance(40.0, 309.65, Tair, hw, Twater, 0.15, 0.6 * 0.25 * S, windSpeed);
    return T - 2.5 * smoothstep(0.10, 0.13, distToHeadCentre);  // nose/ears protrude: colder
  }
  if (m == LWIR_MAT_HAIR)                          // insulating wet layer over 34 C scalp
    return lwirEnergyBalance(6.0, 307.15, Tair, hw, Twater, 1.0, 0.8 * 0.3 * S, windSpeed);
  if (m == LWIR_MAT_HANDS) {                       // cold-induced vasoconstriction below ~27 C water
    float K = mix(10.0, 30.0, smoothstep(285.0, 300.0, Twater));
    return lwirEnergyBalance(K, 309.65, Tair, hw, Twater, 0.5, 0.6 * 0.25 * S, windSpeed);
  }
  if (m == LWIR_MAT_WET_CLOTH)                     // skin under wet cloth sits between Tw and 37 C
    return lwirEnergyBalance(8.0, mix(Twater, 310.15, 0.5), Tair, hw, Twater, 1.0, 0.7 * 0.3 * S, windSpeed);
  // lifejacket: ~5-10 cm air bladder => K ~ 1.5; wet near the waterline, dry above
  float Tdry = lwirEnergyBalance(1.5, 306.15, Tair, hw, Twater, 0.1, 0.55 * 0.4 * S, windSpeed);
  float Twet = lwirEnergyBalance(1.5, 306.15, Tair, hw, Twater, 1.0, 0.55 * 0.4 * S, windSpeed);
  return mix(Twet, Tdry, smoothstep(0.05, 0.25, h));
}
// ===================== end LWIR physics =====================
`;
const THERM = {
  uThermal: { value: 0 }, uTwater: { value: 291.15 }, uTair: { value: 293.15 }, uRH: { value: 0.75 },
  uWindSpd: { value: 7 }, uSunIrr: { value: 0 }, uCloudFrac: { value: 0.4 }, uTcloud: { value: 281 },
};
const THERMAL_UNIFORMS_GLSL = 'uniform float uThermal, uTwater, uTair, uRH, uWindSpd, uSunIrr, uCloudFrac, uTcloud;\n';

// sky uniforms used by the sky, env and sun-illuminance shaders
const SKY_FUNCS = /* glsl */`
uniform sampler2D uTrans;
uniform sampler2D uSkyView;
uniform vec4 uSun;        // xyz towards the light (sun, or moon at night), w = illuminance scale
uniform float uCamAlt;    // km
uniform vec4 uCloud;      // low layer altitude km, coverage, optical density, time
uniform vec2 uVeil;       // high layer coverage, optical density
uniform vec2 uCamXZ;
uniform float uHaze;
float camR() { return RG + uCamAlt; }
vec3 sampleTrans(float r, float mu) { return texture(uTrans, transRMuToUV(r, mu)).rgb; }
vec2 skyViewUV(vec3 dir) {
  float r = camR();
  float cosBeta = rhoOf(r) / r;
  float beta = acos(clamp(cosBeta, -1.0, 1.0));
  float zha = PI - beta;
  float vzc = clamp(dir.y, -1.0, 1.0);
  vec2 uv;
  if (vzc >= -cosBeta) {
    float coord = acos(vzc) / zha;
    coord = 1.0 - sqrt(max(1.0 - coord, 0.0));
    uv.y = coord * 0.5;
  } else {
    float coord = (acos(vzc) - zha) / beta;
    uv.y = sqrt(max(coord, 0.0)) * 0.5 + 0.5;
  }
  float dl = length(dir.xz), sl = length(uSun.xz);
  float lvc = 1.0;
  if (dl > 1e-6 && sl > 1e-6) lvc = clamp(dot(dir.xz, uSun.xz) / (dl * sl), -1.0, 1.0);
  uv.x = sqrt(max(-lvc * 0.5 + 0.5, 0.0));
  return vec2(fromUnitToSubUvs(uv.x, SKY_W), fromUnitToSubUvs(uv.y, SKY_H));
}
vec3 skyAtmo(vec3 dir) { return texture(uSkyView, skyViewUV(dir)).rgb * uSun.w; }

const mat2 CLOUD_ROT = mat2(0.8, 0.6, -0.6, 0.8);
float cloudFbm(vec2 p, float fp, int oct, float billow) {
  float s = 0.0, a = 0.5, f = 1.0;
  vec2 q = p;
  for (int i = 0; i < 8; i++) {
    if (i >= oct) break;
    float w = 1.0 - smoothstep(0.08, 0.28, fp * f);
    float n = gnoise(q);
    if (i >= 2) n = mix(n, abs(n) * 2.0 - 0.38, billow);
    s += a * n * w;
    q = CLOUD_ROT * q * 2.03 + vec2(13.1, 7.7);
    f *= 2.03; a *= 0.5;
  }
  return s;
}
float lowEdge(vec2 q, float fp, int oct) {
  float t = uCloud.w;
  vec2 p = q * 0.38 + vec2(t * 0.004, t * 0.0012);
  vec2 w = vec2(gnoise(p * 0.3 + vec2(1.7, 9.2)), gnoise(p * 0.3 + vec2(8.3, 2.8)));
  p += w * 1.4;
  float base = cloudFbm(p * vec2(1.0, 1.7), fp * 0.65, oct, 0.35) * 1.3 + 0.5;
  float large = gnoise(q * 0.05 + vec2(3.1, 1.3)) * 0.7;
  return base + large - (1.0 - uCloud.y);
}
float lowSoft(float fp) { return 0.38 + min(fp * 0.65, 1.0) * 0.3; }
float lowDensity(vec2 q, float fp, int oct) { return clamp(lowEdge(q, fp, oct) / lowSoft(fp), 0.0, 1.0); }
float highEdge(vec2 q, float fp, int oct) {
  float t = uCloud.w;
  vec2 p = CLOUD_ROT * (q * vec2(0.09, 0.20)) + vec2(t * 0.0015, 0.0);
  float n = cloudFbm(p, fp * 0.2, oct, 0.0) * 1.25 + 0.5;
  float big = gnoise(q * 0.012 + vec2(7.0, 3.0)) * 0.6;
  return n + big - (1.0 - uVeil.x);
}
float highSoft(float fp) { return 0.85 + min(fp * 0.2, 1.0) * 0.3; }
float highDensity(vec2 q, float fp, int oct) { return clamp(highEdge(q, fp, oct) / highSoft(fp), 0.0, 1.0); }

struct CloudHit { float t; vec2 q; float mu; vec3 up; };
CloudHit hitLayer(vec3 dir, float hc) {
  CloudHit h;
  vec3 ro = vec3(0.0, camR(), 0.0);
  h.t = raySphere(ro, dir, RG + hc).y;
  vec3 p = ro + dir * h.t;
  h.up = p / length(p);
  h.q = p.xz + uCamXZ * 0.001;
  h.mu = max(dot(h.up, dir), 0.03);
  return h;
}
// d: density at the hit; s1..s3: densities further towards the sun
vec4 shadeLayer(vec3 dir, CloudHit h, float hc, float d, float s1, float s2, float s3, float sigma, float g) {
  float tau = d * sigma / h.mu;
  float alpha = 1.0 - exp(-tau);
  float tauS = (d * 0.5 + s1 * 0.6 + s2 + s3 * 0.7) * sigma * 0.55;
  float Ts = exp(-tauS);
  vec3 sunC = sampleTrans(RG + hc, dot(h.up, uSun.xyz)) * uSun.w;
  float cosT = dot(dir, uSun.xyz);
  float ph = mix(phaseHG(cosT, g), phaseHG(cosT, -0.2), 0.25);
  float fwd = smoothstep(-0.2, 0.7, cosT);
  vec3 single = sunC * Ts * ph * mix(alpha * 0.55, tau * exp(-tau) * 2.2, fwd) * 3.0;
  float rim = clamp((d - s1) * 2.5, 0.0, 1.0);
  float toward = smoothstep(-0.3, 0.9, cosT);
  vec3 ms = sunC * (0.025 + 0.22 * rim) * mix(1.0, Ts, 0.5) * alpha * (0.25 + 0.75 * toward);
  vec3 amb = (skyAtmo(normalize(vec3(dir.x, 0.03, dir.z))) * 0.26 + skyAtmo(vec3(0.0, 1.0, 0.0)) * 0.42)
             * alpha * (1.0 - 0.35 * d);
  return vec4(single + ms + amb, alpha);
}
// (premultiplied radiance, fraction of sky hidden); aa < 0 estimates edge width from the footprint
vec4 cloudLayers(vec3 dir, float pixAng, vec2 aa, out float sunT) {
  sunT = 1.0;
  if (dir.y <= 0.0) return vec4(0.0);
  vec2 sxz = uSun.xz / max(length(uSun.xz), 1e-4);
  vec4 lo = vec4(0.0);
  float loVis = 1.0;
  if (uCloud.y > 0.001) {
    float hc = uCloud.x;
    CloudHit h = hitLayer(dir, hc);
    float far = 1.0 - smoothstep(18.0, 36.0, h.t);
    if (far > 0.0) {
      float fp = h.t * pixAng / h.mu;
      float e = lowEdge(h.q, fp, 7);
      float d = clamp(e / lowSoft(fp), 0.0, 1.0) * far;
      if (d > 0.002) {
        float s1 = lowDensity(h.q + sxz * 0.08, fp, 5) * far;
        float s2 = lowDensity(h.q + sxz * 0.25, fp, 4) * far;
        float s3 = lowDensity(h.q + sxz * 0.6, fp, 3) * far;
        lo = shadeLayer(dir, h, hc, d, s1, s2, s3, uCloud.z, 0.75);
        float aw = aa.x >= 0.0 ? aa.x : fp * 0.38 * 2.5;
        lo *= smoothstep(0.0, aw * 1.5 + 1e-6, e);
        loVis = exp(-h.t * 0.02 * (0.6 + 0.4 * uHaze));
      }
    }
  }
  vec4 hi = vec4(0.0);
  float hiVis = 1.0;
  if (uVeil.x > 0.001) {
    float hc = 4.5;
    CloudHit h = hitLayer(dir, hc);
    float far = 1.0 - smoothstep(30.0, 70.0, h.t);
    if (far > 0.0) {
      float fp = h.t * pixAng / h.mu;
      float e = highEdge(h.q, fp, 6);
      float d = clamp(e / highSoft(fp), 0.0, 1.0) * far;
      if (d > 0.002) {
        float s1 = highDensity(h.q + sxz * 0.15, fp, 4) * far;
        float s2 = highDensity(h.q + sxz * 0.5, fp, 3) * far;
        hi = shadeLayer(dir, h, hc, d, s1, s2, s2, uVeil.y, 0.80);
        float aw = aa.y >= 0.0 ? aa.y : fp * 0.2 * 2.5;
        hi *= smoothstep(0.0, aw * 1.5 + 1e-6, e);
        hiVis = exp(-h.t * 0.010 * (0.6 + 0.4 * uHaze));
      }
    }
  }
  sunT = (1.0 - lo.a) * (1.0 - hi.a);
  vec3 col = lo.rgb * loVis + hi.rgb * hiVis * (1.0 - lo.a);
  float hidden = lo.a * loVis + hi.a * hiVis * (1.0 - lo.a);
  return vec4(col, hidden);
}
`;
