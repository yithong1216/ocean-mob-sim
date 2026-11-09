# Ocean MOB Simulator — Technical Book

How the simulator is built, and the mathematics and physics behind every part of it.

This book describes the code as it is in `src/` (the single file `ocean_mob_sim.html` is generated from it). Every
formula below is the one the code evaluates, with the constants it uses. Where the code makes a deliberate
simplification, the book says so. Chapter 23 collects these simplifications.

Units are SI throughout (metres, seconds, kelvin, radians) unless stated. The atmosphere code works in kilometres.

---

## Contents

1. [Architecture: how the program is put together](#1-architecture-how-the-program-is-put-together)
2. [The ocean surface: linear random waves and the FFT](#2-the-ocean-surface-linear-random-waves-and-the-fft)
3. [Whitecaps and foam](#3-whitecaps-and-foam)
4. [Shading the sea in visible light](#4-shading-the-sea-in-visible-light)
5. [The atmosphere, the sun and the sky](#5-the-atmosphere-the-sun-and-the-sky)
6. [Clouds and light shafts](#6-clouds-and-light-shafts)
7. [Weather: presets, sea fog, rain and lightning](#7-weather-presets-sea-fog-rain-and-lightning)
8. [People overboard: buoyancy and rigid-body dynamics](#8-people-overboard-buoyancy-and-rigid-body-dynamics)
9. [Ripples and wakes around the people](#9-ripples-and-wakes-around-the-people)
10. [Spray, splashes and spindrift](#10-spray-splashes-and-spindrift)
11. [Reflections of the people, shadows and wet skin](#11-reflections-of-the-people-shadows-and-wet-skin)
12. [Underwater optics](#12-underwater-optics)
13. [Rescue gear and the dye marker](#13-rescue-gear-and-the-dye-marker)
14. [Cameras, lens and post-processing](#14-cameras-lens-and-post-processing)
15. [The EO (RGB) sensor](#15-the-eo-rgb-sensor)
16. [The thermal-infrared (LWIR 8–14 µm) sensor](#16-the-thermal-infrared-lwir-814-µm-sensor)
17. [The LiDAR](#17-the-lidar)
18. [LiDAR file formats: PCD, LAS and LAZ](#18-lidar-file-formats-pcd-las-and-laz)
19. [Data recording](#19-data-recording)
20. [Sea condition (sea-state scale 0–9)](#20-sea-condition-sea-state-scale-09)
21. [Flight route planner and mission flight](#21-flight-route-planner-and-mission-flight)
22. [Interface, language and performance](#22-interface-language-and-performance)
23. [Simplifications and known limits](#23-simplifications-and-known-limits)
24. [References](#24-references)

---

## 1. Architecture: how the program is put together

### 1.1 One file from many sources

The program is plain JavaScript using three.js r169 (WebGL2, GLSL ES 3.00) and lil-gui 0.19, both loaded from a CDN.
There is no bundler or package manager. `build.mjs` (Node 18+, no dependencies) concatenates the sources into one
module script inside `src/page.html` and writes `ocean_mob_sim.html`:

```
src/core/00_header.js ... 09_sensors.js     engine (sorted by file name)
src/features/*.js                           feature modules (sorted by file name)
src/core/99_main.js                         main loop and start-up
```

`import` lines are lifted out of every chunk, merged per module and placed once at the top. Everything else shares
one module scope. A feature file can therefore use every engine object directly. The order matters: features
named `zz_*` load last, because they decorate the GUI or wrap other features.

| File | What it does |
|---|---|
| `core/00_header.js` | licence and credits, imports, constants (`G = 9.81`, `RHO = 1025`, cascade sizes …), `params` |
| `core/01_i18n.js` | translation table and `i18n()` |
| `core/01_setup.js` | renderer, cameras, post chain, extension hooks, full-screen pass helpers |
| `core/02_glsl_common.js` | shared GLSL: atmosphere maths, noise, the complete LWIR radiometry, 2D cloud layers |
| `core/03_atmosphere.js` | transmittance / multiple-scattering / sky-view lookup tables, sky dome, reflection cube |
| `core/04_ocean_sim.js` | wave spectrum, FFT, foam map, surface probes |
| `core/05_ocean_surface.js` | ocean mesh and ocean shader |
| `core/06_sky_state.js` | sun and moon position, sky refresh, sunlight readback |
| `core/07_people.js` | human models, posture, lifejacket, wet / thermal materials, buoyancy dynamics |
| `core/08_gui.js` | the main control panel |
| `core/09_sensors.js` | EO / IR gimbal camera, microbolometer model, automatic gain control |
| `features/sky_*.js` | volumetric clouds, god rays, weather |
| `features/water_*.js` | ripple / wake field, spray particles |
| `features/refl_*.js` | reflections of the people, shadows, wet skin |
| `features/uw_*.js` | underwater view, rescue gear, dye marker |
| `features/cine_*.js` | camera presets, lens and cinematic post-processing |
| `features/sensor_lidar*.js` | LiDAR simulation and its file formats |
| `features/sea_state.js` | the 0–9 sea-condition scale |
| `features/ui_mode.js`, `zz_language.js`, `zz_platform.js` | Simple / Professional panels, language switch, quality and FPS |
| `features/zz_recorder.js` | data recording |
| `features/zz_route.js` | flight route planner and mission flight |

### 1.2 Extension points

The engine never refers to a feature by name. Features attach themselves in two ways:

- **Frame hooks** (`HOOKS` in `01_setup.js`). Each hook is a list of callbacks:
  - `update`: after the core physics;
  - `lateUpdate`: after the camera is final for this frame;
  - `beforeRender` and `afterRender`;
  - `resize`;
  - `personShader`: adds code to every person and gear material;
  - `ready`: runs once at start-up.
- **Shader injection** (`injectShader(material, hook, code)`). The ocean and sky shaders contain marker comments such
  as `// @hook:ocean-frag-slopes`, and a feature inserts GLSL after the marker line. The ripples, for example, add
  their slopes to the wave slopes at `ocean-frag-slopes`. The LiDAR replaces the whole ocean output at
  `ocean-frag-normal` when its range pass is running.

A third extension point, `cameraDriver`, lets a feature take over the main camera from the mouse orbit controls.
The route planner uses it.

### 1.3 Coordinates

three.js is right-handed with **y up**. Compass bearings map to the horizontal plane through

$$
\text{bearing }\beta \;\mapsto\; (x, z) = (\sin\beta,\; -\cos\beta),
$$

so **x is east, −z is north**. A wind "from 300°" blows toward 120°. The LiDAR export uses either the sensor frame
(x forward, y left, z up, ROS REP-103) or the world frame (x east, y north, z up).

### 1.4 One frame

All heavy work is on the GPU. The CPU reads back only a few small textures: surface probes, sunlight, the AGC
statistics and the LiDAR counters.

```
update(dt)
  sky LUTs (only when the sun, haze or weather changed)
  wave spectrum evolution + inverse FFT        5 cascades x (1 + 16) full-screen passes on 256 x 256
  surface probes  -> readback 256 x 3 texels    height, slope, velocity under every body
  rigid-body physics of the people             fixed 1/120 s substeps on the sampled surface
  posture, waterline planes, foam map update
  HOOKS.update        ripples, spray, weather, rescue gear, underwater state, sea condition, recorder clock
  camera              cameraDriver (route planner / flight) or orbit + follow, kept above the water
  uniforms, reflection cube (every 0.5 s), thermal environment, gimbal pose
  HOOKS.lateUpdate    cloud uniforms, camera presets, shadow frustum ...
render()
  HOOKS.beforeRender  cloud raymarch + temporal resolve, shadow maps, TAA jitter
  people-only pre-pass -> refraction target (colour + depth)
  composer: scene -> [god rays, underwater volume, cinematic passes] -> bloom -> composite (AgX)
  sensor windows: RGB view, LWIR view (+ AGC every 3rd frame)
  HOOKS.afterRender   LiDAR scan + point window, recorder capture, route overlay
```

The time step is `min(real dt, 0.05 s, stepLimit)`. `stepLimit` lets the recorder hold the simulation so that no
frame is skipped (Chapter 19).

---

## 2. The ocean surface: linear random waves and the FFT

The sea is a **Gaussian random field**: a sum of a very large number of independent linear waves with random phases,
whose energy per wavenumber follows an oceanographic spectrum. This is the method of Tessendorf (2001). Here it uses a
fetch-limited JONSWAP spectrum with Donelan–Banner directional spreading plus a swell system. The ocean simulation,
the shading model and the atmosphere LUTs are adapted from *web-ocean* (MIT licence) and ported from WGSL to GLSL.

### 2.1 Dispersion

A linear wave of wavenumber vector $\mathbf k$ ($k = \lvert\mathbf k\rvert$) on deep water oscillates at

$$
\omega(k) = \sqrt{g k + \frac{\sigma_s}{\rho}\,k^3}, \qquad \frac{\sigma_s}{\rho} = 7.4\times10^{-5}\ \mathrm{m^3\,s^{-2}},
$$

gravity plus surface tension. The capillary term matters only below about 2 cm wavelength. The group velocity, which
is needed to convert the frequency spectrum into a wavenumber spectrum, is

$$
\frac{d\omega}{dk} = \frac{g + 3(\sigma_s/\rho)\,k^2}{2\,\omega}.
$$

### 2.2 The wind-sea spectrum (JONSWAP, fetch-limited)

For a 10 m wind speed $U$ (m/s) blowing over a fetch $F$ (m):

$$
S(\omega) = \frac{\alpha g^2}{\omega^5}\exp\!\left[-\frac54\left(\frac{\omega_p}{\omega}\right)^4\right]\,
\gamma^{\exp\left[-\frac{(\omega-\omega_p)^2}{2\sigma^2\omega_p^2}\right]},
$$

$$
\alpha = 0.076\left(\frac{U^2}{gF}\right)^{0.22},\qquad
\omega_p = 22\left(\frac{g^2}{U F}\right)^{1/3},\qquad
\gamma = 3.3,\qquad
\sigma = \begin{cases}0.07 & \omega\le\omega_p\\ 0.09 & \omega>\omega_p\end{cases}
$$

These are the JONSWAP fetch laws (Hasselmann et al. 1973). A short fetch gives a higher, sharper peak at a higher
frequency; a long fetch tends towards a fully developed sea.

### 2.3 Directional spreading (Donelan–Banner)

Energy is spread around the wind direction $\theta_w$ with a $\mathrm{sech}^2$ law whose width depends on frequency
(Donelan, Hamilton & Hui 1985; high-frequency branch after Banner 1990):

$$
D(\theta) = \frac{\beta}{2\tanh(\beta\pi)}\,\mathrm{sech}^2\!\big(\beta(\theta-\theta_w)\big),
$$

$$
\beta = \begin{cases}
2.61\,(\omega/\omega_p)^{1.3} & \omega/\omega_p < 0.95\\
2.28\,(\omega/\omega_p)^{-1.3} & 0.95 \le \omega/\omega_p < 1.6\\
10^{-0.4 + 0.8393\exp\left[-0.567\ln\left((\omega/\omega_p)^2\right)\right]} & \text{otherwise}
\end{cases}
$$

The "Choppiness (spread)" slider divides $\beta$ by $0.5 + \text{choppiness}$, which widens the spread and gives a more
confused sea.

### 2.4 Swell

A second, narrow JONSWAP ($\gamma = 6$) with peak $\omega_s = \sqrt{2\pi g/\lambda_s}$ from the swell wavelength. Its
level comes from the requested swell height $H_s$ through Goda's approximation:

$$
\alpha_s = \frac{5}{16}\,\frac{H_s^2\,\omega_s^4}{g^2}\,(1 - 0.287\ln\gamma).
$$

It is spread with a narrow cosine-power law $D_s(\theta) = q\,\lvert\cos(\theta/2)\rvert^{2s}$, $s = 48$, normalised by
$q = \sqrt{s + 1/4}\,/\,(2\sqrt\pi)$, the large-$s$ form of $\Gamma(s+1)/(2\sqrt\pi\,\Gamma(s+\tfrac12))$.

### 2.5 From $S(\omega,\theta)$ to amplitudes on a grid

The wavenumber spectrum is

$$
\Psi(\mathbf k) = \big[S(\omega)D(\theta) + S_s(\omega)D_s(\theta)\big]\,\frac{d\omega}{dk}\,\frac1k\;
e^{-k^2\ell^2}\;G, \qquad \ell = 1.2\ \mathrm{mm},
$$

where $1/k$ is the polar-coordinate Jacobian, $e^{-k^2\ell^2}$ removes waves shorter than a few millimetres, and $G$
is the square of the "Wave height ×" slider.

On an $N\times N$ grid ($N = 256$) covering a patch of side $L$ (spacing $\Delta k = 2\pi/L$), each wavevector gets a
complex Gaussian amplitude

$$
\tilde h_0(\mathbf k) = \tfrac12\,(\xi_r + i\,\xi_i)\,\sqrt{\Psi(\mathbf k)}\;\Delta k,\qquad \xi_r,\xi_i\sim\mathcal N(0,1),
$$

drawn with a PCG hash and Box–Muller, so the same seed gives the same sea. With the time-dependent field

$$
\tilde h(\mathbf k, t) = \tilde h_0(\mathbf k)\,e^{-i\bar\omega t} + \tilde h_0^*(-\mathbf k)\,e^{+i\bar\omega t},
$$

the height field is real, and $\sum\lvert\tilde h\rvert^2 \approx \int\Psi\,d^2k$, so the variance is right.

**Seamless looping.** Every frequency is rounded down to a multiple of $\omega_0 = 2\pi/600\ \mathrm{s^{-1}}$:
$\bar\omega = \lfloor\omega/\omega_0\rfloor\,\omega_0$. All components then have periods that divide 600 s, so the sea
repeats exactly every 10 minutes and the time argument can be wrapped without a jump.

### 2.6 Everything the surface needs, from one spectrum

The inverse FFT turns each spectral field into a spatial one, $f(\mathbf x) = \sum_{\mathbf k}\tilde f(\mathbf k)\,e^{i\mathbf k\cdot\mathbf x}$.
The shader builds these spectral fields at once:

| Field | Spectral form | Used for |
|---|---|---|
| height $h$ | $\tilde h$ | vertical displacement |
| horizontal displacement $\mathbf D$ | $-i\lambda\,(\mathbf k/k)\,\tilde h$ | "choppy" sharp crests, flat troughs |
| slope $\nabla h$ | $i\,\mathbf k\,\tilde h$ | normals |
| displacement derivatives $\partial D_x/\partial x$, $\partial D_z/\partial z$, $\partial D_x/\partial z$ | $\lambda\,k_x^2/k\,\tilde h$, $\lambda\,k_z^2/k\,\tilde h$, $\lambda\,k_xk_z/k\,\tilde h$ | Jacobian, slope correction |
| vertical velocity $\partial h/\partial t$ | $-i\bar\omega\big(\tilde h_0e^{-i\bar\omega t} - \tilde h_0^*e^{i\bar\omega t}\big)$ | buoyancy, drag, spray |
| horizontal orbital velocity $\mathbf u$ | $i\,(\mathbf k/k)\,\partial\tilde h/\partial t$ | drag on the bodies |

$\lambda$ is the "Steepness (choppy disp.)" slider. The shader evaluates the displacement sum with the sign that
matches its own transform convention, so the crests sharpen. A surface point that started at $\mathbf x_0$ is moved to

$$
\mathbf p(\mathbf x_0, t) = \big(x_0 + D_x,\; h,\; z_0 + D_z\big).
$$

**Jacobian.** The horizontal map $\mathbf x_0\mapsto\mathbf x_0 + \mathbf D$ has determinant

$$
J = \Big(1 + \frac{\partial D_x}{\partial x}\Big)\Big(1 + \frac{\partial D_z}{\partial z}\Big) - \Big(\frac{\partial D_x}{\partial z}\Big)^2 .
$$

$J < 1$ means the surface is being compressed. $J \to 0$ means it folds over itself, which is where a real wave
breaks. This drives the foam (Chapter 3).

**Slope after displacement.** The slope of the *displaced* surface divides each slope by the stretch factor:
$s'_x = s_x/(1 + \partial D_x/\partial x)$, likewise for $z$, clamped at 0.15. The normal is
$\mathbf n \propto (-s'_x,\,1,\,-s'_z)$.

### 2.7 Cascades

One 256² grid cannot cover both 100 m swell and centimetre ripples. Five grids (cascades) of different patch sizes
are summed:

$$
L_c = 1103,\ 211.3,\ 39.7,\ 7.43,\ 1.393\ \mathrm m .
$$

The sizes are not multiples of one another, so the tiling of one cascade does not line up with another. Each cascade
keeps only its own band:

$$
k \in \Big[\tfrac{2\pi\cdot 6}{L_c},\ \tfrac{2\pi\cdot 6}{L_{c+1}}\Big),\quad
\text{first cascade from } \pi/L_0,\ \text{last up to } 0.98\,\pi N/L_4 .
$$

The finest resolvable wavelength is $2L_4/N \approx 1.1$ cm. In the shading, the slopes of the two finest cascades
also respond to "gusts": slowly drifting patches where the ripples are 0.35–1.2× as strong (cat's paws).

### 2.8 The GPU FFT

Each cascade runs one evolve pass and then a 2D inverse FFT. The FFT is 8 horizontal plus 8 vertical radix-2
**Stockham** passes. Stockham ordering keeps both input and output in natural order, so no bit-reversal pass is
needed. Six complex fields travel through three RGBA float targets (multiple render targets). The last pass writes
four half-float textures with mipmaps:

- `DISP` = $(D_x, h, D_z, \partial h/\partial t)$;
- `DERIV` = $(s_x, s_z, \partial D_x/\partial x, \partial D_z/\partial z)$;
- `MOM` = $(s_x^2, s_z^2, s_x s_z, \partial D_x/\partial z)$, the second slope moments used by LEAN shading (Chapter 4);
- `VEL` = $(u_x, u_z)$.

That is $5\times(1+16) = 85$ full-screen 256² passes per frame.

### 2.9 The ocean mesh

The mesh is a polar grid centred under the camera: 380 rings by 512 segments. Ring radii are spaced geometrically
from 5 cm to 20 km, so vertex density follows the screen-space detail. The vertex shader samples each cascade at a
mip level that matches the local vertex spacing,

$$
\text{lod}_c = \log_2\!\big(\max(\Delta x / (L_c/N),\ 1)\big) + 0.5,
$$

and skips cascades that are already averaged away. Beyond about 12–19 km the sea is fully fogged before the mesh
ends.

### 2.10 Sea-state numbers

The HUD and the physics use statistics of the actual spectrum:

$$
m_0 = \int S(\omega)\,d\omega \;\;(\text{numerically, } 0.01\ldots8\ \mathrm{rad/s}),\qquad
H_{s,\text{wind}} = 4\sqrt{m_0}\cdot\text{(height ×)},
$$

$$
H_s = \sqrt{H_{s,\text{wind}}^2 + H_{s,\text{swell}}^2},\qquad
T_p = \frac{2\pi}{\omega_p},\qquad
\lambda_p = \frac{2\pi g}{\omega_p^2},\qquad
k_p = \frac{\omega_p^2}{g}.
$$

**Surface drift.** Floating objects drift at about 3 % of the wind speed. The model uses Stokes drift (≈1.25 % $U$)
plus the wind-driven current (≈1.5 % $U$), a total of $\mathbf U_c = 0.0275\,U\,\hat{\mathbf w}$ downwind.

### 2.11 Sampling the surface for physics (probes)

The physics needs the water height under a point $\mathbf x$, but the FFT gives height at the *undisplaced*
position $\mathbf x_0$. The probe pass inverts the choppy map by fixed-point iteration (five steps, enough for
moderate steepness):

$$
\mathbf x_0^{(n+1)} = \mathbf x - \mathbf D\big(\mathbf x_0^{(n)}\big).
$$

At $\mathbf x_0$ it returns:

- the height;
- the corrected slope;
- $J$;
- the vertical and orbital velocity;
- $\mathbf x_0$ itself.

Up to 256 probes run in one pass and are read back as a 256×3 float texture. Probe slots are shared:

| Probes | Used by |
|---|---|
| 0–239 | people (12 each) |
| 240–245 | camera presets |
| 246–254 | rescue gear |
| 255 | camera |

---

## 3. Whitecaps and foam

### 3.1 Where foam is born

Foam is injected where the surface folds: where the Jacobian drops below a threshold,

$$
\text{inject} = \mathrm{clamp}\big((J_\text{thr} - J)\cdot 2,\ 0,\ 1\big),\qquad J_\text{thr} = 0.52 + 0.16\cdot\text{(whitecaps ×)} .
$$

### 3.2 A persistent foam map

Near the camera target, a 1024² map covering 128 m (12.5 cm texels) keeps foam alive after the crest has passed.
Each step:

$$
f \leftarrow \min\!\Big(\max\big(f\,e^{-0.22\,\Delta t},\ \text{inject},\ \text{body}\big) + \text{inject}\cdot\Delta t,\ 1.6\Big)
$$

- The decay rate gives foam an e-folding time of about 4.5 s.
- "body" is a broken ring of bubbles at each person's waterline, stronger when the person moves through the water.
- A second channel holds "fresh" foam, $g \leftarrow \max(g\,e^{-1.6\Delta t},\ \text{inject})$, which is brighter and
  denser.
- The map is **advected semi-Lagrangian**: each texel looks up where its foam was one step ago, at
  $\mathbf x - \mathbf u_f\Delta t$, with $\mathbf u_f = 0.03\,U$ downwind. This is why streaks trail downwind.

Beyond the map, the shader shows only instantaneous crest foam from $J$.

### 3.3 What the foam looks like

A procedural 512² tileable texture is built once. It contains:

- domain-warped periodic Perlin fBm for large lace;
- two Worley "bubble" layers;
- a soft density channel;
- an anisotropic streak channel, stretched along the wind.

The foam coverage $c$ is turned into a mask with a moving threshold, $\mathrm{smoothstep}(1-c-w,\ 1-c+w,\ \text{pattern})$.
Thin coverage therefore shows only the brightest lace, and heavy coverage fills in. Foam is lit as a diffuse surface
with albedo ≈0.85.

### 3.4 How much foam

Whitecap coverage grows steeply with wind (Monahan & O'Muircheartaigh 1980):

$$
W(U) = 3.84\times10^{-6}\,U^{3.41}.
$$

The sea-condition presets set the "Whitecaps ×" slider to $1 + 1.1\log_{10}\!\big(W(U)/W(7)\big)$, clamped to
[0, 3], so the slider value 1 matches 7 m/s.

---

## 4. Shading the sea in visible light

Each ocean pixel combines five things:

$$
L = \underbrace{(1-F)\,L_\text{body}}_{\text{water body}} + \underbrace{F\,L_\text{sky}}_{\text{reflection}} + \underbrace{L_\text{glint}}_{\text{sun}} \;\to\; \text{foam mix} \;\to\; \text{aerial perspective}.
$$

### 4.1 Fresnel

Schlick's approximation with $F_0 = 0.02$, since $((n-1)/(n+1))^2 = 0.0204$ for $n = 1.333$:

$$
F(\cos\theta) = 0.02 + 0.98\,(1-\cos\theta)^5 .
$$

### 4.2 LEAN slope statistics

A pixel far away covers many waves. The surface inside it is described by the **mean and covariance of the slopes**
(LEAN mapping, Olano & Baker 2010). The mipmapped `DERIV` and `MOM` textures give the mean $\bar{\mathbf s}$ and the
second moments over the pixel footprint. Summed over the cascades:

$$
\Sigma = \begin{pmatrix}\overline{s_x^2} - \bar s_x^2 & \overline{s_xs_z} - \bar s_x\bar s_z\\ \cdot & \overline{s_z^2} - \bar s_z^2\end{pmatrix}
+ \sigma_u^2\,I + \Sigma_\text{AA}.
$$

- $\sigma_u^2 = 0.0024 + 0.0035\,\mathrm{smoothstep}(30, 500, d)$ is roughness the FFT does not resolve. It grows with
  distance $d$.
- $\Sigma_\text{AA} = \tfrac12\big(\partial_x\bar{\mathbf s}\,\partial_x\bar{\mathbf s}^\top + \partial_y\bar{\mathbf s}\,\partial_y\bar{\mathbf s}^\top\big)$,
  capped, is the slope change across the pixel, taken from screen derivatives. This is *specular anti-aliasing*: it
  turns shimmering sub-pixel glints into a stable, wider glitter.

### 4.3 Sun glitter: a microfacet model on Gaussian slopes

The sun reflects into the eye from facets whose normal is the half vector $\mathbf h = \mathrm{norm}(\mathbf v + \mathbf l)$.
Such a facet has slope $\mathbf s_h = -\mathbf h_{xz}/h_y$. With the slopes distributed as $\mathcal N(\bar{\mathbf s}, \Sigma)$:

$$
P(\mathbf s_h) = \frac{\exp\!\big(-\tfrac12(\mathbf s_h-\bar{\mathbf s})^\top\Sigma^{-1}(\mathbf s_h-\bar{\mathbf s})\big)}{2\pi\sqrt{\det\Sigma}},\qquad
D(\mathbf h) = \frac{P(\mathbf s_h)}{\cos^4\theta_h}.
$$

This is the Cox & Munk (1954) glitter model in microfacet form. The masking–shadowing term is Smith's, with the
rational fit for a Gaussian (Beckmann) slope distribution (Walter et al. 2007):

$$
G = \frac{1}{1 + \Lambda(\mathbf v) + \Lambda(\mathbf l)},\qquad
\Lambda = \begin{cases}\dfrac{1 - 1.259a + 0.396a^2}{3.535a + 2.181a^2} & a < 1.6\\ 0 & a\ge1.6\end{cases},\qquad
a = \frac{\cot\theta}{\sqrt2\,\sigma_\phi},
$$

where $\sigma_\phi^2 = \hat{\boldsymbol\phi}^\top\Sigma\,\hat{\boldsymbol\phi}$ is the slope variance in the azimuth of
that direction. The glint radiance is

$$
L_\text{glint} = E_\odot\;\frac{F(\mathbf v\cdot\mathbf h)\,D\,G}{4\cos\theta_v},
$$

with $E_\odot$ the sunlight that reaches the sea (Section 5.6), capped at $6\times10^4$ to keep the HDR buffer finite.

### 4.4 Sky reflection

The reflected sky is sampled from the environment cube in direction $\mathbf r = \mathrm{reflect}(-\mathbf v, \mathbf n')$.
The mip level grows with roughness: $\log_2(2\sigma\cdot\text{texels per radian})$. At grazing angles the facets you
can see are biased towards facing you, so the normal is tilted towards the viewer:

$$
\mathbf n' = \mathrm{norm}\big(\mathbf n + 0.32\,\sigma_v\,(1 - \mathrm{smoothstep}(0, 2.5, a_v))\,\hat{\mathbf v}_h\big).
$$

Reflected rays that point below the horizon are folded upwards. The Fresnel weight uses a normal pulled 45 % towards
vertical, which avoids black rims on steep facets.

### 4.5 The water body

Upwelling light is modelled as a scattering colour times the hemispherical sky and sun light. Thin crests lit from
behind glow green, a subsurface-scattering term:

$$
L_\text{sss} = c_\text{sss}\,E_\odot\,s\;\text{crest}^2\;\max(\mathbf l\cdot(-\mathbf v),0)^4\;\Big(\tfrac{1 - \mathbf l\cdot\mathbf n}{2}\Big)^3\,e^{-d/600}.
$$

- $\text{crest} = \mathrm{clamp}((h - 0.05)/0.9)$ is high only on wave tops.
- The power-4 term needs the sun to be behind the wave.
- The cube term favours thin water.

### 4.6 Bodies seen through the surface

The submerged parts of the people are drawn first into their own colour and depth target, a people-only pre-pass.
The ocean shader then looks into that target at a screen position offset by the surface normal, which approximates
refraction. It attenuates by Beer–Lambert along the water path:

$$
T = \exp\!\big(-\mathbf a\,(2\ell + 0.25)\big)\cdot\text{reach},\qquad \ell = \sqrt{d^2 + d_h^2},
$$

- $\mathbf a = (0.6, 0.3, 0.32)\times(0.6 + 0.4\,\text{turbidity})\ \mathrm{m^{-1}}$ is the absorption plus scattering
  loss.
- "reach" fades out what a ray refracted at this surface point could not see. Snell's window is about ±48.6° in the
  water.
- Deeper parts are mixed with a second, more offset sample, which imitates forward-scattering blur.

### 4.7 Aerial perspective over the sea

Haze thins with height (scale height $H = 1.2$ km). The average density along a ray from camera height $h_c$ is
$\bar\rho = (1 - e^{-h_c/H})/(h_c/H)$. The transmittance and the in-scattered colour are

$$
T = \exp\!\big(-\max(\boldsymbol\sigma, \sigma_\text{floor})\,d\,\bar\rho\big),\qquad
L \leftarrow L\,T + L_\text{sky}(\text{mirrored ray})\,(1-T),
$$

- $\boldsymbol\sigma = (\boldsymbol\beta_R + 4.4\times10^{-3}\,\text{haze})\cdot10^{-3}\cdot\text{fog}$ per metre (RGB).
- The floor $\sigma_\text{floor}$ is the sea-fog extinction (Chapter 7).
- The in-scattered colour is the sky in the direction mirrored about the horizon. It has nearly the same scattering
  angle to the sun as the real downward ray.

---

## 5. The atmosphere, the sun and the sky

The sky is a physically based single- plus multiple-scattering atmosphere computed into small lookup tables, after
Hillaire (2020), with the parameterisations of Bruneton & Neyret (2008).

### 5.1 The medium

The planet has radius $R_G = 6360$ km and the atmosphere top is $R_T = 6460$ km. At height $h$ (km):

$$
\boldsymbol\sigma_s^R(h) = \boldsymbol\beta_R\,e^{-h/8},\quad
\sigma_s^M(h) = 3.996\times10^{-3}\,\eta\,e^{-h/1.2},\quad
\boldsymbol\sigma_e = \boldsymbol\sigma_s^R + 4.40\times10^{-3}\,\eta\,e^{-h/1.2} + \boldsymbol\beta_O\,\max\!\Big(0, 1 - \frac{\lvert h-25\rvert}{15}\Big)
$$

with Rayleigh $\boldsymbol\beta_R = (5.802, 13.558, 33.1)\times10^{-3}\ \mathrm{km^{-1}}$ (RGB), the "Haze" slider
$\eta$ scaling the aerosols (Mie), and ozone absorption $\boldsymbol\beta_O = (0.650, 1.881, 0.085)\times10^{-3}$ in a
tent profile around 25 km. Ozone is what keeps the twilight sky blue.

### 5.2 Phase functions

$$
p_R(\cos\theta) = \frac{3}{16\pi}(1+\cos^2\theta),\qquad
p_M(\cos\theta) = \frac{3}{8\pi}\,\frac{(1-g^2)(1+\cos^2\theta)}{(2+g^2)(1+g^2-2g\cos\theta)^{3/2}},\ g = 0.84
$$

The Mie phase is Cornette–Shanks. Clouds and water use Henyey–Greenstein,
$p_{HG} = \dfrac{1-g^2}{4\pi(1+g^2-2g\cos\theta)^{3/2}}$.

### 5.3 The lookup tables

| LUT | Size | Content |
|---|---|---|
| Transmittance | 256×64 | $T(r,\mu) = \exp\!\big(-\int\sigma_e\,dt\big)$ from radius $r$ in direction $\cos\theta = \mu$ to the top (64 steps) |
| Multiple scattering | 32×32 | $\Psi_{ms}(h, \mu_\odot)$ |
| Sky view | 256×144 | sky radiance around the camera, as a function of zenith and azimuth to the sun |

**Multiple scattering** (Hillaire 2020). For each height and sun angle, 64 directions on the sphere are integrated
(20 steps each) for the second-order scattered light $L_2$ and the transfer factor $f_{ms}$, the fraction of light
scattered again. The infinite series of higher orders sums geometrically:

$$
\Psi_{ms} = \frac{L_2}{1 - f_{ms}} .
$$

The sea below reflects with albedo 0.06.

**Sky view.** The radiance along each view ray is integrated in 48 steps, quadratically spaced so that steps are
finer near the camera:

$$
L = \int_0^{t_\max} T(0,t)\,\Big[T_\odot(t)\big(\boldsymbol\sigma_s^R p_R + \sigma_s^M p_M\big) + \Psi_{ms}\,(\boldsymbol\sigma_s^R + \sigma_s^M)\Big]\,dt .
$$

Within each step, the in-scattering $S$ is integrated *analytically* against the step's own extinction. This is
Hillaire's energy-conserving form, and it stays correct even when one step is optically thick:

$$
\int_0^{\Delta t} S\,e^{-\sigma_e s}\,ds = \frac{S\,(1-e^{-\sigma_e\Delta t})}{\sigma_e}.
$$

The latitude axis uses Hillaire's non-linear mapping, which puts more texels near the horizon. The longitude axis is
the angle to the sun, so the table is symmetric about the sun's vertical plane.

### 5.4 The sun's position

Latitude $\varphi = 32°$ and solar declination $\delta = 12°$ are fixed (a spring or late-summer day at mid-latitude).
With hour angle $H = 15°\,(t - 12)$, where $t$ is the time of day in hours:

$$
\sin e = \sin\varphi\sin\delta + \cos\varphi\cos\delta\cos H,\qquad
A = \operatorname{atan2}\!\big(\sin H,\ \cos H\sin\varphi - \tan\delta\cos\varphi\big) + \pi .
$$

- **Night.** The factor $n_f = 1 - \mathrm{smoothstep}(-0.12, -0.02, \sin e)$ drives:
  - stars, hashed points in direction cells;
  - an exposure lift of $(1 + 3n_f)$, a simple form of eye adaptation;
  - stronger lifejacket strobes.
- **Moon.** Below −3° solar elevation the moon lights the scene instead. It sits opposite the sun's azimuth at 38°
  elevation, with 0.4 % of the sun's illuminance.

### 5.5 The sun disc

The angular radius is $0.004675$ rad (0.268°). Its radiance is $E/(\pi r_\odot^2)$, times atmospheric
transmittance, with linear limb darkening per colour channel:

$$
\frac{I(\mu)}{I(1)} = 1 - u\,(1-\mu),\quad \boldsymbol u = (0.397, 0.503, 0.652),\quad \mu = \sqrt{1 - (\theta/r_\odot)^2}.
$$

### 5.6 Sunlight at the sea and image-based lighting

A 1×1 pass computes the sunlight reaching the sea at the camera target. It is the transmittance from the
LUT times the cloud transmittance, averaged over 7 points across the solar disc. This gives soft cloud shadows. The
result $E_\odot$ is read back to the CPU and sets the colour and intensity of the directional light that lights the
people.

The sky is also rendered into a 256² cube map every 0.5 s. Below the horizon the cube holds what a reflected ray
would see: a far, hazy ocean, not the ground. The cube is prefiltered with PMREM for image-based lighting.

---

## 6. Clouds and light shafts

### 6.1 Flat cloud layers (engine)

The engine has two thin layers, low cumulus at a set altitude and a high veil at 4.5 km. Each is a 2D fBm with domain
warping and "billows" ($\lvert n\rvert$ folding in the higher octaves). The fBm is evaluated where the view ray hits a
sphere of that radius. Shading uses the optical depth $\tau = d\,\sigma/\mu$ ($\mu$ = cosine to the layer normal),
coverage $\alpha = 1 - e^{-\tau}$, a few density samples towards the sun for self-shadowing, a two-lobe HG phase, a
multiple-scattering rim term and sky ambient. These layers are replaced by the volumetric clouds when those are on.

### 6.2 Volumetric clouds (`sky_clouds.js`)

The clouds fill a shell of the spherical atmosphere, by default from 1.2 to 3.4 km. They are raymarched in the
style of Schneider (2015). The pieces are:

- **Noise**, built once on the GPU:
  - a 128³ tileable Perlin–Worley texture (billowy Perlin remapped by inverted Worley) plus three Worley octaves, for
    the shape;
  - a 32³ Worley texture for the detail.
- **Weather map**, a 40 km tile. It holds coverage, cloud type (stratus → cumulus) and storm cells. Coverage and type
  are histogram-equalised on the CPU, so the coverage slider maps linearly to the fraction of sky that can hold cloud.
- **Density.** With $h_f$ the normalised height in the layer and $c$ the local coverage:

$$
\begin{aligned}
\text{hp} &= \mathrm{smoothstep}\big(0,\ \mathrm{mix}(0.32, 0.07, \text{type}),\ h_f\big)\,\big(1 - \mathrm{smoothstep}(\mathrm{mix}(0.4,0.6,\text{type}),\ 1,\ h_f)\big)\\
\text{sh} &= \mathrm{clamp}\big(1.9\,(\mathrm{remap}(s_r,\ w_f - 1,\ 1,\ 0,\ 1) - 0.42)\big)\\
d_\text{base} &= c\cdot\mathrm{remap}(\text{sh}\cdot\text{hp},\ 1-c,\ 1,\ 0,\ 1)\\
d &= \mathrm{remap}\big(d_\text{base},\ e\cdot d_m,\ 1,\ 0,\ 1\big),\quad d_m = \mathrm{mix}(w_d,\ 1-w_d,\ \mathrm{clamp}(6h_f))
\end{aligned}
$$

  - The height profile `hp` makes cumulus round-bottomed and tall, and stratus flat.
  - The detail Worley $w_d$ erodes the edges, giving wispy bases and billowy tops.
  - The tops lean downwind.
  - The extinction coefficient is $\sigma = 60\,d$ km⁻¹ times the density slider.
- **Lighting**, at each march step:

$$
S = E_\odot T_\text{air}\Big(\underbrace{\sum_{i=0}^{2} a^i\,p\big(\theta;\,g\,b^i\big)\,e^{-c_i\tau_l}}_{\text{multiple-scattering octaves}}\cdot\text{powder} + \frac{0.22}{1 + 0.25\tau_l}\Big) + L_\text{amb}(h_f)
$$

  - The octaves follow Wrenninge (2013): $a = b = 0.5$, $c = (1, 0.35, 0.12)$.
  - The phase function is dual-lobe HG, $p = \mathrm{mix}(p_{HG}(0.8k), p_{HG}(-0.25k), 0.3)$.
  - $\tau_l$ is the optical depth towards the sun. It comes from a cone march of 4–6 steps (by quality) that start
    at 40 m and double each time, reaching 0.6–2.5 km, with coarser mips further out.
  - "powder" $= 1 - e^{-1.5\tau_l - 0.06\sigma}$, blended in away from the sun. It darkens the edges of clouds facing
    you.
  - The last term is diffuse transmission through deep cloud.
  - The ambient blends from the horizon sky at the base to zenith sky at the top.
- **Compositing.** $L \mathrel{+}= T\,S\,(1 - e^{-\sigma\Delta t})$ and $T \mathrel{*}= e^{-\sigma\Delta t}$. The march
  stops at $T < 0.008$. Aerial perspective is applied at the transmittance-weighted mean depth of the cloud.
- **Cost control.**
  - The main view is marched at ½ to ¼ resolution, with the ray start and sub-pixel position jittered every frame.
  - The result is *temporally resolved*: the clouds are kilometres away, so the previous frame is reprojected by
    view direction alone, and 1/10 of the new frame is blended in.
  - A hemispherical cloud panorama is built a strip per frame. It feeds the reflection cube, the drone cameras and
    any view other than the main camera.
- **Motion.** Clouds drift with the gradient wind, about $1.6\,U + 3$ m/s.
- **Effect on light.**
  - Clouds shadow the sea and the people through the sunlight readback (Section 5.6).
  - Light under the deck is the open-sky fraction plus diffuse transmission, $\approx 1/(1 + 0.15\,\tau_\text{vert})$,
    a two-stream estimate.
  - Rain shafts hang below storm cells.

### 6.3 God rays (`sky_godrays.js`)

Crepuscular rays are a screen-space effect:

1. A bright-pass keeps only the open sky: the sun disc, the gaps between clouds and the sunlit cloud edges. Clouds,
   sea fog and the dark silhouettes of people block it.
2. That image is blurred radially towards the sun's screen position in two passes at ½–¼ resolution.
3. The result is added back, tinted by the sunlight, before bloom and tone mapping.

---

## 7. Weather: presets, sea fog, rain and lightning

### 7.1 Presets

There are six presets: Clear, Fair cumulus, Overcast, Storm, Marine fog and Rain. Each sets:

- the clouds: coverage, type, base, thickness and density;
- haze, aerial haze and sea-fog visibility;
- wind and wave height;
- exposure, rain, rain shafts, lightning and humidity.

Switching presets blends every value over `weatherBlend` seconds with a smoothstep ease. Fog is blended in
*extinction* space, not in visibility, so a fog bank thickens evenly. If a sea condition is locked (Chapter 20), the
weather changes only the sky.

### 7.2 Sea fog and visibility

Visibility $V$ and extinction are linked by Koschmieder's law for a 2 % contrast threshold:

$$
\sigma = \frac{-\ln 0.02}{V} = \frac{3.912}{V}.
$$

The fog is an exponential layer with a scale height of 140–500 m. It enters:

- the sky dome and the reflection cube;
- the ocean's aerial-perspective floor (Section 4.7);
- the sunlight, through the transmittance $e^{-\sigma H/\sin e}$ of the fog column towards the sun.

### 7.3 Rain

- **Streaks.** Rain streaks are instanced quads in a box that wraps around whichever camera is rendering, so the
  drone view gets its own rain. In the LWIR pass they are discarded: raindrops sit at air temperature and give no
  useful thermal contrast.
- **Drop rings.** Rings on the water are procedural. The surface is split into cells of 0.42–0.75 m in several
  layers. Each cell holds one impact per cycle, at a hashed position, and the expanding ring adds slope to the
  normal.
- **Splash roughness.** A small slope variance, $0.0025\,\times$ rain amount, is added everywhere.

### 7.4 Lightning

Flash sequences pick a point 3–12 km away at the cloud base. They light:

- the cloud field, from inside;
- the sea, through an added sky term;
- the people, through a hemisphere light.

A jagged, branching bolt is drawn below the cloud base.

---

## 8. People overboard: buoyancy and rigid-body dynamics

### 8.1 Posture and lifejacket

The two human models are glTF assets from the three.js examples. They are posed procedurally, with no animation
clips:

- **Posture.** Forward kinematics with aim targets puts the body in the posture a lifejacket gives. The body is
  inclined about 49° back (0.85 rad), the head is up, the arms scull near the surface and the legs hang down and
  forward. Each bone is rotated by the shortest-arc quaternion that turns its rest direction towards its target
  direction.
- **Motion.** The arms sweep sinusoidally ($\pm0.22$ rad, with the forearms lagging) and the legs tread slowly.
- **Lifejacket.** It is built around the posed torso: a tube-swept collar behind the head, two flattened capsule
  lobes on the chest, retroreflective tape, a belt and a SOLAS light that flashes about once a second.
- **Wet look.** A waterline plane, fitted to the local wave surface, darkens and smooths every material near and
  below it.

### 8.2 Buoyancy from probe spheres

The body is represented by 11 spheres along the skeleton: head, collar, chest with the jacket, belly, hips, thighs,
shins and forearms. Each has a radius $r_i$ and a volume $V_i$, scaled with body mass ($m/88$ kg). Each sphere's
submerged fraction comes from the spherical-cap volume. With $d$ the depth of the centre below the surface:

$$
f(d, r) = \frac{h^2(3r - h)}{4r^3},\qquad h = \mathrm{clamp}(d + r,\ 0,\ 2r) .
$$

Fully submerged ($h = 2r$) gives 1; half submerged gives ½. The buoyant force is
$F_b = \rho g\sum_i V_i f(d_i, r_i)$, with $\rho = 1025$ kg/m³.

**Equilibrium trim.** When a person is created, the water level $y_w$ at which buoyancy balances weight is found by
bisection (60 steps):

$$
\sum_i \rho V_i\, f(y_w - y_i,\ r_i) = m .
$$

The centre of buoyancy at that level is computed, and the **centre of mass is placed 9 cm below it**. This offset is
what makes the body stable face-up, as a lifejacket is designed to do.

### 8.3 Forces and torques

At each sphere $i$, at position $\mathbf r_i$ relative to the centre of mass, with point velocity
$\mathbf v_p = \mathbf v + \boldsymbol\omega\times\mathbf r_i$:

- **Buoyancy**: $\rho g V_i f_i\,\hat{\mathbf y}$.
- **Water drag.** Relative to the wave orbital velocity, which decays with depth as in deep-water linear theory, plus
  the surface current:

$$
\mathbf u_r = \mathbf v_p - \big(\mathbf u_\text{orb}\,e^{-k_p d_i} + \mathbf U_c\big),\qquad
\mathbf F_d = -\big(\tfrac12\rho C_D A_i\lvert\mathbf u_r\rvert + C_\text{lin}\big)\,f_i\,\mathbf u_r,
$$

  with $C_D = 1$, $A_i = \pi r_i^2$ and $C_\text{lin} = 45$ N·s/m.

- **Windage** on the part above water (horizontal only), with the wind at body height taken as $0.8\,U_{10}$:

$$
\mathbf F_w = \tfrac12\rho_\text{air}\,1.1\,A_i\,(1 - f_i)\,\lvert\mathbf u_a\rvert\,\mathbf u_a,\qquad \mathbf u_a = 0.8\,\mathbf U_{10} - \mathbf v_{p,xz}.
$$

- **Wave-slope alignment.** Pressure on a floating body turns it with the local surface:
  $\boldsymbol\tau_\text{align} = 30\,(\hat{\mathbf u}\times\hat{\mathbf n})\cdot s$, where $\hat{\mathbf u}$ is the
  body's up axis, $\hat{\mathbf n}$ the surface normal and $s$ the submerged fraction.
- **Angular damping**: $-10\,\boldsymbol\omega$. There is also a slow random yaw wander.
- **Contact between people.** Below 1.4 m horizontal separation $d$, a soft repulsion $(1.4 - d)\cdot600$ N pushes
  them apart.

**Added mass.** Accelerating a body in water also accelerates the water around it. This is modelled as
$m_a = 0.45\,\rho\,V_\text{sub}$: the translational mass becomes $m + m_a$, and the principal inertias
($I_x = 0.24\,m$, $I_y = 0.05\,m$ kg·m²) are scaled by $1 + m_a/m$.

### 8.4 Integration

The integration is semi-implicit (symplectic) Euler with fixed substeps of $\Delta t = 1/120$ s, at most 12 per frame:

$$
\mathbf v \mathrel{+}= \frac{\mathbf F}{m+m_a}\Delta t,\qquad
\mathbf x \mathrel{+}= \mathbf v\,\Delta t,\qquad
\boldsymbol\omega_b \mathrel{+}= I^{-1}\boldsymbol\tau_b\,\Delta t,\qquad
q \leftarrow \mathrm{normalize}\Big(q + \tfrac12(\boldsymbol\omega, 0)\,q\,\Delta t\Big).
$$

Torques are applied in the body frame, where the inertia tensor is diagonal.

The GPU probes are sampled once per frame. Between substeps, the water height at each sphere is extrapolated to
first order in time and space:

$$
h \approx h_s + \dot h_s\,(t - t_s) + s_x\,(x - x_s) + s_z\,(z - z_s).
$$

---

## 9. Ripples and wakes around the people

`water_ripples.js` runs a separate **linear, dispersive surface-wave simulation** on a square grid that follows the
group. The grid is 384–1024 texels at 6–12.5 cm, covering 48–61 m.

### 9.1 Equation

For deep-water gravity waves, $\omega^2 = g\lvert\mathbf k\rvert$. In operator form,
$\partial_t^2 h = -g\,\mathcal K[h]$, where $\mathcal K$ multiplies each Fourier mode by $\lvert\mathbf k\rvert$.
$\mathcal K$ is approximated by a sparse, multi-scale "ring" stencil, fitted offline:

$$
\mathcal K[h] \approx \frac{1}{\Delta x}\sum_j w_j\Big(4h - \sum_{\mathbf o\in\text{ring}_j} h(\mathbf x + \mathbf o)\Big),
$$

The rings are the four 90° rotations of offsets (1,0), (1,1), (2,2), (4,0), (3,3), (16,0) and (11,11), with weights
0.3572, 0.1195, 0.0115, 0.0240, 0.0248, 0.0207 and 0.0167.

For a plane wave the stencil gives $\sum_j w_j\sum_{\mathbf o}(1 - \cos\mathbf k\cdot\mathbf o)$. The weights are
chosen so this is $\approx\lvert\mathbf k\rvert\Delta x$: the phase speed is within about 12 % of $\sqrt{g/k}$ from 2
to 64 texel wavelengths, and isotropic to a few percent. All weights are non-negative, so the operator is positive
semi-definite and the scheme stays stable.

### 9.2 Time stepping and boundaries

The state is (height, vertical velocity, turbulence). It is updated with symplectic Euler at a fixed substep:

$$
v \mathrel{+}= \Big(-\frac{g}{\Delta x}\mathcal K[h] + \text{forcing}\Big)\Delta t,\quad
v \mathrel{*}= e^{-\beta\Delta t},\quad
h \mathrel{+}= v\,\Delta t .
$$

- **Sponge border.** The outer 11 % of the grid has extra damping, $\beta = \beta_0 + 7s^2$, so outgoing waves are
  absorbed instead of reflecting back.
- **Grid motion.** The grid lives in the frame of the drifting surface water: FFT Lagrangian coordinates minus the
  integrated surface current. Ripples therefore ride on the swell and drift with the water. The grid is only ever
  moved by whole texels, which avoids numerical diffusion.

### 9.3 Forcing

Each body is a Gaussian source $G = e^{-r^2}$ in texel units. Four kinds of forcing act on the field:

- **Heave.** The body's vertical velocity relative to the water drives a *volume-conserving* source,
  $G - \tfrac14e^{-r^2/4}$. Both terms integrate to $\pi$, so no net water is created; the surface is pushed down in
  the footprint and up in a ring around it.
- **Drift.** Motion through the water adds a dipole $G\,(\mathbf u\cdot\mathbf d)$, giving a bow wave and a stern
  trough.
- **Hands, impacts and crest slaps.** Hands sculling near the surface, droplet impacts from the spray system and
  crests slapping the bodies each add impulses.
- **Turbulence.** A turbulence field (decay plus diffusion) drives random, spatially smooth forcing, so churned
  water "boils".

The ocean shader receives the result as one RGBA texture (height, $\partial_x h$, $\partial_z h$, turbulence). The
height displaces the vertices, the slopes add to the normal and the LEAN roughness, and the turbulence adds
micro-foam. A static waterline shape around each torso adds a raised lip and a shallow trough.

---

## 10. Spray, splashes and spindrift

`water_spray.js` is a GPU particle system. Particle state lives in three float textures: position and age, velocity
and lifetime, kind/size/seed. One full-screen pass updates them all. New particles are written into a ring buffer, so
nothing is read back.

### 10.1 Emitters

- **Hands:** exit sheets, entry crowns, skimming flicks, drips.
- **Crest slaps** against bodies.
- **Breaking whitecaps.** The GPU searches the foam map for folding crests ($J$ below threshold, or fresh foam) and
  spawns blown spray, spindrift streaks and mist there. The number of attempts scales with wind.

### 10.2 Dynamics

Each particle relaxes towards the local wind with a drag time constant $\tau$ and falls under an effective gravity:

$$
\mathbf v \mathrel{+}= (\mathbf u_\text{wind}(z) - \mathbf v)(1 - e^{-\Delta t/\tau}),\qquad v_y \mathrel{-}= g_\text{eff}\,\Delta t .
$$

| Kind | $\tau$ (s) | $g_\text{eff}$ (m/s²) |
|---|---|---|
| droplets | 1.1 | 9.81 |
| spindrift | 0.35 | 8.0 |
| mist | 0.12 | 0.35 (plus small eddies from noise) |

The wind follows the **logarithmic profile** over the sea with roughness length $z_0 = 0.2$ mm:

$$
u(z) = U_{10}\,\frac{\ln(z/z_0)}{\ln(10/z_0)},\qquad \text{clamped to } [0.3, 1.15]\,U_{10}.
$$

A particle dies when it enters the wave surface, found by inverting the choppy displacement as in Section 2.11, and
leaves a downward velocity impulse in the ripple field.

### 10.3 Optics

- **Streaks.** Droplets are drawn as camera-facing streaks smeared along their screen motion during the exposure.
- **Phase function.** Two HG lobes, $0.78\,p_{HG}(0.8) + 0.22\,p_{HG}(-0.25)$. Spray is bright when you look towards
  the sun.
- **Rainbow.** A **primary rainbow** appears at scattering angles 137.6° (red), 138.4° (green) and 139.3° (blue),
  each a Gaussian of 0.75° width. The order comes from the dispersion of water: red outside.
- **Dense spray** multiply-scatters like foam (albedo ≈0.85).
- **Glints.** Each drop throws a tiny twinkling image of the sun.
- **LWIR.** Drops are near-blackbodies at sea temperature, cooling towards the wet-bulb temperature as they fly.

---

## 11. Reflections of the people, shadows and wet skin

`refl_reflections_shadows.js` adds four things:

1. **Screen-space reflections of the people on the water.**
   - Reflected rays are marched against the people-only depth buffer, then refined by bisection.
   - A hit must lie above that person's waterline.
   - The march is blurred by the surface roughness: the reflected rays spread by about $2\sigma$ over the ray length.
   - Where the trace cannot resolve the body (off-screen, or its hidden side), simple analytic proxies are used
     instead: a head sphere and a tapered chest capsule per person, tested with ray–sphere and ray–capsule closest
     approach.
2. **Sun shadows on the people.** The shadow map uses PCF filtering. Its light frustum is fitted every frame to the
   people visible in the main and sensor views, with hysteresis and texel snapping so it does not swim.
3. **Shadows and contact darkening on the water.**
   - Shadows on the water are analytic, from the same proxies.
   - The sun glitter loses the sun sharply, while the diffuse sunlit terms fade softly.
   - Water next to a body is darkened by cosine-weighted sphere occlusion of the sky.
4. **Wet skin.** A thin water film is a clear coat ($F_0 = 0.02$, smooth) over skin, hair and jacket, weighted by the
   wetness term.

In the LWIR view, the sky radiance that the sea reflects also includes the warm heads and lifejackets, so people
show warm reflections in the water as they do in real thermal imagery.

---

## 12. Underwater optics

### 12.1 Inherent optical properties

The water column is moderately clear oceanic water (roughly Jerlov type II), with per-channel coefficients in m⁻¹.
Here $t$ is turbidity and "tint" is the water colour relative to the default:

$$
\mathbf a = (0.43 + 0.03t,\ 0.055 + 0.02t,\ 0.035 + 0.035t),\qquad
\mathbf b = (0.028,\ 0.05,\ 0.058)\,t\cdot\text{tint},\qquad \mathbf c = \mathbf a + \mathbf b .
$$

Red is absorbed fastest, which is why everything turns blue-green with depth. Downwelling light decays with the
diffuse attenuation coefficient

$$
K_d \approx \frac{a + b_b}{\bar\mu_d},\qquad b_b = 0.02\,b,\ \ \bar\mu_d = 0.85,
$$

capped below $0.92\,c$ so that the in-scatter integral below stays finite. All three are divided by the "clarity"
setting.

### 12.2 Refraction at the surface

The sun's direction in the water follows Snell's law with $n = 1.333$:

$$
\sin\theta_t = \frac{\sin\theta_i}{1.333}.
$$

The beam loses the Fresnel reflection and is compressed, $E_\text{below} = E\,(1 - F)\cos\theta_i/\cos\theta_t$.

Looking up from below, the whole sky is squeezed into **Snell's window**, a cone of half-angle
$\arcsin(1/1.333) = 48.6°$ (97° across). Radiance gains a factor $n^2$ as it enters the denser medium. Outside the
window the surface is a mirror: total internal reflection shows the water below.

The water-to-air Fresnel term uses a slightly different index per colour channel, so the window edge shows a coloured
fringe. The undersides of foam patches are visible.

### 12.3 Light in the water volume

The phase function is a forward-peaked HG with $g = 0.85$, mixed with 15 % isotropic scattering. Along a view ray from
depth $z_c$ in direction $\mathbf r$ ($r_y > 0$ = up), over length $D$, the downwelling light at depth $z$ is
$E\,e^{-K_dz}$ and the ray is attenuated by $e^{-cs}$. The single-scattered radiance then has a closed form:

$$
L_\text{in} = \sigma_s\Big(E_\odot\,p(\mathbf r\cdot\mathbf l_w) + \frac{E_\text{amb}}{4\pi}\Big)\,e^{-K_dz_c}\,
\frac{1 - e^{-kD}}{k},\qquad k = c - K_d\,r_y .
$$

The volume pass marches the first few metres to draw **god rays**: shafts of sunlight shadowed by the wave surface
and the bodies. The rest of the ray uses the closed form. Bodies and gear under water get **caustics**, a network of
focused lines from two drifting ridged-noise layers. The pattern is looked up where the refracted sun ray meets the
surface. Bodies also receive depth-dependent light, $e^{-K_d\,\text{depth}}$.

Also drawn under water:

- marine snow and bubbles;
- simple eye adaptation (looking up into Snell's window is far brighter than looking into the blue);
- a half-over / half-under split view with a meniscus line.

---

## 13. Rescue gear and the dye marker

### 13.1 Floating objects

The lifebuoy, life raft, debris and rescue vessel use the same force model as the people (Chapter 8): probe spheres,
drag and windage, with surface probes 246–254. Each object:

| Object | Description |
|---|---|
| Lifebuoy | SOLAS ring, 0.76 m outer / 0.48 m inner diameter; orange with white quarters, retroreflective tape, a grab line and a light flashing about 60 times a minute |
| Life raft | two stacked buoyancy tubes, a canopy on an inflatable arch, ballast pockets, a lifeline and a steady canopy light at night |
| Debris | floating pieces |
| Rescue vessel | hull, superstructure, a hot exhaust for the LWIR view, navigation lights and a RIB on the stern deck |

### 13.2 Fluorescein dye

The sea-marker dye is released from the first person's lifejacket. It is represented as up to 14 Gaussian puffs.
Each puff carries a mass, a spread across the stretch axis and a spread along it.

- **Drift.** The dye lives in the top few centimetres, so it moves with the surface current plus a little extra wind
  drift: $\mathbf u = \mathbf U_c + 0.006\,\mathbf U_\text{wind,body}$.
- **Spread.** Ocean diffusion is scale-dependent (Okubo 1971): larger patches are torn apart by larger eddies. The
  model uses

$$
\frac{d\sigma^2}{dt} = 2K,\qquad K = (0.008 + 0.0045\,\sigma)\,(0.4 + U/10)\ \ \mathrm{m^2/s}.
$$

- **Fading.** Mass decays as $e^{-t/2400\,\text{s}}$, for photobleaching and mixing below the visible layer.
- **Merging.** When there are more than 14 puffs, the two adjacent ones that overlap most are merged.
- **Shading.** The dye absorbs blue and fluoresces green. Concentrated cores turn yellow-orange. The patch is torn into
  filaments and windrows by near-surface turbulence noise and stretched downwind.

---

## 14. Cameras, lens and post-processing

### 14.1 Camera presets (`cine_camera.js`)

The presets are free orbit, sea level, drone orbit, helicopter overhead and rescue-boat point of view. Transitions
between them use a critically or under-damped second-order response:

$$
\ddot x = \omega^2\,(x^* - x) - 2\zeta\omega\,\dot x .
$$

The focal length $f$ (full-frame equivalent, 24 mm sensor height) sets the vertical field of view:

$$
\text{fov} = 2\arctan\frac{12\ \text{mm}}{f}.
$$

The boat preset floats on probes 240–244 (bow, stern, port, starboard, centre). The sea-level camera floats on probe
245 and gets lens droplets when waves wash over it.

### 14.2 The basic composite (engine)

Every HDR image goes through, in order:

- exposure;
- vignette $1 - 0.1\,r^2$;
- a slight cool white balance;
- highlight desaturation;
- **AgX** tone mapping;
- dither.

AgX (by Troy Sobotka) maps colour through an inset matrix, then takes $\log_2$, clamped to [−12.47, +4.03] EV and
normalised. A 6th-order polynomial fit of the AgX contrast curve follows, then a mild look (saturation 1.05, power
1.06) and the outset matrix. Bright colours desaturate gracefully towards white instead of clipping to pure hues.
Dither is ±1 LSB of interleaved gradient noise, which prevents banding in the sky.

### 14.3 The cinematic chain (`cine_post.js`)

- **TAA.** Halton(2,3) projection jitter. History is reprojected using the depth buffer, then clipped to the
  neighbourhood's variance box in YCoCg. It is resampled with a 5-tap Catmull–Rom filter. Blending happens in a
  reversibly luminance-compressed space, $c/(1 + L)$, so HDR glints do not flicker.
- **Bloom.** An energy-conserving dual-filter pyramid. The first downsample uses Karis averaging, which suppresses
  fireflies. Bloom levels are *mixed* into the image rather than added.
- **Auto exposure.** Centre-weighted log-average luminance $\bar L$. The target is
  $\mathrm{EV} = \mathrm{clamp}\big(k(\log_2 0.25 - \log_2\bar L - \log_2 E_\text{man}),\ \mathrm{EV}_\min,\ \mathrm{EV}_\max\big)$
  with partial strength $k = 0.75$, so a bright scene still reads brighter than a dark one. It is approached
  exponentially, faster when brightening (3/s) than when darkening (1/s).
- **Depth of field.** The thin-lens circle of confusion is
  $c = \dfrac{f^2}{N(s-f)}\,\dfrac{\lvert z - s\rvert}{z}$, converted to pixels for a 24 mm sensor height. Focus
  distance $s$ comes from autofocus on the target.
  - Bokeh is gathered on a 7-blade polygonal aperture at half resolution.
  - Near-field CoC tiles let foreground blur spread over in-focus background.
- **Lens and composite.** Camera motion blur, water droplets on the lens, contrast-adaptive sharpening and chromatic
  aberration.
- **Sun flare.** Ghosts are aperture-shaped reflections along the line through the frame centre. There is also a
  starburst and lens dirt. Clouds, people and waves occlude all of them, through the sun-visibility estimate.
- **Grading.** White balance, grade, AgX and film grain finish the frame.

The RGB sensor window keeps the basic composite, as a real camera would.

---

## 15. The EO (RGB) sensor

The EO camera is the same renderer and shading as the main view, drawn from the gimbal pose into a 640×512 (5:4)
target with 4× MSAA. That is the format of common EO/IR gimbals, and it matches the IR camera, so the two images are
**co-registered**: they share one pose and one field of view, and one label file fits both.

The gimbal has two modes:

- **Follow the main view.** The camera is kept at least 0.6 m above the local water surface.
- **Drone orbit.**
  - The drone circles the group at a set altitude and range.
  - It starts on the side away from the sun and turns at a set rate.
  - It bobs ±0.4 m.
  - Residual stabilised jitter of about 0.05° is added to the aim point.

The horizontal field of view shown in the window is $\text{HFOV} = 2\arctan\!\big(\tan(\text{VFOV}/2)\cdot\tfrac54\big)$.

---

## 16. The thermal-infrared (LWIR 8–14 µm) sensor

The thermal camera is not a colour filter on the visible image. Every material in the scene has a second shader
path. When `uThermal = 1`, it outputs **in-band radiance** (W m⁻² sr⁻¹) computed from temperature, emissivity,
reflected sky and atmospheric path. A microbolometer model then turns radiance into the picture.

### 16.1 Planck radiance in the 8–14 µm band

The spectral radiance of a blackbody is

$$
B_\lambda(T) = \frac{c_1}{\lambda^5\big(e^{c_2/(\lambda T)} - 1\big)},\qquad
c_1 = 2hc^2 = 1.191042972\times10^{8}\ \mathrm{W\,m^{-2}\,sr^{-1}\,\mu m^4},\quad
c_2 = \frac{hc}{k_B} = 14387.7688\ \mu\mathrm{m\,K}.
$$

The band radiance $B(T) = \int_8^{14}B_\lambda\,d\lambda$ is computed with **4-point Gauss–Legendre** quadrature. The
nodes are $\lambda_i = 11 \pm 3x_i$ with $x_i = 0.861136, 0.339981$, and the weights are $3w_i$ with
$w_i = 0.347855, 0.652145$:

$$
B(T) \approx 1.04356\big[B_\lambda(8.4166) + B_\lambda(13.5834)\big] + 1.95644\big[B_\lambda(9.9801) + B_\lambda(12.0199)\big].
$$

The integrand is smooth and single-humped, so this is accurate to better than $3\times10^{-6}$ relative over
180–360 K, against a 20 000-step reference.

**Inverse.** Apparent temperature from radiance uses a Sakuma–Hattori form fitted to $B(T)$. It is closed-form, with
error below 0.01 K over 200–350 K:

$$
T(L) = \frac{1}{A}\left(\frac{c_2}{\ln(C/L + 1)} - B\right),\qquad A = 9.3734\ \mu\mathrm m,\ B = 173.725\ \mu\mathrm{m\,K},\ C = 6747.59\ \mathrm{W\,m^{-2}\,sr^{-1}}.
$$

### 16.2 Surface radiance

For an opaque surface, Kirchhoff's law gives reflectance $= 1 -$ emissivity:

$$
L = \varepsilon\,B(T_s) + (1-\varepsilon)\,L_\text{sky}(\mathbf r).
$$

Water at 10 µm absorbs within 10–20 µm ($\alpha = 4\pi k/\lambda \sim 10^3\ \mathrm{cm^{-1}}$). The sea is therefore
an **opaque** emitter–reflector in the thermal view. Nothing below the waterline is visible: submerged limbs vanish,
which is exactly what thermal search imagery shows.

### 16.3 Emissivity of water: complex Fresnel

Water is absorbing in this band, so its refractive index is complex, $\tilde n = n + ik$. The code uses an effective
$n = 1.242$, $k = 0.154$. These were fitted so that one evaluation reproduces, within 0.001 for 0–85°, the
Planck(290 K)-weighted band average of the spectral emissivity from Hale & Querry (1973) water constants. With
$c = \cos\theta$, $s^2 = \sin^2\theta$:

$$
\begin{aligned}
A &= n^2 - k^2 - s^2,\quad r = \sqrt{A^2 + 4n^2k^2},\quad a = \sqrt{\tfrac{r + A}{2}},\quad b^2 = \tfrac{r - A}{2},\\
R_s &= \frac{(a - c)^2 + b^2}{(a + c)^2 + b^2},\qquad
R_p = R_s\,\frac{(a - s\tan\theta)^2 + b^2}{(a + s\tan\theta)^2 + b^2},\\
\varepsilon_w(\theta) &= 1 - \tfrac12(R_s + R_p).
\end{aligned}
$$

This gives 0.984 at nadir, 0.945 at 60° and 0.65 at 80°, consistent with Masuda et al. (1988). Wind roughening is
not built in. It comes from evaluating $\varepsilon_w$ per facet with the wave normal. This is why the sea looks
colder towards the horizon in thermal imagery: at grazing angles it reflects more of the cold sky.

**Foam** is a bubble layer that behaves like a near-black volume emitter:
$\varepsilon_f = 0.99 - 0.06\,(1 - \cos\theta)^4$. Foam also breaks the cool skin and sits at bulk temperature
(+0.05 K), so whitecaps look slightly warm.

### 16.4 Sea skin temperature

The top millimetre of the sea is cooler than the water below, because it loses heat to the air. This is the
cool-skin effect (Donlon et al. 2002):

$$
\Delta T = -0.14 - 0.30\,e^{-u/3.7}\ \ \mathrm K\qquad(-0.44\ \mathrm K \text{ calm} \to -0.16\ \mathrm K \text{ at } 10\ \mathrm{m/s}).
$$

### 16.5 The sky in the thermal band

- **Water vapour** (Magnus–Bolton):

$$
e = \text{RH}\cdot6.112\exp\!\Big(\frac{17.67\,t}{t + 243.5}\Big)\ \text{hPa},\qquad \rho_v = \frac{216.7\,e}{T}\ \mathrm{g\,m^{-3}}.
$$

- **Clear-sky emissivity**, from Idso's (1981) 8–14 µm equation for the hemispheric value. It is capped at 0.92,
  because the fit overshoots in tropical humidity. It is converted to zenith with the Elsasser diffusivity factor
  1.66, then to a slant path at elevation $e_l$ (plane-parallel):

$$
\varepsilon_h = \min\!\big(0.24 + 2.98\times10^{-8}\,e^2\,e^{3000/T},\ 0.92\big),\quad
\varepsilon_z = 1 - (1-\varepsilon_h)^{1/1.66},\quad
\varepsilon(e_l) = 1 - (1 - \varepsilon_z)^{1/\sin e_l}.
$$

  - The constant 0.24 is the CO₂/O₃ floor.
  - $e^2$ is the water-vapour self-continuum.
  - At the horizon $\varepsilon\to1$: the sky there is as warm as the air. At the zenith a clear sky is very cold.
- **Clouds** are black in the LWIR (liquid water path above ~30 g m⁻²) and radiate at their base temperature. That
  temperature uses the standard lapse rate, $T_c = T_\text{air} - 0.0065\,z_\text{base}$, or comes from the
  volumetric layer:

$$
L_\text{sky} = \varepsilon\,B(T_\text{air}) + f_c\,(1-\varepsilon)\,B(T_c).
$$

### 16.6 The air path

Between the target and the camera, extinction in the window is dominated by the water-vapour continuum (Roberts,
Selby & Biberman 1976): a self term $\propto\rho_v^2$ with its temperature dependence, a linear term and a floor.

$$
\gamma = 0.05 + 0.0075\,\rho_v + 0.00095\,\rho_v^2\,\exp\!\Big[1800\Big(\frac1T - \frac1{296}\Big)\Big]\ \ \mathrm{km^{-1}},
$$

which gives about 0.08 km⁻¹ in cold dry air, 0.3 km⁻¹ at 20 °C and 75 % RH, and 0.75 km⁻¹ in the tropics. The path
is isothermal at air temperature:

$$
L_\text{seen} = L\,\tau + (1 - \tau)\,B(T_\text{air}),\qquad \tau = e^{-\gamma d}.
$$

### 16.7 How warm is a person in the water? A surface energy balance

Each body material solves one linearised steady-state energy balance for its surface temperature $T$:

$$
K(T_d - T) + (h_c + h_r)(T_a - T) + h_w(T_w - T) + \alpha S - Q_\text{sky}
- w\,16.5\,h_c\big[(1-\text{RH})\,e_s(T_a) + s\,(T - T_a)\big] = 0,
$$

which solves to

$$
T = \frac{K T_d + (h_c + h_r)T_a + h_wT_w + \alpha S - 30 - L_e(1-\text{RH})\,e_s + L_e\,s\,T_a}{K + h_c + h_r + h_w + L_e\,s},\qquad L_e = 16.5\,w\,h_c .
$$

The terms are:

| Term | Meaning | Value |
|---|---|---|
| $K$, $T_d$ | conductance of tissue or insulation to a deep temperature | per material, below |
| $h_c$ | forced convection (whole body, ISO 9920 / de Dear 1997) | $\max(3,\ 8.3\,u^{0.6})$ W m⁻² K⁻¹ |
| $h_r$ | linearised radiation | 5.5 W m⁻² K⁻¹ |
| $h_w$ | wave-wash coupling to the water at height $z$ above the waterline | $80\,(1 - \mathrm{smoothstep}(0, 0.25, z))$ |
| $Q_\text{sky}$ | extra long-wave loss to a cold sky (≈50 % sky view) | 30 W m⁻² |
| evaporation | Lewis relation 16.5 K/kPa, wetted fraction $w$; $e_s(T)$ linearised about $T_a$ with $s = de_s/dT$ | RH 0.8 (marine) |
| $\alpha S$ | absorbed sunlight, with $S$ the global horizontal irradiance | per material |

| Material | $K$ | $T_d$ | $w$ | Emissivity | Typical result ($T_w$ 15 °C, $T_a$ 20 °C, 5 m/s, night) |
|---|---|---|---|---|---|
| face | 40 | 36.5 °C | 0.15 | 0.98 | ≈28 °C; nose and ears 2.5 K colder |
| wet hair | 6 | 34 °C | 1.0 | 0.97 | ≈18.5 °C (below air: evaporation) |
| hands | 10–30 (vasoconstriction in cold water) | 36.5 °C | 0.5 | 0.98 | ≈21 °C |
| wet clothing | 8 | midway between $T_w$ and 37 °C | 1.0 | 0.95 | $T_w$ + ≈2 K at the waterline |
| lifejacket | 1.5 (air bladder) | 33 °C | wet → dry with height | 0.88 | $T_a$ − 1 K; +5 K in 800 W/m² sun |

The assumptions are a 37 °C core and less than about an hour in the water, so there is no deep hypothermia. The
result is what makes a head stand out against the sea at night and blend in more on a hot afternoon.

### 16.8 Solar glint in the thermal band

The sun's 8–14 µm irradiance at the top of the atmosphere is $\Omega_\odot\,B(5772\ \mathrm K) = 1.51$ W m⁻²,
about 0.1 % of the solar constant. After the slant path it reaches the sea through the same microfacet lobe as the
visible glitter, with the thermal Fresnel reflectance. It is weak, visible as faint sparkle only at grazing geometry.

### 16.9 The microbolometer camera

The camera model is a Boson/Tau-class uncooled 640×512 camera running at 30 Hz.

1. **Optics and pixel aperture.** A 3×3 binomial kernel, $(1,2,1)\otimes(1,2,1)/16$, is applied in *radiance*,
   since a detector integrates flux, not temperature. A stray-light halo mixes a little of a wide blur back in,
   $L_o = L_c + g(0.35L_\text{ring} + 0.65L_\text{wide} - L_c)$. Because the mix is in radiance, which is steep in
   temperature, hot objects glow.
2. **Temporal noise (NETD).** The noise is constant in radiance, so in kelvin it scales with $1/(dB/dT)$:

$$
\sigma_T(T) = \text{NETD}\cdot\frac{d(300)}{d(T)},\qquad d(T) = \frac{x^2e^x}{(e^x-1)^2},\ x = \frac{1438.777}{T}\ \ (\lambda = 10\ \mu\mathrm m),
$$

   with NETD 45 mK at 300 K. Colder scenes are noisier in kelvin. The noise is white per pixel plus a weak per-frame
   row component (20 %).

3. **Fixed-pattern noise after NUC.** The residual is

$$
\text{FPN} = s\big(0.025\,g_\text{col} + 0.012\,g_\text{pix} + 0.003\,g_\text{gain}(T - T_\text{ref}) + \phi\,(0.08\,g'_\text{col} + 0.04\,g'_\text{pix})\big),
$$

   with fixed per-pixel and per-column Gaussian draws $g$. The offset drift $\phi = (t \bmod 180)/180$ builds up
   until the next flat-field correction (shutter event) every 180 s. The gain residual pivots at the shutter
   temperature $T_\text{ref}$.

4. **Low-frequency non-uniformity.** Vignetting or housing narcissus, $-V(0.6 + 0.4\phi)\,r^2$, plus a drift gradient.
5. **Digital detail enhancement (DDE).** $T_d = T_s + k\,\mathrm{clamp}(T_s - T_\text{bg},\ \pm0.2\,\text{span})$,
   with the local background $T_\text{bg}$ from a 2.5-pixel ring. It boosts noise and FPN as real DDE does.
6. **Automatic gain control.**
   - Every third frame an 80×64 apparent-temperature image is read back. Its 1 % and 99 % percentiles come from a
     1024-bin histogram, interpolated within bins.
   - The window gets a 5 % margin and a minimum span of 4 K.
   - It is smoothed with $a = 1 - e^{-\Delta t/\tau}$, $\tau = 0.7$ s.
   - A **plateau-equalisation** curve is built: a 64-bin histogram clipped at 2× the mean, then its cumulative sum
     sampled at 17 knots. It is blended 35 % with the linear ramp.
7. **Palettes.** White-hot, black-hot, ironbow and rainbow-HC, plus a burned-in crosshair and palette bar.

---

## 17. The LiDAR

The LiDAR is a time-of-flight laser scanner on the same payload, boresighted with the EO/IR cameras. Each pulse
direction is generated analytically. Its echo comes from a range and reflectance image of the real scene geometry:
the displaced FFT sea, the skinned people and the gear. The return physics is evaluated per pulse on the GPU, and the
points are stored in world coordinates in a float texture, with no readback.

### 17.1 Sensors modelled

| Model | Scan | Field | Pulses/s | $R_{10}$ / max | Divergence | Returns |
|---|---|---|---|---|---|---|
| Solid-state rosette 70° | Risley prisms, non-repetitive | 70.4°×70.4° | 240 k | 190 / 450 m | 0.28° | 3 |
| Solid-state long range 15° | rosette | 15°×15° | 240 k | 320 / 500 m | 0.07° | 2 |
| DJI L2-type, non-repetitive | rosette | 70°×75° | 240 k | 300 / 450 m | 0.046° | 5 |
| DJI L2-type, repetitive | 6-line sweep | 70°×3° | 240 k | 300 / 450 m | 0.046° | 5 |
| Spinning 32 / 64 / 128 beam | rotating | 360° × (−16…15° / ±22.5° / ±11.25°) | 1024–2048 columns per turn | 60–150 / 120–240 m | 0.1–0.18° | 2 |

$R_{10}$ is the range at which a 10 % Lambertian target is detected with 90 % probability, the usual datasheet
figure. The L2-type also has a 3 m blind zone and 2 cm range noise. The wavelength is selectable: 905 or 1550 nm.

### 17.2 Scan patterns

- **Risley-prism rosette.** Two counter-rotating prisms at $\omega_1 = 2\pi\cdot121.3$ and
  $\omega_2 = -2\pi\cdot109.7$ rad/s steer the beam to

$$
\mathbf d(t) = \frac{\Theta}{2}\big(e^{i\omega_1 t} + e^{i\omega_2 t}\big)\quad\text{(as a 2D angle vector)},
$$

  a rose whose petals precess, so the field fills in more with every cycle. This is the "non-repetitive" scanning of
  Livox-type sensors.
- **Repetitive line scan.** Six channels are stacked across the 3° field and swept horizontally at 25 Hz, with a slow
  vertical dither.
- **Spinning.** Azimuth runs over columns; elevation is linear across the channels.

A direction with off-axis angle $\theta$ and azimuth $\phi$ becomes
$(\sin\theta\cos\phi,\ \sin\theta\sin\phi,\ -\cos\theta)$ in the sensor frame, then is rotated into the world.

Each frame, a range image is rendered for the part of the field the pulses of that frame will cover. The solid-state
sensors use one frustum just larger than the field, with pixels roughly square in angle. The spinning sensors render
the swept sector in chunks of up to 100°. Every pulse then samples that image.

### 17.3 What comes back from each surface

The range pass writes, per pixel, the range, the **effective reflectance** $\rho_\text{eff} = \pi f_r\cos\theta$ and a
class. For a Lambertian card $\rho_\text{eff} = \rho\cos\theta$.

**Water: specular backscatter.** A monostatic laser sees water only through the wave facets that face it exactly
($\mathbf h = \mathbf v$). Using the same Gaussian slope statistics as the glitter (LEAN covariance, Section 4.2),
Kirchhoff / geometric-optics backscatter gives

$$
\rho_\text{eff} = \frac{\pi\,R(0)\,P\big(\mathbf s = -\mathbf v_{xz}/v_y\big)\,G(\mathbf v)}{4\cos^5\theta},\qquad
R(0) = \Big(\frac{n-1}{n+1}\Big)^2 \approx 0.020\ (905\ \text{nm},\ n = 1.328),\ \ 0.019\ (1550\ \text{nm},\ n = 1.318),
$$

capped at 60. This is why calm water at grazing angles returns almost nothing, while near nadir the glints are
strong. A tiny volume term, $5\times10^{-4}\cos\theta$ at 905 nm and $5\times10^{-5}$ at 1550 nm, models water-leaving
light. Water absorbs about 7 m⁻¹ at 905 nm and about 1000 m⁻¹ at 1550 nm.

**Foam:** Lambertian, $\rho = 0.25$ at 905 nm and 0.08 at 1550 nm.

**People and gear:** Lambertian per material at the laser wavelength.

| Material | $\rho$ at 905 nm | $\rho$ at 1550 nm | Wetting weight $k_w$ |
|---|---|---|---|
| skin | 0.45 | 0.07 | 0.25 |
| hair | 0.12 | 0.05 | 0.5 |
| textiles | $0.12 + 0.45\,m$ | $0.04 + 0.10\,m$ | 1 |
| coated fabric, plastic, wood | $0.06 + 0.62\,m$ | $0.04 + 0.40\,m$ | 0.5 |

- $m$ is the brightest sRGB channel of the material colour.
- Water darkens a surface: $\rho \leftarrow \rho\,\big(1 - k_w\,\text{wetness}\cdot0.40\big)$ at 905 nm, with 0.55 in
  place of 0.40 at 1550 nm, where water absorbs more. Wet skin is therefore dark at 1550 nm.
- **Retroreflective tape** adds $300\cos^2\theta$ to $\rho_\text{eff}$, about 1000× a white card at normal incidence.
  It saturates the receiver.

### 17.4 Link budget and detection

The signal-to-noise ratio is normalised so that $\text{SNR} = 1$ for a 10 % target at $R_{10}$:

$$
\text{SNR}(\rho, R) = \frac{\rho\;e^{-2\alpha R}\;O(R)\;R_{10}^2}{0.1\,R^2}\;\eta_\text{sun},\qquad
O(R) = \mathrm{smoothstep}\big(\tfrac12R_\min,\ 2R_\min,\ R\big),
$$

- $e^{-2\alpha R}$ is the two-way atmospheric loss.
- $O(R)$ is the transmitter/receiver overlap near the sensor, which makes the blind zone.
- $\eta_\text{sun} = 1/(1 + k_\lambda E_\odot/1000)$ models daylight background noise, with $k_\lambda = 0.6$ (905 nm)
  or 0.3 (1550 nm) and $E_\odot$ the solar irradiance in W/m². It brings the L2 from 300 m to about 240 m at noon.

**Detection probability** is a Weibull law calibrated to the datasheet point:

$$
P_d = 1 - \exp\!\Big[-\Big(\frac{\text{SNR}}{0.659}\Big)^2\Big],\qquad P_d(1) = 1 - e^{-\ln10} = 0.9 .
$$

**Speckle.**
- A glint comes from a few facets, so its intensity is exponentially distributed (fully developed speckle) and
  flickers from pulse to pulse.
- Diffuse targets average many speckles and vary by only about 12 % (Gaussian).

**Range noise** grows at low SNR: $\sigma_R = \sigma_0\,\mathrm{clamp}(1/\sqrt{\text{SNR}},\ 1,\ 6)$.

### 17.5 The atmosphere at the laser wavelength

- The visible extinction $\sigma_{550}$ comes from the sea fog or haze. Visibility follows from Koschmieder,
  $V = 3.912/\sigma_{550}$.
- Wavelength scaling follows **Kim et al. (2001)**:

$$
\alpha_\lambda = \sigma_{550}\Big(\frac{\lambda}{550}\Big)^{-q},\qquad
q = \begin{cases}1.6 & V > 50\ \text{km}\\ 1.3 & 6 < V \le 50\\ 0.16V + 0.34 & 1 < V \le 6\\ V - 0.5 & 0.5 < V \le 1\\ 0 & V \le 0.5\end{cases}
$$

  In dense fog ($q = 0$) infrared is no better than visible light.
- Rain adds $0.365\,R^{0.63}$ km⁻¹ ($R$ in mm/h; the "Rain" weather setting maps to 0–25 mm/h).
- Fog and rain also cause **clutter**: false echoes from droplets near the sensor, with exponentially distributed
  range.
- Rare **solar false alarms** occur, more of them in bright sun.

### 17.6 Multiple returns per pulse (DJI L2-type, up to 5)

A real pulse has a footprint. When it straddles two surfaces, for example a head and the sea behind it, the receiver
sees two echoes. The model works as follows:

1. **Footprint samples.** The footprint is sampled at **five points**: the centre (slightly jittered) and four on the
   edge at half the divergence. Each sample carries ⅕ of the pulse energy.
2. **Merging.** Two samples merge into one echo, with energy-weighted range, if they are closer than the pulse can
   resolve ($\Delta R < 0.6$ m, about a 4 ns pulse). They also merge if they are on the same kind of surface stretched
   over neighbouring pixels at grazing incidence ($\Delta R < 0.6 + 4R\cdot$ pixel angle). Otherwise they are separate
   echoes.
3. **Extra echoes.**
   - **Blooming** around retroreflectors: optical crosstalk leaks a weak echo into neighbouring pulses.
   - **Spray** blown off breaking crests, 0.4–2.9 m in front of the water echo, from about sea state 4–5.
   - Fog and rain droplet clutter.
4. **Detection and storage.** Each echo is detected independently (Section 17.4). The detected echoes are sorted by
   range. Return $r$ of the pulse is the $r$-th of them.

Each pulse owns $R$ consecutive texels of the point buffer, one per return. A texel stores the world position in xyz.
The w component packs the return number, the class and the reflectance:

$$
w = \big(16\cdot\text{return} + \text{class}\big)\cdot1000 + \min(\rho_\text{app}, 999).
$$

Classes: 1 water, 2 foam, 3 person, 4 retroreflector, 5 object, 6 clutter, 7 blooming. The window draws one GL point
per texel, with the position fetched in the vertex shader (`gl_VertexID`). It can colour by:

- reflectivity;
- range;
- height;
- return type;
- return number;
- true colour from the EO image (projected with the camera's view-projection matrix, as the L2 does with its built-in
  camera).

---

## 18. LiDAR file formats: PCD, LAS and LAZ

### 18.1 PCD

PCD files carry the fields `x y z intensity label return`, in ASCII for single exports and binary for recordings.
The header is pure ASCII, because a non-ASCII character would shift the binary offsets.

### 18.2 LAS 1.2, point format 1

| LAS field | Content |
|---|---|
| X, Y, Z | integers in millimetres (scale 0.001) |
| intensity | $\mathrm{clamp}(\mathrm{round}(32768\,\rho_\text{app}),\ 0,\ 65535)$: a white Lambertian card ≈ 32768, retroreflectors saturate |
| return number / number of returns | from the multi-return list |
| classification | ASPRS codes: 9 water (and foam), 7 noise (clutter, blooming), 1 objects; user-defined 19 person, 20 retroreflector |
| user data | the simulator's own label |
| point source ID | frame number |
| GPS time | simulation time of the pulse (s) |
| scan angle | beam angle from the sensor's forward axis (deg, clamped to ±90) |

### 18.3 LAZ (LASzip compression)

`sensor_lidar_las.js` contains a JavaScript port of the encoder side of LASzip (Isenburg 2013; Apache 2.0). It writes
"pointwise chunked" compression with the POINT10 v2 and GPSTIME11 v2 items, 50 000 points per chunk, as LAStools
does for LAS 1.2. The principle:

- **Prediction.** Each field is predicted from earlier points, and only the residual is coded.
  - x and y use the **median of the last five differences**, kept per return-number context (16 contexts).
  - z uses the last z of the same return layer.
  - Intensity, flags, classification and GPS time use their own adaptive context models.
  - GPS time is coded as a delta, handling several interleaved time sequences.
- **Integer compressor.** A residual $c$ is coded as its bit-length class $k$, with an adaptive symbol model, then
  its position within the class.
- **Arithmetic coder.** A 32-bit range coder after Said's FastAC. It renormalises when the range drops below
  $2^{24}$, and adaptive models periodically rescale their frequency tables.
- **Chunks.** Each chunk starts a fresh coder, so a reader can seek. A chunk table at the end, pointed to from the
  header, lists the chunk sizes.

The output is about a third of the size of LAS. Encoding runs in two Blob-URL web workers built from the same function
source, so long recordings do not stall the frame loop.

---

## 19. Data recording

### 19.1 Frame clock

The recorder captures on **simulation time**: frame $k$ is due at $t_k = k/f_\text{rec}$, so pausing the simulation
pauses the recording.

- **"Never drop frames" on.** The main loop's step is capped (`stepLimit`) so that the simulation never passes a
  capture time before that frame is written. If the disk or GPU is slow, the simulation slows down.
- **"Never drop frames" off.** The simulation stays real-time, and missed captures are counted as dropped.

### 19.2 What is written per frame

| Folder | Content |
|---|---|
| `rgb/` | EO image, tone-mapped as in its window (JPEG or PNG) |
| `ir/` | the LWIR image as displayed (AGC, palette, noise) |
| `ir_raw/` | **radiometric** 16-bit PGM (big-endian), $\text{value} = \mathrm{round}(100\,T)$, so $T\,[\mathrm K] = \text{value}/100$ |
| `lidar/` | the points of the pulses fired since the previous frame, as PCD / LAS / LAZ; a 360° scanner gives its latest full turn |
| `labels/` | YOLO boxes (below) |
| `meta/` | time, camera and LiDAR poses, intrinsics, environment, per-person boxes in pixels |

The number of new LiDAR pulses since the last frame is $n = (\text{head} - \text{lastHead}) \bmod N_\text{buf}$.

### 19.3 Camera intrinsics

The camera is a pinhole with square pixels, principal point at the image centre:

$$
f_x = f_y = \frac{h/2}{\tan(\text{VFOV}/2)},\qquad c_x = w/2,\quad c_y = h/2 .
$$

### 19.4 Ground-truth boxes

For each person, every bone with a radius $r$ contributes five points: its centre and centre $\pm r$ along the
camera's right and up axes. Points below the person's waterline plane ($\mathbf n\cdot\mathbf p + w < 0$) or behind
the camera are discarded. The rest are projected:

$$
u = \frac{x_\text{ndc} + 1}{2},\qquad v = \frac{1 - y_\text{ndc}}{2}.
$$

The box is the min/max over the visible points, clipped to the image. It needs at least three visible points. It is
written in YOLO form as class 0 with centre, width and height normalised. The JSON metadata adds the visible fraction,
whether the box is truncated, the distance and the 3D position. The RGB and IR images share one pose, so one label
file fits both.

### 19.5 Storage

Recordings go to a folder chosen with the File System Access API. The handle is kept in IndexedDB, so the browser
asks only for permission on a later visit. Without a folder, everything goes into a store-only ZIP:

- local headers and a central directory;
- CRC-32 with the reflected polynomial `0xEDB88320`;
- one download when recording stops.

---

## 20. Sea condition (sea-state scale 0–9)

The scale is the Douglas / WMO code 3700 table (中国海况等级 无浪 … 怒涛). It is defined by the **significant height
of the wind sea**; swell is graded separately.

| Level | Name | $H_s$ band | Height used |
|---|---|---|---|
| 0 | Calm (glassy) | 0 m | 0 |
| 1 | Calm (rippled) | 0–0.1 m | 0.06 |
| 2 | Smooth (wavelets) | 0.1–0.5 m | 0.3 |
| 3 | Slight | 0.5–1.25 m | 0.88 |
| 4 | Moderate | 1.25–2.5 m | 1.9 |
| 5 | Rough | 2.5–4 m | 3.25 |
| 6 | Very rough | 4–6 m | 5 |
| 7 | High | 6–9 m | 7.5 |
| 8 | Very high | 9–14 m | 11.5 |
| 9 | Phenomenal | > 14 m | 16 |

Picking a level builds a fully developed wind sea in the middle of its band:

1. **Wind.** It comes from the Pierson–Moskowitz relation for a fully developed sea. This also matches the Beaufort
   force usually quoted for each level.

$$
H_s = 0.0248\,U_{10}^2 \quad\Longrightarrow\quad U_{10} = \sqrt{H_s/0.0248}.
$$

2. **Fetch.** It is found by **bisection** (geometric mean, 50 steps, 0.5–3000 km) such that this simulator's own
   JONSWAP integral (Section 2.10) gives exactly that height. Its non-dimensional fetch $gF/U^2$ lands near
   $1.3\times10^4$, close to full development.
3. **Whitecaps** follow the Monahan law (Section 3.4). Level 9, for example, gives $U = 25.4$ m/s, $F = 842$ km and a
   wind sea of 16 m.
4. **Swell.** A moderate swell, 0.2–3 m and 130–250 m long, grows with the level.

The level shown is always read back from the wind-sea height of the spectrum in use, by comparing against the band
edges [0.03, 0.1, 0.5, 1.25, 2.5, 4, 6, 9, 14] m. Moving the wind or fetch by hand therefore moves the level. Weather
presets keep the chosen sea and change only the sky.

---

## 21. Flight route planner and mission flight

The planner works like a mapping-mission planner (DJI Pilot 2 style). It shows a top-down map of the live scene
through a perspective camera looking straight down (40° field of view, map north up), with a 2D canvas drawn over it.

### 21.1 Area

The user clicks points to build a polygon. A new point is inserted into the edge nearest to it, so the polygon stays
simple.

- **Area** by the shoelace formula:

$$
A = \tfrac12\Big\lvert\sum_i (x_i z_{i+1} - x_{i+1} z_i)\Big\rvert .
$$

- **Margin.** The polygon is offset by $m$ metres. Each vertex moves along the bisector of its two edge normals
  $\mathbf n_1$ and $\mathbf n_2$:

$$
\mathbf p' = \mathbf p + (\mathbf n_1 + \mathbf n_2)\,\frac{m}{\max(1 + \mathbf n_1\cdot\mathbf n_2,\ 0.25)} .
$$

  Since $\lvert\mathbf n_1 + \mathbf n_2\rvert = 2\cos(\varphi/2)$ and $1 + \mathbf n_1\cdot\mathbf n_2 = 2\cos^2(\varphi/2)$,
  this moves the vertex $m/\cos(\varphi/2)$ along the bisector, the exact miter offset. The 0.25 floor limits sharp
  corners to about 2.8 m per metre of margin.

### 21.2 Camera footprint, line spacing and photo interval

The gimbal camera points straight down from altitude $h$. Its vertical field is VFOV; the sensor is 5:4.

$$
\text{HFOV} = 2\arctan\!\Big(\tfrac54\tan\tfrac{\text{VFOV}}2\Big),\qquad
W_\text{across} = 2h\tan\tfrac{\text{HFOV}}2,\qquad
W_\text{along} = 2h\tan\tfrac{\text{VFOV}}2,
$$

$$
\Delta_\text{line} = W_\text{across}\,(1 - o_\text{side}),\qquad
\Delta_\text{photo} = W_\text{along}\,(1 - o_\text{front}).
$$

Side overlap is therefore the line density, and doubling the altitude doubles the spacing. This is how photogrammetry
planners compute it.

### 21.3 The zigzag (boustrophedon) route

Let $\psi$ be the course angle.

1. **Rotate.** Each vertex is rotated into a frame aligned with the lines:
   - $u = \mathbf p\cdot\hat{\mathbf d}$, along the line, with $\hat{\mathbf d} = (\sin\psi, -\cos\psi)$;
   - $v = \mathbf p\cdot\hat{\mathbf e}$, across the lines, with $\hat{\mathbf e} = (\cos\psi, \sin\psi)$.
2. **Count and centre the lines.** With $n_L = \lfloor(v_\max - v_\min)/\Delta_\text{line}\rfloor + 1$, the lines sit at
   $v_k = v_0 + k\Delta_\text{line}$, where $v_0 = v_\min + \big((v_\max - v_\min) - (n_L - 1)\Delta_\text{line}\big)/2$.
3. **Intersect.** Each line $v = v_k$ meets each edge $(a, b)$ that it crosses at
   $u = u_a + (v_k - v_a)\dfrac{u_b - u_a}{v_b - v_a}$. The flown segment is $[\min u,\ \max u]$.
4. **Alternate.** The direction alternates line to line, giving the zigzag. "Change start point" flips the line order
   and/or the first direction, which cycles the four corners.
5. **Read-outs.** Route length is the polyline length. Photos are $\sum_\text{lines}\big(\lfloor\ell/\Delta_\text{photo}\rfloor + 1\big)$.
   Duration is $(\text{length} + \text{return leg})/V$.

### 21.4 Flying the route

The aircraft position is a function of arc length: $s \mathrel{+}= V\,\Delta t$ at constant speed $V$ (5–100 m/s,
stepless). The position is interpolated on the current segment. When the route ends:

- **Repeat:** wrap $s$ and count laps.
- **Return to home / hover:** stop and show the completion banner.

The heading turns toward the segment direction with a rate limit of 90°/s, as mapping flights keep the nose along
the line:

$$
\psi \mathrel{+}= \mathrm{clamp}\big(\mathrm{wrap}(\psi^* - \psi),\ \pm\tfrac\pi2\Delta t\big),\qquad
\psi^* = \operatorname{atan2}\big(-(b_x - a_x),\ -(b_z - a_z)\big).
$$

The camera, and with it all three sensors, sits at the set altitude. Its orientation is the YXZ Euler rotation
(yaw $\psi$, then gimbal pitch $\theta$, no roll). Vertical mouse drag changes only $\theta$, at 0.25° per pixel,
within [−90°, +30°]. Horizontal drag does nothing, so there is no pan. Space pauses. Esc restores every piece of view
state that was saved on entry: camera, field of view, target, gimbal source, camera presets, depth of field, motion
blur and panels.

---

## 22. Interface, language and performance

- **Two panels.**
  - Simple shows sea condition, weather, time of day and the number of people.
  - Professional shows everything.
  - Both are lil-gui panels. The Simple controls mirror the Professional ones.
- **Language.**
  - All text passes through `i18n(text, context)`. A Chinese table holds every label, option and OSD string.
  - Context keys (`Folder>Label`, `opt:Control>Option`, `sea>…`) keep identical English words apart when they
    translate differently.
  - The control panel is translated in place.
- **Quality and resolution.**
  - Four quality presets scale the cloud steps, LiDAR render size, ripple grid and so on.
  - **Automatic resolution** smooths the frame time. If FPS < 50 it steps the render scale down by 0.1 (to a
    minimum of 0.5) and waits 1.5 s. If FPS > 58 it steps up by 0.05 and waits 4 s.
- **Precision.**
  - The FFT runs in 32-bit float.
  - Rendering targets are half float. A half-float radiance target quantises temperature to about 0.04 K at 300 K,
    below the NETD, so it is dithered away.
  - The AGC readback and the LiDAR point buffer are 32-bit float.
- **Testing hooks.** `window.sim` exposes the state. `sim.tick(n, dt)` advances frames when the browser tab is hidden
  (where `requestAnimationFrame` is paused). Feature namespaces include `sim.lidar`, `sim.rec`, `sim.route` and
  `sim.seaState`. LAS/LAZ output was checked by decoding it with laspy + lazrs: every LAZ file decodes to exactly the
  records of its LAS twin, within 0.5 mm of the simulator's points.

---

## 23. Simplifications and known limits

These are choices made for real-time speed. They matter if the data is used quantitatively.

- **Waves.**
  - The sea is linear and Gaussian. Choppy displacement sharpens crests, but there is no true nonlinear wave
    interaction, no crest asymmetry from wave–wave interaction and no plunging breaker. Breaking is detected only
    where the surface folds ($J$).
  - The field repeats every 600 s, and each cascade tiles in space. Five incommensurate sizes hide the tiling.
  - Deep water everywhere; no currents other than the uniform surface drift.
- **People and ripples.** The FFT sea does not feel the bodies. Waves radiated and diffracted by the bodies come only
  from the separate linear ripple grid. Drag and inertia constants are empirical, tuned to look and move right, not
  measured.
- **Atmosphere.**
  - Latitude 32° and declination 12° are fixed.
  - The moon is a fixed light, not an ephemeris.
  - The thermal sky model is band-averaged, the air path is isothermal, and the camera has a flat 8–14 µm response.
    Real cameras weight the band by their spectral response.
- **Water emissivity.** One effective complex index represents the whole band. There is no polarisation output.
- **LiDAR.**
  - Single-bounce only, so there is no multipath.
  - Footprints are five samples of a rendered range image, so returns are limited by that image's resolution.
  - The link budget is normalised to the datasheet $R_{10}$, not computed from absolute laser power and aperture.
- **Route.**
  - Constant speed through the turns; no acceleration limits; no wind on the aircraft.
  - Take-off and landing are at the start point and altitude.
  - Each flight line spans the outermost edge crossings, so a concave notch is flown across instead of around.
- **Performance** has been measured only on the development machine.

---

## 24. References

Waves and the sea surface

- Tessendorf, J. (2001). *Simulating Ocean Water*. SIGGRAPH course notes.
- Hasselmann, K. et al. (1973). Measurements of wind-wave growth and swell decay during the Joint North Sea Wave
  Project (JONSWAP). *Deutsche Hydrographische Zeitschrift*, A8(12).
- Pierson, W. J. & Moskowitz, L. (1964). A proposed spectral form for fully developed wind seas. *J. Geophys. Res.* 69.
- Donelan, M. A., Hamilton, J. & Hui, W. H. (1985). Directional spectra of wind-generated waves. *Phil. Trans. R.
  Soc. A* 315.
- Banner, M. L. (1990). Equilibrium spectra of wind waves. *J. Phys. Oceanogr.* 20.
- Goda, Y. (2000). *Random Seas and Design of Maritime Structures*. World Scientific.
- Monahan, E. C. & O'Muircheartaigh, I. (1980). Optimal power-law description of oceanic whitecap coverage dependence
  on wind speed. *J. Phys. Oceanogr.* 10.
- Okubo, A. (1971). Oceanic diffusion diagrams. *Deep-Sea Research* 18.

Rendering

- Cox, C. & Munk, W. (1954). Measurement of the roughness of the sea surface from photographs of the sun's glitter.
  *J. Opt. Soc. Am.* 44.
- Olano, M. & Baker, D. (2010). LEAN mapping. *I3D 2010*.
- Walter, B., Marschner, S., Li, H. & Torrance, K. (2007). Microfacet models for refraction through rough surfaces.
  *EGSR 2007*.
- Bruneton, E. & Neyret, F. (2008). Precomputed atmospheric scattering. *Computer Graphics Forum* 27(4).
- Hillaire, S. (2020). A scalable and production ready sky and atmosphere rendering technique. *Computer Graphics
  Forum* 39(4).
- Cornette, W. M. & Shanks, J. G. (1992). Physically reasonable analytic expression for the single-scattering phase
  function. *Applied Optics* 31.
- Schneider, A. (2015). The real-time volumetric cloudscapes of Horizon Zero Dawn. *SIGGRAPH Advances in Real-Time
  Rendering*.
- Wrenninge, M. et al. (2013). Oz: the great and volumetric. *SIGGRAPH Talks*.
- Karis, B. (2014). High-quality temporal supersampling. *SIGGRAPH Advances in Real-Time Rendering*.
- Sobotka, T. AgX display rendering transform.
- Dendura, M. *web-ocean* (MIT licence): source of the ocean, shading, atmosphere and post-processing techniques
  adapted here.

Thermal infrared

- Hale, G. M. & Querry, M. R. (1973). Optical constants of water in the 200-nm to 200-µm wavelength region. *Applied
  Optics* 12.
- Friedman, D. (1969). Infrared characteristics of ocean water (1.5–15 µ). *Applied Optics* 8.
- Masuda, K., Takashima, T. & Takayama, Y. (1988). Emissivity of pure and sea waters for the model sea surface in the
  infrared window regions. *Remote Sensing of Environment* 24.
- Salisbury, J. W. et al. (1993); Niclòs, R. et al. (2007): emissivity of foam.
- Idso, S. B. (1981). A set of equations for full spectrum and 8- to 14-µm and 10.5- to 12.5-µm thermal radiation from
  cloudless skies. *Water Resources Research* 17.
- Bolton, D. (1980). The computation of equivalent potential temperature. *Monthly Weather Review* 108.
- Roberts, R. E., Selby, J. E. A. & Biberman, L. M. (1976). Infrared continuum absorption by atmospheric water vapor
  in the 8–12-µm window. *Applied Optics* 15.
- Donlon, C. J. et al. (2002). Toward improved validation of satellite sea surface skin temperature measurements for
  climate research. *J. Climate* 15.
- Steketee, J. (1973). Spectral emissivity of skin and pericardium. *Physics in Medicine and Biology* 18.
- ISO 9920 (2007). Ergonomics of the thermal environment: clothing thermal insulation; de Dear, R. et al. (1997).
  Convective and radiative heat transfer coefficients for individual human body segments. *Int. J. Biometeorology* 40.
- Sakuma, F. & Hattori, S. (1982). Establishing a practical temperature standard by using a narrow-band radiation
  thermometer with a silicon detector.

LiDAR and the atmosphere

- Kim, I. I., McArthur, B. & Korevaar, E. (2001). Comparison of laser beam propagation at 785 nm and 1550 nm in fog
  and haze for optical wireless communications. *Proc. SPIE* 4214.
- Koschmieder, H. (1924). Theorie der horizontalen Sichtweite. *Beiträge zur Physik der freien Atmosphäre* 12.
- Isenburg, M. (2013). LASzip: lossless compression of LiDAR data. *Photogrammetric Engineering & Remote Sensing* 79(2).
- Said, A. (2004). Introduction to arithmetic coding: theory and practice (FastAC). HP Labs.
- ASPRS (2008). *LAS Specification Version 1.2*.

Underwater optics

- Jerlov, N. G. (1976). *Marine Optics*. Elsevier.
- Mobley, C. D. (1994). *Light and Water: Radiative Transfer in Natural Waters*. Academic Press.
