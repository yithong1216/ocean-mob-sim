# Ocean MOB simulator

Author: Yit Hong Wong ([@yithong1216](https://github.com/yithong1216))

A real-time, physically based open-ocean scene with people overboard (man overboard, MOB), written in three.js / WebGL2.
It runs in a web browser with nothing to install. The page includes:
- FFT wind waves and swell, a physical sky, volumetric clouds and god rays;
- weather presets (clear, fair cumulus, overcast, storm with lightning, marine fog, rain);
- floating people in lifejackets whose buoyancy follows the waves, with ripples, wakes and spray around them;
- reflections of the people in the water, and sun shadows;
- an underwater view (Snell's window, caustics, light shafts) and rescue gear (lifebuoy, raft, debris, vessel, dye marker);
- cinematic camera modes (sea level, drone, helicopter, rescue boat), depth of field, lens flare and droplets, and auto exposure;
- co-registered RGB and thermal-infrared (8–14 µm) camera windows;
- a LiDAR window with a physically based point cloud. It has solid-state rosette scanners, DJI L2-type
  scanners (up to 5 returns per pulse) and spinning 32/64/128-beam scanners, at 905 or 1550 nm, and exports the
  cloud as a .pcd file.

## Sharing it

`ocean_mob_sim.html` is the whole application in one file (about 575 KB). It loads three.js, the GUI library and the
two human models from the jsDelivr CDN when it opens.

| Way | How | Recipient needs |
|---|---|---|
| Send the file | Email, WeChat or a cloud drive: send `ocean_mob_sim.html`; they double-click it | Chrome or Edge, internet connection |
| Share a link | Drag this folder onto **https://app.netlify.com/drop**, or put it in a GitHub repo and turn on Pages | Just the link |
| Show it live | Open it in Chrome and share your screen | Nothing |

A reasonably modern GPU is needed. The **Performance** folder in the GUI has a quality preset, and automatic resolution
scaling is on by default, so slower laptops still run smoothly.

If a double-clicked file shows a blank page (some locked-down browsers block CDN scripts on local files), use the
link option instead, or run a local server in this folder:

```
python -m http.server 8000
```

then open http://localhost:8000/ocean_mob_sim.html.

## Controls

- Two switches sit at the top right, next to the control panel. The browser remembers both choices.
  - **Simple / Professional** chooses the panel. Simple shows four controls (sea condition, weather, time of day,
    number of people in the water) and is what a first visit opens. Professional shows every setting.
  - **EN / 中文** switches the whole interface between English and Simplified Chinese. On a first visit it follows
    the browser language.
- Sea condition (海况等级) is levels 0–9 of the sea-state scale, 无浪 … 怒涛:
  - Each level gives a fully developed wind sea in the middle of its wave-height band (for example level 4,
    Moderate, 1.25–2.5 m). It sets the wind for that level, the matching fetch, whitecaps that grow with the wind,
    and a moderate swell.
  - The panel shows the level's textbook description and the wave height now in the water.
  - After you pick a level, weather presets change only the sky. Moving the wind or wave sliders in the
    Professional panel takes over again, and the level then follows the waves.
- Drag to orbit, mouse wheel to zoom, right-drag to pan.
- The panel on the right has sky and light, wind and waves, water, people overboard, the EO/IR camera windows,
  performance and the feature modules.
- The windows at the bottom are an RGB camera, an LWIR thermal camera and a LiDAR. By default they follow your
  view; switch them to the drone gimbal under "Drone EO / IR cameras".
- LiDAR window: drag inside it to orbit, use the wheel to zoom, and double-click to reset. The "LiDAR" folder has:
  - the scanner model, wavelength, view (3D, sensor POV, bird's-eye) and colouring (reflectivity, range, height,
    return type);
  - "Export point cloud", which saves the current cloud.
    - Formats ("Export format"): PCD as text, LAS 1.2, or LAZ.
    - Coordinates ("LiDAR coordinates"): the sensor frame (x forward, y left, z up) or the world frame (x east,
      y north, z up).
    - PCD fields are intensity (apparent reflectance), a label (1 water, 2 foam, 3 person, 4 retroreflector,
      5 object, 6 clutter, 7 blooming) and the return number (1 = first).
- LAS / LAZ details:
  - Point format 1: millimetre coordinates, return number and number of returns, GPS time (simulation time of each
    pulse), scan angle, and the frame number as point source ID.
  - Intensity is reflectance × 32768: a white card is about 32768, and retroreflectors saturate at 65535.
  - Classification uses the standard codes 9 water and 7 noise, 1 for objects, and two custom codes: 19 person and
    20 retroreflector. The user-data byte holds the simulator's own label.
  - LAZ is about a third of the size of LAS. It is written with a JavaScript port of the LASzip encoder and opens
    in PDAL, CloudCompare, LAStools, QGIS and laspy.
- DJI L2-type LiDAR, as specified for the L2:
  - 905 nm, 240,000 pulses/s;
  - 1, 2, 3 or 5 returns per pulse ("Returns per pulse"), so up to 1.2 million points/s;
  - about 250 m at 10 % reflectivity in full sun, 450 m maximum, 3 m blind zone, 2 cm range noise;
  - a 70° × 75° non-repetitive field, or a 70° × 3° repetitive field.
- How later returns arise in the simulation:
  - each pulse's footprint is sampled at five points;
  - samples that hit different surfaces become separate echoes, for example a head and the sea behind it, or the
    edge of a raft;
  - spray above breaking crests (from about sea state 5) and fog or rain droplets add echoes in front of the
    water.
- LiDAR colouring:
  - "Return number" shows the 1st to 5th returns;
  - "True colour (RGB camera)" colours each point from the EO camera image, as the L2 does with its built-in
    camera.
- **✈ Flight route**: a mapping mission planner like DJI Pilot 2. Open it with the button at the top right.
  - **Drawing the area:**
    - Click on the sea to add boundary points (3 or more); each new point joins the nearest edge.
    - Drag a point to move it; drag a **+** on an edge to add a point; Delete removes the selected point.
    - The toolbar on the left can clear the area, delete a point, draw a rectangle, or centre the view.
    - Drag the map to pan and use the wheel to zoom.
  - **The route:** the system fills the area with parallel flight lines flown back and forth (a zigzag).
    - **Course angle** sets the line direction.
    - **Side overlap** sets the line density. Line spacing = camera ground footprint × (1 − overlap), and the
      footprint grows with altitude.
    - **Front overlap** sets the photo interval.
  - **Other settings:** altitude 10–1000 m (with −100 … +100 buttons), speed 5–100 m/s (stepless), margin, gimbal
    pitch, completion action (return to home, hover, or repeat), start corner, and an option to record the three
    sensors during the flight with the Data recording settings.
  - **Read-outs:** the panel shows area, route length, estimated duration, estimated photos, line spacing and number
    of lines. The area and settings are kept in the browser.
  - **Flying:** **Start flight** flies the route at constant altitude and speed from the start point S. The heading
    follows the route.
    - The main view and the RGB, IR and LiDAR windows show the gimbal view.
    - The mouse only tilts the gimbal (drag up or down); it cannot pan. Space pauses, and **Esc** leaves the flight
      and restores the previous view.
    - A top bar shows status, progress, altitude, speed, distance from home, time left and gimbal angle. There is
      also a gimbal-angle gauge, and a minimap with the route, the people and the aircraft.
- Data recording: in the Professional panel, the **Data recording** folder saves the three sensors frame by
  frame.
  - **Choose storage folder…** picks a folder on your disk (Chrome or Edge). The browser remembers it and asks
    for permission again on a later visit. Without a folder, or in Firefox and Safari, the recording downloads
    as one `.zip` when you stop (up to 1.5 GB).
  - Settings: session name, capture rate (1–30 Hz), RGB resolution (640×512 to 2560×2048) and format
    (JPEG or PNG), IR resolution (320×256 to 1280×1024), what to save, LiDAR format (PCD, LAS or LAZ) and
    coordinates (sensor or world),
    "Never drop frames", and an automatic stop time.
  - "Never drop frames" slows the simulation when the computer or disk can't keep up, so every frame is saved.
    Turn it off to keep real time; frames that can't be saved in time are then counted as dropped.
  - Frames follow simulation time, so pausing the simulation pauses the recording.
  - A red REC badge at the top shows time, frames, size and drops, with a Stop button.
  - Each session is a folder `<name>_<date>_<time>/`:

    | Path | Content |
    |---|---|
    | `rgb/000000.jpg` | EO camera image, tone-mapped as in its window |
    | `ir/000000.png` | LWIR image as the camera shows it (gain control, palette, noise) |
    | `ir_raw/000000.pgm` | 16-bit radiometric image: apparent temperature in centi-kelvin (T [K] = value / 100); OpenCV reads it with `IMREAD_ANYDEPTH` |
    | `lidar/000000.pcd` (or `.las` / `.laz`) | the points from the pulses fired since the previous frame (a 360° scanner gives its latest full turn); binary PCD with fields x y z intensity label return, or LAS / LAZ as above |
    | `labels/000000.txt` | YOLO boxes of the people (`0 cx cy w h`). Each box covers the part above the water. RGB and IR share one co-registered pose, so one file fits both |
    | `meta/000000.json` | simulation time, camera and LiDAR poses, environment (sea state, wind, weather, temperatures), and per-person boxes in pixels for each image size |
    | `session.json` | settings, camera intrinsics (fx, fy, cx, cy), LiDAR description, coordinate conventions, class names, frame and drop counts |

- What the LiDAR models:
  - glints from wave facets that face the sensor, which is why calm water at grazing angles returns almost nothing;
  - foam;
  - material reflectance at 905 / 1550 nm (wet skin is dark at 1550 nm);
  - retroreflective tape, including blooming around it;
  - range loss, noise and clutter in fog and rain.

## Editing (developers)

`TECHNICAL_BOOK.md` explains how the simulator is built and the mathematics and physics behind each part (waves,
sky, thermal infrared, LiDAR, file formats, route planner).

The single file is generated from small source files:

```
src/page.html            page shell, styles, import map
src/core/00..09_*.js     engine: renderer, atmosphere, FFT ocean, people, GUI, sensor cameras
src/features/*.js        optional feature modules (each adds its own GUI folder)
src/core/99_main.js      main loop and start-up
build.mjs                bundles everything into ocean_mob_sim.html
```

Rebuild after editing (Node 18+, no packages needed):

```
node build.mjs
```

Interface text is written in English and passed through `i18n()`. The Chinese is in the `ZH` table in
`src/core/01_i18n.js`, so a new label needs one entry there. The control panel is translated in place by
`src/features/zz_language.js`.

Feature modules hook into the engine without editing it:
- `HOOKS` in `src/core/01_setup.js` has update, lateUpdate, beforeRender, afterRender, resize, personShader and ready.
- `injectShader(material, hook, glsl)` inserts code at the `// @hook:...` markers in the ocean and sky shaders.

## Credits and licences

- The ocean simulation, ocean shading, atmosphere and post-processing techniques are adapted from **web-ocean** by
  Maksymilian Dendura (MIT licence; the full notice is in the header of the generated file).
- The LAZ writer is a JavaScript port of the encoder side of **LASzip** by Martin Isenburg / rapidlasso GmbH
  (Apache License 2.0); the notice is in `src/features/sensor_lidar_las.js`.
- three.js (MIT) and lil-gui (MIT) are loaded from jsDelivr.
- The human models `readyplayer.me.glb` and `Michelle.glb` come from the three.js repository examples
  (mrdoob/three.js `examples/models/gltf`). Check their individual licences before any commercial use.
