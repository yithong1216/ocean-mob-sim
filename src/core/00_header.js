/*
 * Ocean MOB (man overboard) simulator — three.js / WebGL2, single file.
 * Author: Yit Hong Wong (https://github.com/yithong1216)
 *
 * The wave simulation (Tessendorf FFT cascades, fetch-limited JONSWAP + Donelan-Banner spreading, swell),
 * ocean shading (LEAN slope moments, anisotropic sun glitter, grazing-angle reflection tilt, subsurface glow),
 * world-space foam with decay and drift, the procedural foam texture, the Hillaire-style atmosphere LUTs,
 * the cloud layers and the bloom + AgX look are adapted from "web-ocean" (WebGPU/WGSL), ported to GLSL here.
 *
 * web-ocean licence:
 *   MIT License
 *   Copyright (c) 2026 Maksymilian Dendura
 *   Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
 *   documentation files (the "Software"), to deal in the Software without restriction, including without limitation
 *   the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and
 *   to permit persons to whom the Software is furnished to do so, subject to the following conditions:
 *   The above copyright notice and this permission notice shall be included in all copies or substantial portions
 *   of the Software.
 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO
 *   THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
 *   CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 *   DEALINGS IN THE SOFTWARE.
 *
 * Human models (loaded at runtime from the three.js repository via jsDelivr): readyplayer.me.glb and Michelle.glb
 * from mrdoob/three.js examples/models/gltf.
 */
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import * as SkeletonUtils from 'three/addons/utils/SkeletonUtils.js';
import { EffectComposer } from 'three/addons/postprocessing/EffectComposer.js';
import { RenderPass } from 'three/addons/postprocessing/RenderPass.js';
import { UnrealBloomPass } from 'three/addons/postprocessing/UnrealBloomPass.js';
import { ShaderPass } from 'three/addons/postprocessing/ShaderPass.js';
import GUI from 'lil-gui';

