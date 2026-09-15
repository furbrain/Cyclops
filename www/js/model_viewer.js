// Three.js viewer for a single model's assets.
//
// Assets are split into two layers:
//   - "base" (pointcloud / sparse / dense / textured): mutually exclusive,
//     one shown at a time, picked from the main asset-btn group.
//   - "overlay" (track / survey): independently toggleable on top of
//     whichever base asset is showing, picked from the overlay-btn group.
//
// Uses three@0.170 via the importmap in model.html — "three.js 4.5" isn't a real
// version string, versions run r1xx or, since 2023, 0.1xx.y. Bump the importmap
// if you need a different pinned build.

import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { PLYLoader } from "three/addons/loaders/PLYLoader.js";
import { GLTFLoader } from "three/addons/loaders/GLTFLoader.js";

const modelId = window.MODEL_ID;
let assetStatus = window.ASSET_STATUS || {};
const assetFormats = window.ASSET_FORMATS || {};

const container = document.getElementById("viewer");
const statusEl = document.getElementById("viewer-status");
const generateBtn = document.getElementById("generate-btn");
const resetViewBtn = document.getElementById("reset-view-btn");
const assetButtons = [...document.querySelectorAll(".asset-btn")];
const overlayButtons = [...document.querySelectorAll(".overlay-btn")];

// --- Scene setup -----------------------------------------------------------

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x121417);

const camera = new THREE.PerspectiveCamera(60, 1, 0.01, 5000);
camera.position.set(2, 2, 2);

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(window.devicePixelRatio);
container.appendChild(renderer.domElement);

const controls = new OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;

scene.add(new THREE.AmbientLight(0xffffff, 0.6));
const dirLight = new THREE.DirectionalLight(0xffffff, 0.8);
dirLight.position.set(5, 10, 7);
scene.add(dirLight);

const grid = new THREE.GridHelper(10, 10, 0x2a2e33, 0x1c1f22);
scene.add(grid);

const plyLoader = new PLYLoader();
const gltfLoader = new GLTFLoader();

// The single base asset currently shown.
let currentObject = null;
let currentAsset = null;

// Overlay assets — loaded lazily, kept around once loaded (added/removed
// from the scene on toggle rather than re-fetched every time), independent
// of whatever base asset is currently showing.
const overlayObjects = { track: null, survey: null };
const overlayVisible = { track: false, survey: false };

// Your SLAM/survey data is Z-up (IMU ENU convention: X-east, Y-north,
// Z-up), but three.js is Y-up. Left as-is, whichever axis is "up" in your
// data (Z) gets drawn along three's X/depth axes instead, which reads as
// the model being tipped 90° with a horizontal axis standing vertical.
// Rotating -90° about X maps source (x, y, z) -> three (x, z, -y), putting
// your Z where three expects "up". Set to "y" if you ever export
// already-Y-up data and this becomes a no-op.
const SOURCE_UP_AXIS = "z";

function applySourceAxisConvention(object) {
  if (SOURCE_UP_AXIS === "z") {
    object.rotateX(-Math.PI / 2);
  }
}

// Which position component is "elevation" in *source* coordinates (before
// applySourceAxisConvention rotates the object into three's frame) — keep
// this in sync with SOURCE_UP_AXIS above.
const ELEVATION_AXIS_INDEX = SOURCE_UP_AXIS === "z" ? 2 : 1;

// Pointclouds/track/survey rarely carry real color, and untextured PLY
// meshes often don't either. Rather than render everything as one flat
// tone, fall back to an elevation-driven gradient — the standard way cave
// survey tools (Survex, Tunnel, etc.) make depth and passage shape
// readable without real color data.
//
// baseHue null       -> full rainbow by elevation (blue low -> red high).
//                        Used for the point cloud/mesh, where there's no
//                        "identity color" to preserve.
// baseHue 0-1         -> keep that hue (so track/survey stay visually
//                        distinct from each other) and vary lightness by
//                        elevation instead — dim low, bright high.
function addElevationColorFallback(geometry, baseHue = null) {
  const position = geometry.getAttribute("position");
  if (!position || position.count === 0) return;

  let min = Infinity;
  let max = -Infinity;
  for (let i = 0; i < position.count; i++) {
    const v = position.getComponent(i, ELEVATION_AXIS_INDEX);
    if (v < min) min = v;
    if (v > max) max = v;
  }
  const range = max - min || 1;

  const colors = new Float32Array(position.count * 3);
  const c = new THREE.Color();
  for (let i = 0; i < position.count; i++) {
    const t = (position.getComponent(i, ELEVATION_AXIS_INDEX) - min) / range;
    if (baseHue === null) {
      c.setHSL(0.7 * (1 - t), 1, 0.5); // 0.7 = blue, 0 = red
    } else {
      c.setHSL(baseHue, 0.85, 0.25 + 0.5 * t); // dim -> bright with elevation
    }
    colors[i * 3] = c.r;
    colors[i * 3 + 1] = c.g;
    colors[i * 3 + 2] = c.b;
  }
  geometry.setAttribute("color", new THREE.BufferAttribute(colors, 3));
}

// Hues used to keep track/survey visually distinct even when neither has
// real color data (see addElevationColorFallback above).
const TRACK_HUE = 0.09; // orange
const SURVEY_HUE = 0.33; // green

function resize() {
  const w = container.clientWidth;
  const h = container.clientHeight || 480;
  renderer.setSize(w, h);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
}
new ResizeObserver(resize).observe(container);
resize();

(function animate() {
  requestAnimationFrame(animate);
  controls.update();
  renderer.render(scene, camera);
})();

// --- Asset loading -----------------------------------------------------------

function disposeMaterial(material) {
  for (const key of ["map", "normalMap", "roughnessMap", "metalnessMap", "aoMap", "emissiveMap"]) {
    material[key]?.dispose?.();
  }
  material.dispose?.();
}

// A plain PLY/points object is a single Mesh/Points/Line; a GLB brings in
// a whole Group of meshes, each with its own geometry/material(s) and
// texture(s) — traverse either way so nothing leaks.
function disposeObject3D(object) {
  object.traverse((node) => {
    node.geometry?.dispose?.();
    if (Array.isArray(node.material)) {
      node.material.forEach(disposeMaterial);
    } else if (node.material) {
      disposeMaterial(node.material);
    }
  });
}

function clearObject() {
  if (currentObject) {
    scene.remove(currentObject);
    disposeObject3D(currentObject);
    currentObject = null;
  }
}

function disposeOverlay(key) {
  if (overlayObjects[key]) {
    scene.remove(overlayObjects[key]);
    disposeObject3D(overlayObjects[key]);
    overlayObjects[key] = null;
  }
}

// Frames the camera on whatever's currently visible (base object + any
// visible overlays) combined into one bounding box. Only called on the
// very first successful load and from the explicit "Reset view" button —
// NOT on every asset switch/overlay toggle, so your viewpoint sticks once
// you've set it up, instead of jumping back to a fitted default each time.
function frameCombinedView() {
  const box = new THREE.Box3();
  let any = false;
  if (currentObject) {
    box.expandByObject(currentObject);
    any = true;
  }
  for (const key of Object.keys(overlayObjects)) {
    if (overlayVisible[key] && overlayObjects[key]) {
      box.expandByObject(overlayObjects[key]);
      any = true;
    }
  }
  if (!any || box.isEmpty()) return;

  const size = box.getSize(new THREE.Vector3()).length();
  const center = box.getCenter(new THREE.Vector3());
  controls.target.copy(center);
  const dist = Math.max(size, 0.01) * 1.2;
  camera.position.copy(center).add(new THREE.Vector3(dist, dist * 0.6, dist));
  camera.near = dist / 100;
  camera.far = dist * 100;
  camera.updateProjectionMatrix();
}

let hasFramedInitialView = false;
function maybeFrameInitialView() {
  if (!hasFramedInitialView) {
    frameCombinedView();
    hasFramedInitialView = true;
  }
}

function buildMeshFromGeometry(geometry) {
  geometry.computeVertexNormals?.();
  if (!geometry.hasAttribute("color")) {
    addElevationColorFallback(geometry);
  }

  // sparse / dense mesh variants (loaded from PLY, may or may not carry
  // real vertex colors — falls back to the elevation gradient above)
  const material = new THREE.MeshStandardMaterial({
    color: 0xffffff,
    vertexColors: true,
    side: THREE.DoubleSide,
  });
  return new THREE.Mesh(geometry, material);
}

// Build a Points/Line/LineSegments object straight from a plain
// {"points": [[x,y,z],...], "colors": [[r,g,b],...]?} JSON payload —
// no PLY parsing involved. `colors` is optional and 0-255 per channel.
function buildPointsObjectFromJSON(data, assetKey) {
  const points = data.points || [];
  const geometry = new THREE.BufferGeometry();

  const positions = new Float32Array(points.length * 3);
  points.forEach(([x, y, z], i) => {
    positions[i * 3] = x;
    positions[i * 3 + 1] = y;
    positions[i * 3 + 2] = z;
  });
  geometry.setAttribute("position", new THREE.BufferAttribute(positions, 3));

  if (data.colors && data.colors.length === points.length) {
    const colors = new Float32Array(data.colors.length * 3);
    data.colors.forEach(([r, g, b], i) => {
      colors[i * 3] = r / 255;
      colors[i * 3 + 1] = g / 255;
      colors[i * 3 + 2] = b / 255;
    });
    geometry.setAttribute("color", new THREE.BufferAttribute(colors, 3));
  } else {
    const baseHue = assetKey === "track" ? TRACK_HUE : assetKey === "survey" ? SURVEY_HUE : null;
    addElevationColorFallback(geometry, baseHue);
  }

  const hasColor = geometry.hasAttribute("color");

  if (assetKey === "pointcloud") {
    const material = new THREE.PointsMaterial({
      size: 0.05,
      vertexColors: hasColor,
      color: 0xffffff,
    });
    return new THREE.Points(geometry, material);
  }

  if (assetKey === "track") {
    // Camera trajectory — one continuous path through the points in order.
    const material = new THREE.LineBasicMaterial({ vertexColors: hasColor, color: 0xffffff });
    return new THREE.Line(geometry, material);
  }

  // survey — each consecutive pair of points (0-1, 2-3, ...) is one leg,
  // not a single continuous path (the cave can branch).
  const material = new THREE.LineBasicMaterial({ vertexColors: hasColor, color: 0xffffff });
  return new THREE.LineSegments(geometry, material);
}

// Loads a base asset (pointcloud / sparse / dense / textured) — mutually
// exclusive with whatever base asset was showing before.
function loadAsset(assetKey) {
  currentAsset = assetKey;
  updateButtonStates();

  if (!assetStatus[assetKey]) {
    clearObject();
    statusEl.textContent = `"${labelFor(assetKey)}" hasn't been generated yet.`;
    generateBtn.classList.remove("d-none");
    generateBtn.disabled = false;
    return;
  }

  generateBtn.classList.add("d-none");
  statusEl.textContent = `Loading ${labelFor(assetKey)}…`;

  const url = `/model/api/${modelId}/asset/${assetKey}`;
  const format = assetFormats[assetKey] || "ply";

  if (format === "points") {
    fetch(url)
      .then((res) => {
        if (!res.ok) throw new Error(`HTTP ${res.status}`);
        return res.json();
      })
      .then((data) => {
        clearObject();
        currentObject = buildPointsObjectFromJSON(data, assetKey);
        applySourceAxisConvention(currentObject);
        scene.add(currentObject);
        maybeFrameInitialView();
        statusEl.textContent = labelFor(assetKey);
      })
      .catch((err) => {
        console.error(err);
        statusEl.textContent = `Failed to load ${labelFor(assetKey)}: ${err.message || err}`;
      });
    return;
  }

  if (format === "glb") {
    gltfLoader.load(
      url,
      (gltf) => {
        clearObject();
        // gltf.scene already carries its own materials/textures/UVs —
        // no need to build a material by hand like the plain-PLY path.
        currentObject = gltf.scene;
        applySourceAxisConvention(currentObject);
        scene.add(currentObject);
        maybeFrameInitialView();
        statusEl.textContent = labelFor(assetKey);
      },
      undefined,
      (err) => {
        console.error(err);
        statusEl.textContent = `Failed to load ${labelFor(assetKey)}: ${err.message || err}`;
      }
    );
    return;
  }

  // format === "ply"
  plyLoader.load(
    url,
    (geometry) => {
      clearObject();
      currentObject = buildMeshFromGeometry(geometry);
      applySourceAxisConvention(currentObject);
      scene.add(currentObject);
      maybeFrameInitialView();
      statusEl.textContent = labelFor(assetKey);
    },
    undefined,
    (err) => {
      console.error(err);
      statusEl.textContent = `Failed to load ${labelFor(assetKey)}: ${err.message || err}`;
    }
  );
}

function labelFor(assetKey) {
  const btn = [...assetButtons, ...overlayButtons].find(
    (b) => (b.dataset.asset || b.dataset.overlay) === assetKey
  );
  return btn ? btn.textContent.trim() : assetKey;
}

function updateButtonStates() {
  for (const btn of assetButtons) {
    const key = btn.dataset.asset;
    btn.classList.toggle("active", key === currentAsset);
    btn.classList.toggle("btn-outline-warning", !assetStatus[key]);
    btn.classList.toggle("btn-outline-light", !!assetStatus[key]);
  }
}

// --- Overlays (track / survey) ----------------------------------------------
//
// Independent of the base asset selection above: each can be toggled on or
// off, loaded once and kept in memory (added/removed from the scene rather
// than re-fetched every toggle), and shown at the same time as each other
// and as whatever base asset is currently selected.

function updateOverlayButtonState(key) {
  const btn = overlayButtons.find((b) => b.dataset.overlay === key);
  if (!btn) return;

  if (!assetStatus[key]) {
    btn.classList.remove("btn-outline-light", "btn-light", "active");
    btn.classList.add("btn-outline-warning");
    return;
  }

  btn.classList.remove("btn-outline-warning");
  const on = !!overlayVisible[key];
  btn.classList.toggle("active", on);
  btn.classList.toggle("btn-light", on);
  btn.classList.toggle("btn-outline-light", !on);
}

// Shows/hides an overlay, fetching+building it the first time it's shown.
function setOverlayVisible(key, visible) {
  overlayVisible[key] = visible;
  updateOverlayButtonState(key);

  if (!visible) {
    if (overlayObjects[key]) scene.remove(overlayObjects[key]);
    return Promise.resolve();
  }

  if (overlayObjects[key]) {
    scene.add(overlayObjects[key]);
    maybeFrameInitialView();
    return Promise.resolve();
  }

  const url = `/model/api/${modelId}/asset/${key}`;
  return fetch(url)
    .then((res) => {
      if (!res.ok) throw new Error(`HTTP ${res.status}`);
      return res.json();
    })
    .then((data) => {
      const obj = buildPointsObjectFromJSON(data, key);
      applySourceAxisConvention(obj);
      overlayObjects[key] = obj;
      scene.add(obj);
      maybeFrameInitialView();
    })
    .catch((err) => {
      console.error(err);
      overlayVisible[key] = false;
      updateOverlayButtonState(key);
      statusEl.textContent = `Failed to load ${labelFor(key)}: ${err.message || err}`;
    });
}

// --- Background job polling (generation output, merge output) --------------

const jobOutput = document.getElementById("job-output");
const jobStateBadge = document.getElementById("job-state-badge");
const mergeBtn = document.getElementById("merge-btn");
const submapsValueEl = document.getElementById("submaps-value");

// Only one job (generate or merge) at a time — they share the same output
// panel, so running two at once would interleave unrelated logs.
let jobRunning = false;

function setJobBadge(state) {
  jobStateBadge.textContent = state;
  jobStateBadge.className =
    "badge " +
    (state === "running"
      ? "text-bg-info"
      : state === "done"
      ? "text-bg-success"
      : state === "error"
      ? "text-bg-danger"
      : "text-bg-secondary");
}

// Polls `statusUrl` (a .../status endpoint returning {"state", "log"})
// every 800ms while the job is running, streaming its output into the
// output panel, and calls onDone/onError once it settles.
function pollJob(statusUrl, { onDone, onError } = {}) {
  const tick = () => {
    fetch(statusUrl)
      .then((res) => res.json())
      .then((data) => {
        jobOutput.textContent = data.log || "(no output yet)";
        jobOutput.scrollTop = jobOutput.scrollHeight;
        setJobBadge(data.state);

        if (data.state === "running") {
          setTimeout(tick, 800);
        } else if (data.state === "done") {
          onDone?.();
        } else if (data.state === "error") {
          onError?.(data.log);
        }
      })
      .catch((err) => {
        // Keep trying through a transient network blip rather than
        // silently going stale.
        console.error(err);
        setTimeout(tick, 2000);
      });
  };
  tick();
}

// Kicks off generation of `assetKey` (base or overlay) and resolves once
// it's done, or rejects with a message once it's failed. Guards against
// two jobs running at once since they'd interleave in the shared output
// panel.
function generateAsset(assetKey) {
  if (jobRunning) return Promise.reject(new Error("Another job is already running"));
  jobRunning = true;
  jobOutput.textContent = "";
  setJobBadge("running");

  return fetch(`/model/api/${modelId}/generate/${assetKey}`, { method: "POST" })
    .then((res) => {
      if (!res.ok) {
        return res
          .json()
          .catch(() => ({}))
          .then((body) => {
            throw new Error(body.error || `HTTP ${res.status}`);
          });
      }
      return new Promise((resolve, reject) => {
        pollJob(`/model/api/${modelId}/generate/${assetKey}/status`, {
          onDone: () => {
            assetStatus[assetKey] = true;
            resolve();
          },
          onError: () => reject(new Error(`Generation of "${labelFor(assetKey)}" failed — see output above.`)),
        });
      });
    })
    .finally(() => {
      jobRunning = false;
    });
}

function runMerge() {
  if (jobRunning) return Promise.reject(new Error("Another job is already running"));
  jobRunning = true;
  jobOutput.textContent = "";
  setJobBadge("running");

  return fetch(`/model/api/${modelId}/merge`, { method: "POST" })
    .then((res) => {
      if (!res.ok) {
        return res
          .json()
          .catch(() => ({}))
          .then((body) => {
            throw new Error(body.error || `HTTP ${res.status}`);
          });
      }
      return new Promise((resolve, reject) => {
        pollJob(`/model/api/${modelId}/merge/status`, {
          onDone: resolve,
          onError: () => reject(new Error("Merge failed — see output above.")),
        });
      });
    })
    .finally(() => {
      jobRunning = false;
    });
}

// After a merge completes, re-fetch info.json rather than guessing the new
// submap count client-side — the backend is the source of truth for it.
function refreshSubmapsFromServer() {
  return fetch(`/model/api/${modelId}/info`)
    .then((res) => res.json())
    .then((info) => {
      submapsValueEl.textContent = info.submaps ?? "—";
      mergeBtn.disabled = !(info.submaps > 1);
    })
    .catch((err) => console.error(err));
}

// --- UI wiring -----------------------------------------------------------

assetButtons.forEach((btn) => {
  btn.addEventListener("click", () => loadAsset(btn.dataset.asset));
});

overlayButtons.forEach((btn) => {
  const key = btn.dataset.overlay;
  btn.addEventListener("click", () => {
    if (jobRunning) return;

    if (!assetStatus[key]) {
      btn.disabled = true;
      generateAsset(key)
        .then(() => setOverlayVisible(key, true))
        .catch((err) => {
          statusEl.textContent = err.message;
        })
        .finally(() => {
          btn.disabled = false;
          updateOverlayButtonState(key);
        });
      return;
    }

    setOverlayVisible(key, !overlayVisible[key]);
  });
});

generateBtn.addEventListener("click", () => {
  if (!currentAsset || jobRunning) return;
  generateBtn.disabled = true;
  statusEl.textContent = `Generating ${labelFor(currentAsset)}…`;

  generateAsset(currentAsset)
    .then(() => loadAsset(currentAsset))
    .catch((err) => {
      statusEl.textContent = err.message;
    })
    .finally(() => {
      generateBtn.disabled = false;
    });
});

mergeBtn.addEventListener("click", () => {
  if (jobRunning) return;
  mergeBtn.disabled = true;

  runMerge()
    .then(refreshSubmapsFromServer)
    .catch((err) => {
      jobOutput.textContent += `\n${err.message}`;
    })
    .finally(() => {
      mergeBtn.disabled = false;
    });
});

resetViewBtn.addEventListener("click", () => frameCombinedView());

updateButtonStates();
overlayButtons.forEach((btn) => updateOverlayButtonState(btn.dataset.overlay));
// Load the point cloud by default if it exists, otherwise show the first asset.
loadAsset(assetStatus.pointcloud ? "pointcloud" : (assetButtons[0]?.dataset.asset ?? "pointcloud"));

// --- Survey form -----------------------------------------------------------

const surveyForm = document.getElementById("survey-form");
const surveyText = document.getElementById("survey-text");
const surveyFile = document.getElementById("survey-file");
const surveyStatus = document.getElementById("survey-status");
const surveyClearFile = document.getElementById("survey-clear-file");

surveyClearFile.addEventListener("click", () => {
  surveyFile.value = "";
});

// When a file is picked, read it and show its content in the textbox.
// The file input is then cleared so "Save survey" always sends what's in
// the textarea — one clear source of truth, and the user can tweak the
// text after upload before saving.
surveyFile.addEventListener("change", () => {
  const file = surveyFile.files[0];
  if (!file) return;

  const reader = new FileReader();
  reader.onload = () => {
    surveyText.value = reader.result;
    surveyFile.value = "";
    surveyStatus.textContent = `Loaded "${file.name}" (${reader.result.length} chars) — not saved yet`;
  };
  reader.onerror = () => {
    surveyStatus.textContent = `Couldn't read "${file.name}": ${reader.error}`;
  };
  reader.readAsText(file);
});

// Prefill the textarea with any existing survey.
fetch(`/model/api/${modelId}/survey`)
  .then((r) => r.json())
  .then((data) => {
    if (data.text) surveyText.value = data.text;
  })
  .catch(() => {});

surveyForm.addEventListener("submit", async (e) => {
  e.preventDefault();
  surveyStatus.textContent = "Saving…";

  // File uploads are read into the textarea on selection (see the
  // surveyFile "change" handler above), so by the time this fires the
  // textarea is always the source of truth.
  const formData = new FormData();
  formData.append("text", surveyText.value);

  try {
    const res = await fetch(`/model/api/${modelId}/survey`, { method: "POST", body: formData });
    const data = await res.json();
    if (!res.ok) throw new Error(data.error || `HTTP ${res.status}`);
    surveyStatus.textContent = `Saved (${data.length} chars)`;

    // The survey overlay (if any) is now stale relative to the new text —
    // drop it and turn the overlay off; the user re-generates explicitly
    // by clicking the Survey button again.
    assetStatus.survey = false;
    disposeOverlay("survey");
    overlayVisible.survey = false;
    updateOverlayButtonState("survey");
  } catch (err) {
    surveyStatus.textContent = `Error: ${err.message}`;
  }
});