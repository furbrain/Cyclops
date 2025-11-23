
const canvas = document.getElementById("c");

// Create scene, camera, renderer
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x202020);

const camera = new THREE.PerspectiveCamera(60, window.innerWidth / window.innerHeight, 0.1, 5000);
camera.position.set(1, 1, 2);

const renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
renderer.setSize(window.innerWidth, window.innerHeight);

// Controls (mouse drag)
const controls = new THREE.OrbitControls(camera, renderer.domElement);

// Lighting
scene.add(new THREE.AmbientLight(0xffffff, 0.6));
const dir = new THREE.DirectionalLight(0xffffff, 1);
dir.position.set(5, 10, 5);
scene.add(dir);

// Load PLY
const loader = new THREE.PLYLoader();
loader.load("/ply/current/map_1/scene_dense_mesh_refine_texture.ply", function (geometry) {
    console.log(geometry.attributes);
    geometry.computeVertexNormals(); // For shading
    const textureLoader = new THREE.TextureLoader();
    textureLoader.load('/ply/current/map_1/scene_dense_mesh_refine_texture.png', (texture) => {
        texture.flipY = false;
        const material = new THREE.MeshStandardMaterial({
            map: texture,
            roughness: 0.8,
            metalness: 0.1
        });

        const mesh = new THREE.Mesh(geometry, material);
        geometry.center();      // optional: center the model
        mesh.scale.set(1, 1, 1); // adjust scale if needed
        scene.add(mesh);
    });
});

// Animation loop
function animate() {
    requestAnimationFrame(animate);
    controls.update();
    renderer.render(scene, camera);
}
animate();

// Resize handling
window.addEventListener("resize", () => {
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
});
