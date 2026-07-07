import * as THREE from 'three';
import URDFLoader from 'urdf-loader';
import { ColladaLoader } from 'three/addons/loaders/ColladaLoader.js';
import { OBJLoader } from 'three/addons/loaders/OBJLoader.js';
import { MTLLoader } from 'three/addons/loaders/MTLLoader.js';

/* ----------------------------------------------------------------------------
 * Franka Emika Panda arm avatar in the lower-right corner.
 *  - CCD inverse kinematics: the end-effector reaches toward the mouse cursor.
 *  - A billboarded Waddle Dee is "held" at the end-effector.
 *  - Flat, light-free look (ambient/hemisphere only) for a clean, cheap render.
 *  - Auto-fit camera, parked in the lower-right corner.
 *  - Disabled on mobile and when prefers-reduced-motion is set.
 * Arm model: justagist/franka_panda_description (URDF + Collada visual meshes).
 * Adapted for this al-folio site from Hokyun Im's homepage (github.com/jellyho).
 * -------------------------------------------------------------------------- */

// Base URL for model assets. Set by the layout (baseurl-safe); falls back to
// the repo-root path used on user/organization GitHub Pages.
const ASSET_BASE = (typeof window !== 'undefined' && window.ROBOT_ASSETS) || '/assets/models/';

const CONFIG = {
    offsetX: 0.82,    // viewport fraction: + = right  (range ~ -1..1)
    offsetY: -0.40,   // viewport fraction: - = down
    fitPadding: 3.3,  // >1 = smaller arm (more margin)
    basePose: [0.4, -0.5, 0, -2.0, 0, 1.6, 0.78], // panda_joint1..7 seed pose (rad)
    ikChain: ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7'],
    ikIterations: 4,
    ikDamping: 0.4,   // per-iteration step (lower = lazier/smoother)
    targetEase: 0.12, // mouse-target smoothing
};

/* Waddle Dee model: billboarded at the end-effector, always facing the camera. */
const EEF_MODEL = {
    path: `${ASSET_BASE}waddledee/`,
    mtl: 'waddledee.mtl',
    obj: 'waddledee.obj',
    targetSize: 0.17,                 // longest dimension in meters
    faceOffset: [0, 0, 0],            // correction so the FACE points at the camera
};

const URDF_PATH = `${ASSET_BASE}franka/panda_arm.urdf`;

const canvas = document.getElementById('robot-canvas');
const reduceMotion = window.matchMedia('(prefers-reduced-motion: reduce)').matches;
const isMobile = window.matchMedia('(max-width: 768px)').matches;

if (canvas && !reduceMotion && !isMobile) {
    initRobot(canvas).catch(err => console.error('[robot] init failed:', err));
}

/* --- Load the Waddle Dee OBJ/MTL, flatten to unlit, auto-fit, return a group. --- */
async function loadEefModel() {
    const mtl = await new MTLLoader().setPath(EEF_MODEL.path).loadAsync(EEF_MODEL.mtl);
    mtl.preload();
    const obj = await new OBJLoader().setMaterials(mtl).setPath(EEF_MODEL.path).loadAsync(EEF_MODEL.obj);

    // Unlit flat material using the model's texture (clean, light-independent).
    obj.traverse(o => {
        if (!o.isMesh) return;
        const src = Array.isArray(o.material) ? o.material[0] : o.material;
        const map = src && src.map ? src.map : null;
        if (map) map.colorSpace = THREE.SRGBColorSpace;
        o.material = new THREE.MeshBasicMaterial({ map, color: map ? 0xffffff : 0xffb7d5 });
    });

    // Center at origin and scale to a consistent size.
    const box = new THREE.Box3().setFromObject(obj);
    const size = box.getSize(new THREE.Vector3());
    const center = box.getCenter(new THREE.Vector3());
    const s = EEF_MODEL.targetSize / (Math.max(size.x, size.y, size.z) || 1);
    obj.position.sub(center);

    const wrap = new THREE.Group();
    wrap.add(obj);
    wrap.scale.setScalar(s);
    return wrap;
}

async function initRobot(canvas) {
    const scene = new THREE.Scene();

    const renderer = new THREE.WebGLRenderer({ canvas, alpha: true, antialias: true });
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 1)); // low-spec
    renderer.setSize(window.innerWidth, window.innerHeight);
    renderer.outputColorSpace = THREE.SRGBColorSpace;

    const camera = new THREE.PerspectiveCamera(40, window.innerWidth / window.innerHeight, 0.01, 100);

    // Flat, soft lighting only — no directional lights, so no shiny reflections.
    scene.add(new THREE.HemisphereLight(0xffffff, 0xdfe6ef, 1.6));
    scene.add(new THREE.AmbientLight(0xffffff, 0.55));

    const pivot = new THREE.Group();
    scene.add(pivot);

    let robot = null;
    let eef = null;
    let eefModel = null;
    let framed = false;
    const pointer = { x: 0, y: 0 };

    const manager = new THREE.LoadingManager();
    const loader = new URDFLoader(manager);
    loader.parseCollision = false; // visual meshes only (skip collision STLs)
    loader.loadMeshCb = (path, mgr, done) => {
        new ColladaLoader(mgr).load(
            path,
            (collada) => done(collada.scene),
            undefined,
            (err) => { console.error('[robot] mesh failed:', path, err); done(null, err); }
        );
    };
    // Resolves when the URDF and all its meshes have finished loading.
    const allLoaded = new Promise(res => { manager.onLoad = res; });

    console.log('[robot] loading URDF…');
    robot = await new Promise((resolve, reject) => {
        loader.load(URDF_PATH, resolve, undefined, reject);
    });

    robot.rotation.x = -Math.PI / 2; // URDF Z-up -> three.js Y-up
    CONFIG.basePose.forEach((v, i) => {
        const n = `panda_joint${i + 1}`;
        if (robot.joints[n]) robot.setJointValue(n, v);
    });
    pivot.add(robot);

    // Flatten any specular so the matte/clean look holds even if lights change.
    robot.traverse(o => {
        if (o.material) {
            (Array.isArray(o.material) ? o.material : [o.material]).forEach(m => {
                if ('shininess' in m) m.shininess = 0;
                if (m.specular) m.specular.setHex(0x000000);
            });
        }
    });

    // Attach the Waddle Dee model to the end-effector flange.
    const eefLink = robot.links['panda_link8'] || robot.links['panda_link7'];
    try {
        // Added to the scene (not the flange) so its rotation is independent
        // of the arm — it gets billboarded toward the camera every frame.
        eefModel = await loadEefModel();
        scene.add(eefModel);
    } catch (err) {
        console.error('[robot] EEF model failed to load:', err);
    }
    eef = new THREE.Object3D();        // IK end-effector marker (drives model position)
    eef.position.set(0, 0, 0.06);
    eefLink.add(eef);
    console.log('[robot] URDF + EEF model ready. links:', Object.keys(robot.links));

    // ---- Camera framing: fit, then park lower-right (after meshes load). ----
    await allLoaded;
    {
        robot.updateMatrixWorld(true);
        const box = new THREE.Box3().setFromObject(pivot);
        const size = box.getSize(new THREE.Vector3());
        const center = box.getCenter(new THREE.Vector3());
        robot.position.sub(center); // recenter model at the pivot origin

        const maxDim = Math.max(size.x, size.y, size.z) || 1;
        const fov = camera.fov * Math.PI / 180;
        const dist = (maxDim / 2) / Math.tan(fov / 2) * CONFIG.fitPadding;

        camera.position.set(0, 0, dist);
        camera.lookAt(0, 0, 0);

        const vH = 2 * dist * Math.tan(fov / 2);
        const vW = vH * camera.aspect;
        pivot.position.x = (vW / 2) * CONFIG.offsetX;
        pivot.position.y = (vH / 2) * CONFIG.offsetY;

        framed = true;
        console.log('[robot] framed.');
    }

    window.addEventListener('pointermove', (e) => {
        pointer.x = (e.clientX / window.innerWidth) * 2 - 1;
        pointer.y = (e.clientY / window.innerHeight) * 2 - 1;
    });

    window.addEventListener('resize', () => {
        camera.aspect = window.innerWidth / window.innerHeight;
        camera.updateProjectionMatrix();
        renderer.setSize(window.innerWidth, window.innerHeight);
    });

    // ---- CCD IK: drive the EEF toward the mouse target on the z=0 plane. ----
    const target = new THREE.Vector3();
    const smooth = new THREE.Vector3(0, 0, 0);
    const _jp = new THREE.Vector3(), _ee = new THREE.Vector3(), _ax = new THREE.Vector3();
    const _a = new THREE.Vector3(), _b = new THREE.Vector3(), _c = new THREE.Vector3();
    let seeded = false;

    function mouseTarget(out) {
        out.set(pointer.x, -pointer.y, 0.5).unproject(camera);
        out.sub(camera.position).normalize();
        const t = (0 - camera.position.z) / out.z;
        out.multiplyScalar(t).add(camera.position);
    }

    function solveIK() {
        for (let it = 0; it < CONFIG.ikIterations; it++) {
            for (let k = CONFIG.ikChain.length - 1; k >= 0; k--) {
                const joint = robot.joints[CONFIG.ikChain[k]];
                if (!joint) continue;
                robot.updateMatrixWorld(true);
                joint.getWorldPosition(_jp);
                eef.getWorldPosition(_ee);
                _ax.copy(joint.axis).transformDirection(joint.matrixWorld).normalize();

                _a.subVectors(_ee, _jp);          // joint -> end-effector
                _a.addScaledVector(_ax, -_a.dot(_ax)); // project onto joint plane
                _b.subVectors(smooth, _jp);       // joint -> target
                _b.addScaledVector(_ax, -_b.dot(_ax));
                if (_a.lengthSq() < 1e-8 || _b.lengthSq() < 1e-8) continue;
                _a.normalize(); _b.normalize();

                let ang = Math.acos(THREE.MathUtils.clamp(_a.dot(_b), -1, 1));
                if (_c.crossVectors(_a, _b).dot(_ax) < 0) ang = -ang;
                joint.setJointValue(joint.angle + ang * CONFIG.ikDamping); // clamps to limits
            }
        }
    }

    let running = true;
    document.addEventListener('visibilitychange', () => {
        running = !document.hidden;
        if (running) animate();
    });

    function animate() {
        if (!running) return;
        requestAnimationFrame(animate);

        if (robot && framed) {
            mouseTarget(target);
            if (!seeded) { smooth.copy(target); seeded = true; }
            smooth.lerp(target, CONFIG.targetEase);
            solveIK();

            // Billboard Waddle Dee: park at the EEF, but always face the camera.
            if (eefModel) {
                eef.getWorldPosition(_ee);
                eefModel.position.copy(_ee);
                eefModel.lookAt(camera.position);
                const fo = EEF_MODEL.faceOffset;
                eefModel.rotateX(fo[0]);
                eefModel.rotateY(fo[1]);
                eefModel.rotateZ(fo[2]);
            }
        }

        renderer.render(scene, camera);
    }

    animate();
}
