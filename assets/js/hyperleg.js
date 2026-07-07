import * as THREE from 'three';
import URDFLoader from 'urdf-loader';
import { DRACOLoader } from 'three/addons/loaders/DRACOLoader.js';

/* ----------------------------------------------------------------------------
 * HyperLeg bipedal robot that walks along the bottom of the viewport, following
 * the cursor's X position. Procedural gait: far from cursor → run, near → walk.
 * Rendered BEHIND page content, pinned to the bottom edge (fixed).
 * Model: HyperLeg (WIRobotics/IRIM) — extracted from USD to URDF, Draco meshes.
 * -------------------------------------------------------------------------- */
const ASSET_BASE = (typeof window !== 'undefined' && window.HYPERLEG_ASSETS) || '/assets/models/hyperleg/';
const URDF_PATH = `${ASSET_BASE}HyperLeg.urdf`;

const CONFIG = {
    scaleFrac: 0.32,     // robot height as fraction of viewport height
    footMargin: 0.02,    // fraction of viewport height the feet sit above the very bottom
    ease: 0.06,          // horizontal follow smoothing
    kSpeed: 2.2,         // world units/sec of walk speed per world unit of distance
    maxSpeed: 3.2,       // cap (world units/sec)
    arriveDist: 0.05,    // within this world-x distance, stand still
    cadence: 2.6,        // gait phase (rad) advanced per world-unit travelled
    runSpeedNorm: 0.5,   // speedNorm above this blends toward the run gait
    // gait amplitudes (radians): [walk, run]
    hipAmp: [0.35, 0.72],
    kneeAmp: [0.55, 1.15],
    ankleAmp: [0.18, 0.35],
    toeAmp: [0.12, 0.28],
    bob: 0.035,          // vertical bob (fraction of robot height) at 2× cadence
    // per-joint sign so both legs bend the same way (tuned via render)
    sign: { HP: { L: 1, R: 1 }, KN: { L: 1, R: 1 }, AK: { L: 1, R: 1 }, TO: { L: 1, R: 1 } },
};

const canvas = document.getElementById('hyperleg-canvas');
const reduceMotion = window.matchMedia('(prefers-reduced-motion: reduce)').matches;
const isMobile = window.matchMedia('(max-width: 768px)').matches;
if (canvas && !reduceMotion && !isMobile) {
    initWalker(canvas).catch(err => console.error('[hyperleg] init failed:', err));
}

function lerp(a, b, t) { return a + (b - a) * t; }

async function initWalker(canvas) {
    const scene = new THREE.Scene();
    const renderer = new THREE.WebGLRenderer({ canvas, alpha: true, antialias: true });
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 1.5));
    renderer.setSize(window.innerWidth, window.innerHeight);
    renderer.outputColorSpace = THREE.SRGBColorSpace;

    const camera = new THREE.PerspectiveCamera(40, window.innerWidth / window.innerHeight, 0.01, 100);
    camera.position.set(0, 0, 5);
    scene.add(new THREE.HemisphereLight(0xffffff, 0x445566, 2.0));
    scene.add(new THREE.AmbientLight(0xffffff, 0.5));
    const key = new THREE.DirectionalLight(0xffffff, 1.0); key.position.set(0.5, 1.5, 2); scene.add(key);

    const mover = new THREE.Group();   // positioned along the bottom (x follows cursor)
    const facer = new THREE.Group();   // yaw: faces walking direction
    const fit = new THREE.Group();     // base orientation + scale, centered
    mover.add(facer); facer.add(fit); scene.add(mover);

    const manager = new THREE.LoadingManager();
    const loader = new URDFLoader(manager);
    loader.packages = { hyperleg: ASSET_BASE.replace(/\/$/, '') };
    loader.parseCollision = false;
    const draco = new DRACOLoader(manager);
    draco.setDecoderPath('https://cdn.jsdelivr.net/npm/three@0.160.0/examples/jsm/libs/draco/');
    draco.setDecoderConfig({ type: 'wasm' });
    loader.loadMeshCb = (path, mgr, done) => {
        const url = path.replace(/\.stl$/i, '.drc');
        draco.load(url, (geo) => {
            geo.computeVertexNormals();
            done(new THREE.Mesh(geo, new THREE.MeshStandardMaterial({ color: 0xccd0d6, roughness: 0.6, metalness: 0.25 })));
        }, undefined, (e) => { console.error('[hyperleg] mesh failed:', url.split('/').pop()); done(null, e); });
    };
    const allLoaded = new Promise(res => { manager.onLoad = res; });

    console.log('[hyperleg] loading URDF…');
    const robot = await new Promise((res, rej) => loader.load(URDF_PATH, res, undefined, rej));
    robot.rotation.x = -Math.PI / 2; // USD Z-up -> three Y-up (side view, forward = +x)
    fit.add(robot);
    await allLoaded;
    console.log('[hyperleg] loaded. joints:', Object.keys(robot.joints).length);

    const setJ = (n, v) => { if (robot.joints[n]) robot.setJointValue(n, v); };

    // ---- gait: pose the legs for phase t (rad) and speedNorm [0..1] ----
    function poseGait(t, speedNorm) {
        const rn = THREE.MathUtils.clamp(speedNorm, 0, 1);           // walk↔run blend
        const act = THREE.MathUtils.clamp(speedNorm / 0.10, 0, 1);   // 0 at rest → legs straight (idle stand)
        const hipA = lerp(CONFIG.hipAmp[0], CONFIG.hipAmp[1], rn) * act;
        const kneeA = lerp(CONFIG.kneeAmp[0], CONFIG.kneeAmp[1], rn) * act;
        const ankA = lerp(CONFIG.ankleAmp[0], CONFIG.ankleAmp[1], rn) * act;
        const toeA = lerp(CONFIG.toeAmp[0], CONFIG.toeAmp[1], rn) * act;
        const ankOff = 0.05 * act;
        for (const side of ['L', 'R']) {
            const off = side === 'L' ? 0 : Math.PI;
            const p = t + off;
            const s = CONFIG.sign;
            const hip = hipA * Math.sin(p);
            const knee = kneeA * Math.max(0, Math.sin(p + 0.5)); // flex during swing
            const ankle = -ankA * Math.sin(p) + ankOff;
            const toe = toeA * Math.max(0, -Math.sin(p));         // push-off at stance end
            setJ(`${side}_HP`, s.HP[side] * hip);
            setJ(`${side}_KN`, s.KN[side] * knee);
            setJ(`${side}_AK`, s.AK[side] * ankle);
            setJ(`${side}_TO`, s.TO[side] * toe);
            setJ(`${side}_HY`, 0); setJ(`${side}_HR`, 0);
        }
    }

    // viewport world extents on z=0 plane
    function viewport() {
        const fov = camera.fov * Math.PI / 180;
        const vH = 2 * camera.position.z * Math.tan(fov / 2);
        return { vH, vW: vH * camera.aspect };
    }
    // ---- lazy fit: Draco meshes attach asynchronously (after manager.onLoad),
    // so the bounding box is empty for a few frames — fit once it's non-empty. ----
    let fitted = false, robotH = 1, robotScreenH = CONFIG.scaleFrac * viewport().vH;
    function applyScale() {
        const targetH = CONFIG.scaleFrac * viewport().vH;
        fit.scale.setScalar(targetH / robotH);
        robotScreenH = targetH;
    }
    function tryFit() {
        robot.updateMatrixWorld(true);
        const size = new THREE.Box3().setFromObject(robot).getSize(new THREE.Vector3());
        if (size.y < 1e-4) return false;            // geometry not attached yet
        robot.updateMatrixWorld(true);
        const center = new THREE.Box3().setFromObject(robot).getCenter(new THREE.Vector3());
        robot.position.sub(center);
        robotH = size.y;
        applyScale();
        fitted = true;
        window.__allexReady = true; // verification hook (shared with harness driver)
        console.log('[hyperleg] fitted. robotH=' + robotH.toFixed(3) + ' scale=' + fit.scale.x.toFixed(3));
        return true;
    }

    window.addEventListener('resize', () => {
        camera.aspect = window.innerWidth / window.innerHeight;
        camera.updateProjectionMatrix();
        renderer.setSize(window.innerWidth, window.innerHeight);
        if (fitted) applyScale();
    });

    // ---- cursor tracking (x only) ----
    const pointer = { x: 0 };
    window.addEventListener('pointermove', (e) => { pointer.x = (e.clientX / window.innerWidth) * 2 - 1; });
    function cursorWorldX() {
        const { vW } = viewport();
        return pointer.x * (vW / 2);
    }

    let posX = 0, faceDir = 1, phase = 0, last = performance.now(), seeded = false;
    let running = true;
    document.addEventListener('visibilitychange', () => { running = !document.hidden; last = performance.now(); if (running) animate(); });

    function animate() {
        if (!running) return;
        requestAnimationFrame(animate);
        const now = performance.now();
        let dt = (now - last) / 1000; last = now;
        dt = Math.min(dt, 0.05);

        if (!fitted) { tryFit(); renderer.render(scene, camera); return; }

        const tgtX = (typeof window.__hlTargetX === 'number') ? window.__hlTargetX : cursorWorldX();
        if (!seeded) { posX = tgtX; seeded = true; }
        const dx = tgtX - posX;
        const dist = Math.abs(dx);
        let speed = dist > CONFIG.arriveDist ? Math.min(dist * CONFIG.kSpeed, CONFIG.maxSpeed) : 0;
        if (dist > CONFIG.arriveDist) { faceDir = Math.sign(dx); }
        posX += Math.sign(dx) * Math.min(speed * dt, dist);

        // advance gait phase proportional to distance travelled
        phase += speed * CONFIG.cadence * dt;
        const speedNorm = speed / CONFIG.maxSpeed;

        // debug overrides
        const ph = (typeof window.__hlPhase === 'number') ? window.__hlPhase : phase;
        const sn = (typeof window.__hlSpeedNorm === 'number') ? window.__hlSpeedNorm : speedNorm;
        poseGait(ph, sn);

        // place along the bottom, facing walk direction, with a vertical bob
        const { vH } = viewport();
        const groundY = -vH / 2 + CONFIG.footMargin * vH + robotScreenH / 2;
        const bob = CONFIG.bob * robotScreenH * sn * Math.abs(Math.sin(ph));
        mover.position.set((typeof window.__hlPosX === 'number') ? window.__hlPosX : posX, groundY + bob, 0);
        facer.rotation.y = faceDir >= 0 ? 0 : Math.PI;

        renderer.render(scene, camera);
        window.__hlReady = true;
    }
    animate();
}
