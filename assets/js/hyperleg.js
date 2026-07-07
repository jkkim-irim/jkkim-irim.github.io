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
    scaleFrac: 0.211,    // robot height as fraction of viewport height (~10% larger than 0.192)
    footMargin: 0.04,    // fraction of viewport height the feet sit above the very bottom (just above the footer)
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
    // ---- claw-machine grab: click the robot → it dangles from the cursor, release → falls ----
    grabHangFrac: 0.55,  // pivot-to-centre distance as a fraction of robot height
    pendGravity: 15,     // pendulum restoring toward hanging-down (lower = wider, slower swing)
    pendDamp: 0.3,       // angular damping (per second) — very low so it keeps dangling for a while
    pendDrive: 2.0,      // how much cursor motion swings it
    fallGravity: 9.81,   // world units/s² when dropped
    fallBounce: 0.28,    // bounce factor on landing
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

// Per-link colours (mesh files are named by link, e.g. "l_hp.drc").
const BLACK = 0x1c1c20, DARKGRAY = 0x3a3a40, RED = 0xd23a2e, DEFAULT = 0xccd0d6;
const LINK_COLOR = {
    hp: BLACK,      // thigh (upper leg)
    hr: DARKGRAY,   // hip-roll link
    ft: BLACK,      // foot link
    to: DARKGRAY,   // toe link
    heel: RED,      // heel
    tp: RED,        // toe tip
};
function colorFor(fname) {
    const base = fname.toLowerCase().replace(/\.(drc|stl)$/, '').replace(/^[lr]_/, '');
    return LINK_COLOR[base] ?? DEFAULT; // torso / hy / kn / ak keep the default light grey
}

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

    const mover = new THREE.Group();   // world position of the grab pivot / body
    const swing = new THREE.Group();   // pendulum rotation while grabbed
    const facer = new THREE.Group();   // yaw: faces walking direction
    const fit = new THREE.Group();     // base orientation + scale, centered
    mover.add(swing); swing.add(facer); facer.add(fit); scene.add(mover);

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
            done(new THREE.Mesh(geo, new THREE.MeshStandardMaterial({ color: colorFor(url.split('/').pop()), roughness: 0.6, metalness: 0.25 })));
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
        // Recolor per link now that the (async Draco) meshes are attached — overrides
        // urdf-loader's default material so the requested per-link colours win.
        for (const ln in robot.links) {
            const col = colorFor(ln);
            robot.links[ln].traverse(o => { if (o.isMesh && o.material) o.material.color.setHex(col); });
        }
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

    // ---- cursor tracking (x and y, NDC) ----
    const pointer = { x: 0, y: 0 };
    window.addEventListener('pointermove', (e) => {
        pointer.x = (e.clientX / window.innerWidth) * 2 - 1;
        pointer.y = -((e.clientY / window.innerHeight) * 2 - 1);
    });
    function cursorWorldX() { return pointer.x * (viewport().vW / 2); }
    const _cw = new THREE.Vector3();
    function cursorWorld(out) { // unproject cursor onto the z=0 plane
        out.set(pointer.x, pointer.y, 0.5).unproject(camera);
        out.sub(camera.position).normalize();
        out.multiplyScalar((0 - camera.position.z) / out.z).add(camera.position);
    }

    // limp "grabbed" leg pose (gentle dangle)
    function poseHang() {
        for (const side of ['L', 'R']) {
            const s = CONFIG.sign;
            setJ(`${side}_HP`, s.HP[side] * 0.12);
            setJ(`${side}_KN`, s.KN[side] * 0.45);
            setJ(`${side}_AK`, s.AK[side] * 0.10);
            setJ(`${side}_TO`, 0); setJ(`${side}_HY`, 0); setJ(`${side}_HR`, 0);
        }
    }

    // ---- grab / release (canvas is pointer-events:none, so listen on window + raycast) ----
    const raycaster = new THREE.Raycaster();
    let state = 'walk';                 // 'walk' | 'held' | 'fall'
    let theta = 0, omega = 0;           // pendulum angle + angular velocity
    let grabX = 0, fallVY = 0, prevCurX = 0, hangL = 0;
    function hitsRobot() {
        raycaster.setFromCamera({ x: pointer.x, y: pointer.y }, camera);
        return raycaster.intersectObject(mover, true).length > 0;
    }
    function setNoSelect(on) {                 // stop the page text from being drag-selected while grabbing
        document.body.style.userSelect = on ? 'none' : '';
        document.body.style.webkitUserSelect = on ? 'none' : '';
    }
    window.addEventListener('pointerdown', (e) => {
        if (!fitted || state !== 'walk') return;
        if (hitsRobot()) {
            state = 'held'; theta = 0; omega = 0; cursorWorld(_cw); prevCurX = _cw.x;
            setNoSelect(true);
            const sel = window.getSelection && window.getSelection(); if (sel) sel.removeAllRanges();
            e.preventDefault();
        }
    }, { passive: false });
    document.addEventListener('selectstart', (e) => { if (state === 'held') e.preventDefault(); });
    function release() {
        if (state !== 'held') return;
        state = 'fall'; fallVY = 0;      // start dropping from the current dangling height
        setNoSelect(false);
    }
    window.addEventListener('pointerup', release);
    window.addEventListener('pointercancel', release);
    window.addEventListener('blur', release);
    window.__hlGrab = () => { if (fitted) { state = 'held'; theta = 0; omega = 0; } }; // harness hook
    window.__hlRelease = release;

    let posX = 0, faceDir = 1, phase = 0, last = performance.now(), seeded = false;
    // Only reset the dt baseline when returning to the tab. Do NOT restart animate()
    // here — the single rAF loop resumes on its own; restarting stacks duplicate loops
    // (which produced dt≈0 frames → divide-by-zero → NaN position → robot vanished).
    document.addEventListener('visibilitychange', () => { if (!document.hidden) last = performance.now(); });

    function animate() {
        requestAnimationFrame(animate);
        const now = performance.now();
        let dt = (now - last) / 1000; last = now;
        if (!(dt > 0)) dt = 1 / 60;        // guard dt<=0/NaN so nothing divides by zero
        dt = Math.min(dt, 0.05);

        if (!fitted) { tryFit(); renderer.render(scene, camera); return; }

        const { vH } = viewport();
        const centerY = -vH / 2 + CONFIG.footMargin * vH + robotScreenH / 2; // body-centre Y when standing
        hangL = CONFIG.grabHangFrac * robotScreenH;                          // pivot → body-centre distance

        if (state === 'held') {
            // dangle from the cursor: driven, damped pendulum
            cursorWorld(_cw);
            const curVX = THREE.MathUtils.clamp((_cw.x - prevCurX) / dt, -6, 6); prevCurX = _cw.x;
            omega += (-CONFIG.pendGravity * Math.sin(theta) - CONFIG.pendDrive * curVX * Math.cos(theta)) * dt;
            omega -= omega * Math.min(CONFIG.pendDamp * dt, 1);
            omega = THREE.MathUtils.clamp(omega, -9, 9);
            theta = THREE.MathUtils.clamp(theta + omega * dt, -1.4, 1.4);
            if (Math.abs(theta) >= 1.4) omega = 0; // don't build up against the limit
            if (!isFinite(theta) || !isFinite(omega)) { theta = 0; omega = 0; } // safety: never let NaN persist
            poseHang();
            mover.position.set(_cw.x, _cw.y, 0);
            swing.rotation.z = theta;
            facer.position.y = -hangL;
            grabX = _cw.x;
        } else if (state === 'fall') {
            // drop under gravity back to the ground, then resume walking
            fallVY -= CONFIG.fallGravity * dt;
            let py = mover.position.y + fallVY * dt;
            const landY = centerY + hangL;
            poseHang();
            theta += (0 - theta) * Math.min(8 * dt, 1); swing.rotation.z = theta;
            facer.position.y = -hangL;
            if (py <= landY) {
                if (Math.abs(fallVY) > 0.6) { py = landY; fallVY = -fallVY * CONFIG.fallBounce; }
                else { py = landY; state = 'walk'; posX = grabX; theta = 0; omega = 0; }
            }
            mover.position.set(grabX, py, 0);
        } else { // 'walk'
            const tgtX = (typeof window.__hlTargetX === 'number') ? window.__hlTargetX : cursorWorldX();
            if (!seeded) { posX = tgtX; seeded = true; }
            const dx = tgtX - posX;
            const dist = Math.abs(dx);
            const speed = dist > CONFIG.arriveDist ? Math.min(dist * CONFIG.kSpeed, CONFIG.maxSpeed) : 0;
            if (dist > CONFIG.arriveDist) faceDir = Math.sign(dx);
            posX += Math.sign(dx) * Math.min(speed * dt, dist);
            phase += speed * CONFIG.cadence * dt;
            const speedNorm = speed / CONFIG.maxSpeed;
            const ph = (typeof window.__hlPhase === 'number') ? window.__hlPhase : phase;
            const sn = (typeof window.__hlSpeedNorm === 'number') ? window.__hlSpeedNorm : speedNorm;
            poseGait(ph, sn);
            const bob = CONFIG.bob * robotScreenH * sn * Math.abs(Math.sin(ph));
            mover.position.set((typeof window.__hlPosX === 'number') ? window.__hlPosX : posX, centerY + bob, 0);
            swing.rotation.z = 0;
            facer.position.y = 0;
        }
        facer.rotation.y = faceDir >= 0 ? 0 : Math.PI;
        renderer.render(scene, camera);
        window.__hlReady = true;
    }
    animate();
}
