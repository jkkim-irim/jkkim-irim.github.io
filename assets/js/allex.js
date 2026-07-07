import * as THREE from 'three';
import URDFLoader from 'urdf-loader';
import { STLLoader } from 'three/addons/loaders/STLLoader.js';

/* ----------------------------------------------------------------------------
 * ALLEX right hand that follows the cursor.
 *  - Middle/Ring/Little fingers curled (~0/90/90), Index + Thumb extended
 *    → a "pointing" hand.
 *  - On mouse click, the index finger flicks (a click/tap gesture).
 *  - Flat, clean lighting; disabled on mobile / prefers-reduced-motion.
 * Model: ALLEX by WIRobotics (URDF trimmed to the R_Wrist_Pitch subtree).
 * -------------------------------------------------------------------------- */
const ASSET_BASE = (typeof window !== 'undefined' && window.ALLEX_ASSETS) || '/assets/models/allex/';
const URDF_PATH = `${ASSET_BASE}ALLEX_Right_Hand.urdf`;

const CONFIG = {
    handScale: 3.2,          // model is in meters (~0.2 m hand); scale up for screen presence
    ease: 0.16,              // cursor-follow smoothing
    // Base orientation of the hand group (radians) — tuned so it faces the
    // viewer with the index finger reading as a pointer. Verified via render.
    euler: [0, Math.PI / 2, 0],           // base facing
    roll: 200 * Math.PI / 180,            // in-screen-plane rotation; aims the index finger up-left
    twist: 30 * Math.PI / 180,            // roll about the index-finger axis (palm toward camera)
    curlMCP: Math.PI / 2,    // 90°
    curlPIP: Math.PI / 2,    // 90°
    curlSign: 1,             // flip if fingers bend the wrong way
    flickMs: 260,            // index flick duration
    flickPeak: 1.1,          // radians the index MCP bends at the peak of a flick
};

const CURLED = ['Middle', 'Ring', 'Little'];

const canvas = document.getElementById('robot-canvas');
const reduceMotion = window.matchMedia('(prefers-reduced-motion: reduce)').matches;
const isMobile = window.matchMedia('(max-width: 768px)').matches;
if (canvas && !reduceMotion && !isMobile) {
    initHand(canvas).catch(err => console.error('[allex] init failed:', err));
}

function colorFor(name) {
    const n = name.toLowerCase();
    if (n.includes('pad'))   return 0x2a2b30;   // rubber finger/palm pads
    if (n.includes('black')) return 0x33343a;
    if (n.includes('white')) return 0xf0f0f2;
    if (n.includes('gray') || n.includes('grey')) return 0x8b929b;
    return 0x9198a0;
}

async function initHand(canvas) {
    const scene = new THREE.Scene();
    const renderer = new THREE.WebGLRenderer({ canvas, alpha: true, antialias: true });
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 1.5));
    renderer.setSize(window.innerWidth, window.innerHeight);
    renderer.outputColorSpace = THREE.SRGBColorSpace;

    const camera = new THREE.PerspectiveCamera(40, window.innerWidth / window.innerHeight, 0.01, 100);
    camera.position.set(0, 0, 5);

    scene.add(new THREE.HemisphereLight(0xffffff, 0x40454d, 2.2));
    scene.add(new THREE.AmbientLight(0xffffff, 0.5));
    const key = new THREE.DirectionalLight(0xffffff, 1.1); key.position.set(1, 1.5, 2); scene.add(key);

    // group we move to the cursor; inner group holds the model centered + oriented
    const follow = new THREE.Group();       // positioned at cursor
    const orient = new THREE.Group();        // base orientation + scale
    follow.add(orient);
    scene.add(follow);

    const manager = new THREE.LoadingManager();
    const loader = new URDFLoader(manager);
    loader.packages = { allex_description: ASSET_BASE.replace(/\/$/, '') };
    loader.parseCollision = false;
    const stl = new STLLoader(manager);
    loader.loadMeshCb = (path, mgr, done) => {
        stl.load(path, (geo) => {
            geo.computeVertexNormals();
            const mat = new THREE.MeshStandardMaterial({ color: colorFor(path.split('/').pop()), roughness: 0.6, metalness: 0.15 });
            done(new THREE.Mesh(geo, mat));
        }, undefined, (e) => { console.error('[allex] mesh failed:', path.split('/').pop()); done(null, e); });
    };
    const allLoaded = new Promise(res => { manager.onLoad = res; });

    console.log('[allex] loading hand URDF…');
    const hand = await new Promise((res, rej) => loader.load(URDF_PATH, res, undefined, rej));
    hand.rotation.x = -Math.PI / 2; // URDF Z-up -> three Y-up
    const twist = new THREE.Group();  // rotates the hand about its own index-finger axis
    orient.add(twist);
    twist.add(hand);
    // NOTE: orient rotation/scale are applied AFTER recentring (below) so the
    // centroid is computed in the hand's own frame.

    await allLoaded;
    console.log('[allex] meshes loaded. joints:', Object.keys(hand.joints).length);

    // ---- pose the fingers ----
    function setJ(name, val) { if (hand.joints[name]) hand.setJointValue(name, val); }
    function poseFingers(indexMCP) {
        const sign = (typeof window.__allexCurlSign === 'number') ? window.__allexCurlSign : CONFIG.curlSign;
        for (const f of CURLED) {
            setJ(`R_${f}_ABAD_Joint`, 0);
            setJ(`R_${f}_MCP_Joint`, sign * CONFIG.curlMCP);
            setJ(`R_${f}_PIP_Joint`, sign * CONFIG.curlPIP);
            // DIP is coupled (mimic) but set explicitly in case loader ignores mimic
            setJ(`R_${f}_DIP_Joint`, sign * CONFIG.curlPIP * 0.656296489);
        }
        // Index: extended, except the transient flick on MCP
        setJ('R_Index_ABAD_Joint', 0);
        setJ('R_Index_MCP_Joint', sign * indexMCP);
        setJ('R_Index_PIP_Joint', sign * indexMCP * 0.6);
        setJ('R_Index_DIP_Joint', sign * indexMCP * 0.4);
        // Thumb: extended
        setJ('R_Thumb_Yaw_Joint', 0);
        setJ('R_Thumb_CMC_Joint', 0);
        setJ('R_Thumb_MCP_Joint', 0);
        setJ('R_Thumb_IP_Joint', 0);
    }
    poseFingers(0);

    // ---- Anchor the index FINGERTIP at the cursor (orient origin) ----
    // Index-finger axis in twist-local frame (orient/twist still identity here),
    // so twisting rolls the palm toward the camera without moving the fingertip
    // (the fingertip lies on this axis) or changing where the index points.
    hand.updateMatrixWorld(true);
    const tipLink  = hand.links['R_Index_Fingertip'] || hand.links['R_Index_Distal_Link'] || hand;
    const baseLink = hand.links['R_Index_Proximal_Link'] || hand;
    const _base = new THREE.Vector3(), _tip = new THREE.Vector3();
    baseLink.getWorldPosition(_base);
    tipLink.getWorldPosition(_tip);
    const indexAxis = _tip.clone().sub(_base).normalize();

    // twist about the index axis, then offset so the fingertip sits at the origin
    twist.setRotationFromAxisAngle(indexAxis, CONFIG.twist);
    twist.updateMatrixWorld(true);
    tipLink.getWorldPosition(_tip);
    twist.position.sub(_tip);

    // presentation orientation + scale (about the fingertip = cursor point)
    orient.rotation.set(CONFIG.euler[0], CONFIG.euler[1], CONFIG.euler[2]);
    orient.scale.setScalar(CONFIG.handScale);
    window.__allexReady = true; // verification hook

    // ---- cursor tracking ----
    const pointer = { x: 0, y: 0 };
    window.addEventListener('pointermove', (e) => {
        pointer.x = (e.clientX / window.innerWidth) * 2 - 1;
        pointer.y = (e.clientY / window.innerHeight) * 2 - 1;
    });
    window.addEventListener('resize', () => {
        camera.aspect = window.innerWidth / window.innerHeight;
        camera.updateProjectionMatrix();
        renderer.setSize(window.innerWidth, window.innerHeight);
    });

    // ---- index flick on click ----
    let flickStart = -1e9;
    window.addEventListener('pointerdown', () => { flickStart = performance.now(); });
    // allow harness to trigger a flick
    window.__allexFlick = () => { flickStart = performance.now(); };

    const target = new THREE.Vector3();
    const smooth = new THREE.Vector3();
    let seeded = false;
    function mouseWorld(out) {
        out.set(pointer.x, -pointer.y, 0.5).unproject(camera);
        out.sub(camera.position).normalize();
        const t = (0 - camera.position.z) / out.z;
        out.multiplyScalar(t).add(camera.position);
    }

    let running = true;
    document.addEventListener('visibilitychange', () => { running = !document.hidden; if (running) animate(); });

    function animate() {
        if (!running) return;
        requestAnimationFrame(animate);
        mouseWorld(target);
        if (!seeded) { smooth.copy(target); seeded = true; }
        smooth.lerp(target, CONFIG.ease);
        follow.position.copy(smooth);
        follow.rotation.z = (typeof window.__allexRoll === 'number') ? window.__allexRoll : CONFIG.roll;
        const tw = (typeof window.__allexTwist === 'number') ? window.__allexTwist : CONFIG.twist;
        twist.setRotationFromAxisAngle(indexAxis, tw);
        if (Array.isArray(window.__allexEuler)) orient.rotation.set(...window.__allexEuler); // debug

        // flick curve: 0 -> peak -> 0
        const dt = performance.now() - flickStart;
        let idx = 0;
        if (dt >= 0 && dt < CONFIG.flickMs) {
            const p = dt / CONFIG.flickMs;
            idx = Math.sin(p * Math.PI) * CONFIG.flickPeak; // smooth up-and-back
        }
        if (typeof window.__allexHoldIndex === 'number') idx = window.__allexHoldIndex; // debug/verify
        poseFingers(idx);

        renderer.render(scene, camera);
    }
    animate();
}
