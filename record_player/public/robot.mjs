/** Three.js renderer for the repository's URDF-derived COMPAS scene. Created once per team. */
import * as THREE from 'three';
import {OrbitControls} from 'three/addons/controls/OrbitControls.js';

/** Build a rigid transform from COMPAS point/xaxis/yaxis; all translations are metres. */
export function frameMatrix(frame) {
  const matrix = new THREE.Matrix4(); // Identity for missing URDF visual origins.
  if (!frame) return matrix;
  frame = frame.data || frame;
  const x = new THREE.Vector3(...frame.xaxis); // Local X basis.
  const y = new THREE.Vector3(...frame.yaxis); // Local Y basis.
  const z = new THREE.Vector3().crossVectors(x,y); // Right-handed Z basis.
  return matrix.makeBasis(x,y,z).setPosition(...frame.point);
}

/** Build one indexed visual mesh; called for robot links, tool, and static cell bodies. */
function meshFrom(data, color, opacity = 1) {
  const geometry = new THREE.BufferGeometry(); // Mesh coordinates already converted to metres by data.py.
  geometry.setAttribute('position', new THREE.Float32BufferAttribute(data.positions,3));
  geometry.setIndex(data.indices);
  geometry.computeVertexNormals();
  const mesh = new THREE.Mesh(geometry,new THREE.MeshStandardMaterial({color,roughness:.67,metalness:.2,side:THREE.DoubleSide,transparent:opacity<1,opacity})); // Neutral material keeps shape legible.
  if (data.origin) mesh.applyMatrix4(frameMatrix(data.origin));
  if (data.scale) mesh.scale.multiply(new THREE.Vector3(...data.scale));
  return mesh;
}

/** Mount an orbitable robot scene; app.mjs calls update for every synchronized playback frame. */
export function createRobot(container, model, team) {
  const scene = new THREE.Scene(); // Independent camera/scene per team.
  scene.background = new THREE.Color('#0d1622');
  const camera = new THREE.PerspectiveCamera(42,1,.01,100); // Full arena framing.
  camera.up.set(0,0,1);
  const renderer = new THREE.WebGLRenderer({antialias:true}); // GPU renderer attached to this panel.
  renderer.setPixelRatio(Math.min(devicePixelRatio,2));
  container.prepend(renderer.domElement);
  const controls = new OrbitControls(camera,renderer.domElement); // Mouse/touch orbit, pan, zoom.
  controls.enableDamping = true;
  const robotRoot = new THREE.Group(); // Robot base placement from curated scene state.
  robotRoot.applyMatrix4(frameMatrix(model.base));
  scene.add(robotRoot);
  const links = {}; // URDF link-name to local transform node.
  const movable = []; // Joint origin and axis needed for forward kinematics.
  const bodyGroups = {}; // Static geometry toggles keyed by curator names.
  for (const link of model.links) {
    const group = new THREE.Group(); // Local coordinate frame of this link.
    group.name = link.name;
    for (const visual of link.visuals) group.add(meshFrom(visual, link.name.includes('wrist') ? (team==='a' ? '#69cfc5' : '#a598e6') : '#b6c1cd'));
    links[link.name] = group;
  }
  const children = new Set(model.joints.map(joint=>joint.child.link)); // Identify the kinematic root.
  for (const link of model.links) if (!children.has(link.name)) robotRoot.add(links[link.name]);
  for (const joint of model.joints) {
    const group = links[joint.child.link]; // Child transform includes origin followed by joint rotation.
    const origin = frameMatrix(joint.origin); // Fixed URDF transform.
    group.matrixAutoUpdate = false;
    group.matrix.copy(origin);
    links[joint.parent.link].add(group);
    if (joint.type === 'revolute' || joint.type === 'continuous') movable.push({group,origin,axis:new THREE.Vector3(joint.axis.x,joint.axis.y,joint.axis.z).normalize()});
  }
  const tool = new THREE.Group(); // Curated bucket tool attached at manipulator end link tool0.
  tool.applyMatrix4(frameMatrix(model.tool_state.attachment_frame));
  for (const visual of model.tool) tool.add(meshFrom(visual,'#e4b968'));
  links.tool0.add(tool);
  for (const body of model.bodies) {
    const group = new THREE.Group(); // Environment placement including native scale already applied.
    group.applyMatrix4(frameMatrix(body.frame));
    const color = body.name.includes('bucket') ? '#638095' : body.name==='pedestal' ? '#667689' : '#344657'; // Distinguish scoring area and pedestal.
    for (const mesh of body.meshes) group.add(meshFrom(mesh,color,body.hidden ? .15 : .85));
    group.visible = !body.hidden;
    scene.add(group);
    bodyGroups[body.name] = group;
  }
  scene.add(new THREE.HemisphereLight('#d5eaff','#4b5a6c',2.4));
  const light = new THREE.DirectionalLight('#fff7ec',3); // Key light reveals bucket/link shape.
  light.position.set(-2,-3,5); scene.add(light);
  const grid = new THREE.GridHelper(5,20,'#3b5269','#203246'); // Z-up ground reference beneath arena.
  grid.rotation.x=Math.PI/2; grid.position.z=-.86; scene.add(grid);
  const resize = new ResizeObserver(() => { // Panel width changes and responsive layout trigger camera resize.
    const width=container.clientWidth, height=container.clientHeight; // Current drawing dimensions.
    if (!width || !height) return;
    renderer.setSize(width,height,false); camera.aspect=width/height; camera.updateProjectionMatrix();
  });
  resize.observe(container);

  /** Restore a common viewing angle; called initially and by Reset 3D cameras. */
  function reset() { camera.position.set(3.3,-4.1,2.6); controls.target.set(.35,0,-.05); controls.update(); }
  /** Apply recorded joint positions, hide stale robot poses, and redraw while orbiting. */
  function update(angles) {
    robotRoot.visible = !!angles && angles.length===movable.length && angles.every(Number.isFinite);
    if (robotRoot.visible) movable.forEach((joint,index)=> { joint.group.matrix.copy(joint.origin).multiply(new THREE.Matrix4().makeRotationAxis(joint.axis,angles[index])); joint.group.matrixWorldNeedsUpdate=true; });
    controls.update(); renderer.render(scene,camera);
  }
  /** Toggle one curator-named environment body from the settings controls. */
  function visibility(name, visible) { if(bodyGroups[name]) bodyGroups[name].visible=visible; }
  reset();
  return {update,reset,visibility};
}
