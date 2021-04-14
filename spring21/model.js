function buildSteve() {
  head = buildHead();
  torso = buildTorso();
  torso.add(head);
  head.position.y = HH;
  scene.add(torso);
  torso.position.set(0, HH, 0);

  lLeg = buildLLeg();
  torso.add(lLeg);
  lLeg.position.set(WW / 2, 0, 0);
  rLeg = buildRLeg();
  torso.add(rLeg);
  rLeg.position.set(-WW / 2, 0, 0);
}

function buildHead() {
  let head = new THREE.Group();
  let mesh = new THREE.Mesh(new THREE.BoxGeometry(2 * WW, 2 * WW, 2 * WW), new THREE.MeshNormalMaterial());
  head.add(mesh);
  mesh.position.y = WW;
  return head;
}

function buildLLeg() {
  let lLeg = new THREE.Group();
  let mesh = new THREE.Mesh(new THREE.BoxGeometry(WW, HH, WW), new THREE.MeshNormalMaterial());
  lLeg.add(mesh);
  mesh.position.y = -HH / 2;
  return lLeg;
}

function buildRLeg() {
  let rLeg = new THREE.Group();
  let mesh = new THREE.Mesh(new THREE.BoxGeometry(WW, HH, WW), new THREE.MeshNormalMaterial());
  rLeg.add(mesh);
  mesh.position.y = -HH / 2;
  return rLeg;
}

function buildTorso() {
  let torso = new THREE.Group();
  let mesh = new THREE.Mesh(new THREE.BoxGeometry(2 * WW, HH, WW), new THREE.MeshNormalMaterial());
  torso.add(mesh);
  mesh.position.y = HH / 2;

  return torso;
}
