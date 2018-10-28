/////////////////////////////////
/// HELPER FUNCTIONS
// p: the vector to be projected
// n: the normal defining the projection plane (unit vector)
// clarification: call by reference/pointer or call-by-value
function proj2plane(p, n) {
	return p.clone().projectOnPlane(n);
}

function CLAMP(x, xLo, xHi) {
	return x < xLo ? xLo : ( x > xHi ? xHi : x) ;
}


class CCDSys {
	constructor ( fkFunc ) {
		this.axes = [];
		this.fkFunc = fkFunc
	}
	
  	setCCDAxis (vec, id, angleLo, angleHi) {
  	  let CCD_axis = {axis: vec.clone(), jointid: id};
  	  let thetaLo = angleLo || -1e4 // default: no limits
 	  let thetaHi = angleHi || 1e4 
 	  CCD_axis.limits = new THREE.Vector2 (thetaLo, thetaHi)
 	  
 	  this.axes.push (CCD_axis)
  	}
  
	solve ( target, thetas ) { // (NY) in case base is changing ...
		
		// local variable for iterations
		let end = new THREE.Vector3();
    	let base = new THREE.Vector3();

		// short hand 
		let axes = this.axes;
    
    	// e.g., njoints = 2;
    	// jointid: 0,0,1
		var njoints = axes[axes.length - 1].jointid + 1;
		var joints = [];
		for (var i = 0; i <= njoints; i++) joints[i] = new THREE.Vector3();

		this.fkFunc (thetas, joints);
		end.copy(joints[joints.length - 1]);

		// convergence
		const EPS = 1e-1;
		const MAXITER = 20;

		let t_target = new THREE.Vector3();
		let t_end = new THREE.Vector3();
		let tmpV = new THREE.Vector3();

		// iterations

		for (var iter = 0; iter < MAXITER; iter++) {
		  for (var i = axes.length - 1; i >= 0; i--) {
			base.copy(joints[axes[i].jointid]);

			// this part is quite different from the C counterpart
			var axis = axes[i].axis.clone();
			for (var j = i - 1; j >= 0; j--)
			  axis.applyMatrix4(new THREE.Matrix4().makeRotationAxis(axes[j].axis, thetas[j]));

			// after this manipulation,
			// axis become world coordinate

			tmpV.subVectors(target, base);
			tmpV = proj2plane(tmpV, axis);
			t_target.copy(tmpV.normalize());

			tmpV.subVectors(end, base);
			tmpV = proj2plane(tmpV, axis);
			t_end.copy(tmpV.normalize());

			var dotV = t_end.dot(t_target);
			var angle = Math.acos(CLAMP(dotV, -1, 1));
			tmpV.crossVectors(t_end, t_target);
			var sign = (tmpV.dot(axis) > 0) ? 1 : -1;
			thetas[i] += sign * angle;

			// joint limit [-2.4, -0.1]
			thetas[i] = CLAMP(thetas[i], axes[i].limits.x, axes[i].limits.y)

			this.fkFunc (thetas, joints);
			end.copy(joints[joints.length - 1]);

			if (end.distanceTo(target) < EPS) {
			  return 1;
			}
	 	   }
		}


		if (iter < MAXITER)
		  return 1;
		else {
		  console.log("do not converge");   // does not mean "fail"
		                                    // sometimes just mean "target not reachable"
		  return 0;
		}

	} // end of solve


}

//////////////////////////////////////////////////////////////////////////
/*
var scene, renderer, camera;
var link1, link2, theta1, theta2;

var target = new THREE.Vector3();
var xx = 0; sign = 1;

var ccdSys;

init();
animate();

////////////////////////////////////////////////////////
// forward kinematics
function fk (theta, joints) {  // all memory have been allocated
  joints[0].set (0,0,0);
  
  var localzero = new THREE.Vector3(0, 0, 0);
  var m = new THREE.Matrix4();
  m.makeRotationY(theta[0]);
  m.multiply(new THREE.Matrix4().makeTranslation(60, 0, 0));
  localzero.applyMatrix4(m);
  joints[1].copy(localzero);

  localzero.set(0, 0, 0);
  m.multiply(new THREE.Matrix4().makeRotationY(theta[1]));
  m.multiply(new THREE.Matrix4().makeTranslation(90, 0, 0));
  localzero.applyMatrix4(m);
  joints[2].copy(localzero);
}


////////////////////////////////////////////////////////////////
function init() {

  renderer = new THREE.WebGLRenderer({
    antialias: true
  });
  renderer.setSize(window.innerWidth, window.innerHeight);
  renderer.setClearColor(0x888888);

	scene = new THREE.Scene();
  camera = new THREE.PerspectiveCamera(45, window.innerWidth / window.innerHeight, 1, 10000);
  camera.position.y = 160;
  camera.position.z = 400;
  camera.lookAt(new THREE.Vector3(0, 0, 0));
  document.body.appendChild(renderer.domElement);
  let controls = new THREE.OrbitControls(camera, renderer.domElement);

  var gridXZ = new THREE.GridHelper(200, 20, 'red', 'white');
  scene.add(gridXZ);
  window.addEventListener('resize', onWindowResize, false);

  //////////////////////////////////////////////////
	link1 = makeLink(60);
  scene.add (link1);
	link2 = makeLink(90);
  link1.add (link2);
  link2.position.set (60,0,0);

	theta1 = 0;
  theta2 = 0;
  // base
  var cyl_geom = new THREE.CylinderGeometry(10, 10, 6, 32);
  var cyl_mat = new THREE.MeshBasicMaterial({
    color: 0xff2211
  });
  var base = new THREE.Mesh(cyl_geom, cyl_mat);
  scene.add(base);

  //////////////////////////////////////////////////
  // setting ccdSys
  ccdSys = new CCDSys (fk)
  ccdSys.setCCDAxis (new THREE.Vector3(0,1,0), 0)
  ccdSys.setCCDAxis (new THREE.Vector3(0,1,0), 1, -3.1, -0.01)
}

function makeLink(length) {
  var oneLink = new THREE.Object3D();
  var mesh = new THREE.Mesh(new THREE.BoxGeometry(length, 10,10), new THREE.MeshNormalMaterial());
  oneLink.add(mesh);
  mesh.position.set(length/2, 0, 0);
  return oneLink;
}

function onWindowResize() {
  let width = window.innerWidth;
  let height = window.innerHeight;
  camera.aspect = width / height;
  camera.updateProjectionMatrix();
  renderer.setSize(width, height);
}

function animate() {
  
  requestAnimationFrame(animate);
  update()
  render();
}

function update() {
    
  // ccdSys works on the theta array
  var thetas = [theta1, theta2]; 

  if (Math.abs(xx) > 100) 
  	sign *= -1;
  xx += sign * 5;
  target.set (xx,0,-100);

  ccdSys.solve (target, thetas);
  
  // copy the theta array back to theta1 & theta2  
  theta1 = thetas[0], theta2 = thetas[1];

}


function render() {

	link1.rotation.y = theta1;
  link2.rotation.y = theta2;

  renderer.render(scene, camera);
}
*/
