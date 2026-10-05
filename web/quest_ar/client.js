import * as THREE from 'three';
import {GLTFLoader} from 'three/addons/loaders/GLTFLoader.js';
import {ButtonEdges, readController, worldMatrix, floorPoint, distanceInRobot, startGateText, graspTip, axisErrorsInRobot} from './core.mjs';

const enter = document.querySelector('#enter'), status = document.querySelector('#status');
const renderer = new THREE.WebGLRenderer({antialias: true, alpha: true});
renderer.setPixelRatio(Math.min(window.devicePixelRatio, 1.5));
renderer.setSize(window.innerWidth, window.innerHeight);
renderer.setClearColor(0x000000, 0);
renderer.xr.enabled = true;
renderer.xr.setReferenceSpaceType('local-floor');
renderer.xr.setFoveation(1);
document.body.appendChild(renderer.domElement);
const scene = new THREE.Scene(); // no background or Isaac ground: keep passthrough transparent
scene.add(new THREE.HemisphereLight(0xffffff, 0x566174, 2.5));
const light = new THREE.DirectionalLight(0xffffff, 2.2); light.position.set(1, 3, 2); scene.add(light);
const camera = new THREE.PerspectiveCamera(55, window.innerWidth/window.innerHeight, .01, 50);
camera.position.set(1.6, 1.3, 1.8); camera.lookAt(0, .5, 0);
const world = new THREE.Group(); world.matrixAutoUpdate = false; scene.add(world);
const links = new Map(), cubes = [], parts = [];
let ws = null, manifest = null, modelReady = false, session = null, latest = null, ui = {};
let placed = false, space = 0, sequence = 0, placementSequence = Infinity;
let origin = [0, 0, 0], yaw = Math.PI, shift = [0, 0], anchor = null, anchorGeneration = 0;
let lastInput = -Infinity, lastFrame = 0, lastHUD = 0, sceneReceived = 0, geometryKey = '';
let lastStartKey = '', lastStartReceived = -Infinity;
const edges = new ButtonEdges(), inverse = new THREE.Matrix4();
const spheres = ['left', 'right'].map(() => {
  const mesh = new THREE.Mesh(new THREE.SphereGeometry(.05, 24, 16),
    new THREE.MeshBasicMaterial({color: 0xffba55, wireframe: true, transparent: true, opacity: .65,
                                 depthTest:false, depthWrite:false}));
  mesh.renderOrder=20;
  world.add(mesh); return mesh;
});
const handMarkers = ['left', 'right'].map(() => {
  const mesh = new THREE.Mesh(new THREE.SphereGeometry(.012, 12, 8),
    new THREE.MeshBasicMaterial({color: 0x53c9f3, depthTest:false, depthWrite:false}));
  mesh.renderOrder=30;
  mesh.visible = false; scene.add(mesh); return mesh;
});
function alignmentAxes(parent, axes=[0,1], names=['B','A']) {
  const group=new THREE.Group(); group.visible=false; parent.add(group);
  for (const [index,axis] of axes.entries()) {
    const end=new THREE.Vector3(); end.setComponent(axis,.05);
    const geometry=new THREE.BufferGeometry().setFromPoints([end.clone().negate(),end]);
    const line=new THREE.Line(geometry,new THREE.LineDashedMaterial({color:0xffffff,
      transparent:true,opacity:.6,dashSize:.004,gapSize:.003,depthTest:false,depthWrite:false}));
    line.computeLineDistances(); line.renderOrder=40; group.add(line);
    const canvas=document.createElement('canvas'); canvas.width=64; canvas.height=64;
    const ctx=canvas.getContext('2d'); ctx.font='bold 48px sans-serif';ctx.fillStyle='rgba(255,255,255,.7)';
    ctx.textAlign='center';ctx.fillText(names[index],32,48);
    const label=new THREE.Sprite(new THREE.SpriteMaterial({map:new THREE.CanvasTexture(canvas),
      transparent:true,depthTest:false,depthWrite:false}));
    label.position.copy(end).multiplyScalar(1.2); label.scale.set(.012,.012,1);label.renderOrder=41;group.add(label);
  }
  return group;
}
const controllerAxes=[alignmentAxes(scene),alignmentAxes(scene)];
controllerAxes.forEach(a=>a.matrixAutoUpdate=false);
const eeAxes=[alignmentAxes(world,[0,2],['A','B']),alignmentAxes(world,[0,2],['A','B'])];
const originMarker = new THREE.AxesHelper(.25); world.add(originMarker);
const hudCanvas = document.createElement('canvas'); hudCanvas.width=1024; hudCanvas.height=430;
const hudTexture = new THREE.CanvasTexture(hudCanvas);
const hud = new THREE.Sprite(new THREE.SpriteMaterial({map:hudTexture, depthTest:false, transparent:true}));
hud.scale.set(.76, .32, 1); hud.visible=false; hud.renderOrder=100; scene.add(hud);

function send(value) {
  if (ws?.readyState === WebSocket.OPEN && ws.bufferedAmount < 32768) { ws.send(JSON.stringify(value)); return true; }
  return false;
}

function clearPlacement(reason, notify=true) {
  placed=false; placementSequence=Infinity; space++; edges.reset();
  anchorGeneration++; anchor?.delete(); anchor=null;
  if (notify) send({kind:'reset', reason});
  status.textContent=reason+'\nA로 공간을 다시 고정하세요.';
}

function setWorld(position, angle) {
  world.matrix.fromArray(worldMatrix(position, angle)); world.matrixWorldNeedsUpdate=true;
  world.updateMatrixWorld(true); inverse.copy(world.matrix).invert();
}

function setPose(object, pose) {
  object.position.set(pose[0], pose[1], pose[2]);
  object.quaternion.set(pose[4], pose[5], pose[6], pose[3]); // Isaac wxyz -> Three xyzw
}

function cuboid(spec) {
  const mesh = new THREE.Mesh(new THREE.BoxGeometry(...spec.size),
    new THREE.MeshStandardMaterial({color:new THREE.Color(...spec.color), roughness:.65}));
  mesh.position.fromArray(spec.position); world.add(mesh); return mesh;
}

function applyScene(value) {
  if (!value) return;
  latest=value;
  const key=JSON.stringify(value.geometry);
  if (geometryKey !== key) {
    for (const mesh of [...parts, ...cubes]) { world.remove(mesh); mesh.geometry.dispose(); mesh.material.dispose(); }
    parts.length=0; cubes.length=0;
    parts.push(cuboid(value.geometry.table));
    for (const part of value.geometry.box) parts.push(cuboid(part));
    geometryKey=key;
  }
  // Display measured rigid-link poses. No commanded FK or collision physics
  // are run here. Spheres use the same measured link origins as server gating.
  for (const [name, pose] of Object.entries(value.links)) {
    const mesh=links.get(name); if (mesh) setPose(mesh, pose);
  }
  for (let i=0; i<value.cubes.length; i++) {
    if (!cubes[i]) cubes[i]=cuboid({size:Array(3).fill(value.geometry.cube_size),
                                  color:value.geometry.cube_color, position:[0,0,0]});
    setPose(cubes[i], value.cubes[i]);
  }
  while (cubes.length>value.cubes.length) {
    const mesh=cubes.pop(); world.remove(mesh); mesh.geometry.dispose(); mesh.material.dispose();
  }
  if (manifest?.tcp_offset)
    ['left','right'].forEach((side,i)=> {
      const pose=value.links[side+'_dof7_link']; if (!pose) return;
      const center=graspTip(pose,manifest.tcp_offset);
      spheres[i].position.fromArray(center); spheres[i].scale.setScalar((manifest.attach_radius??.04)/.05);
      eeAxes[i].position.fromArray(center); eeAxes[i].quaternion.set(pose[4],pose[5],pose[6],pose[3]);
      eeAxes[i].visible=Boolean(session);
    });
}

async function loadModel(value) {
  if (modelReady && manifest.model_id !== value.model_id) {
    modelReady=false;
    world.visible=false;
    clearPlacement('표시 모델이 변경되었습니다. 페이지를 새로 고침하세요.');
    throw new Error('표시 모델이 변경되었습니다. 페이지를 새로 고침하세요.');
  }
  manifest=value;
  if (modelReady) return;
  status.textContent='URDF 원본 형상을 불러오는 중…';
  const gltf=await new GLTFLoader().loadAsync(value.robot_url,progress=> {
    const size=(progress.loaded/1e6).toFixed(1);
    status.textContent=`URDF 원본 형상 로딩: ${size} MB`+
      (value.glb_bytes ? ` / ${(value.glb_bytes/1e6).toFixed(1)} MB` : '');
  });
  // A link can have multiple visuals (the base has the pillar and upper body).
  // Move the whole link group in its measured Isaac frame, keeping all children.
  links.clear();
  for (const name of value.links) {
    const object=gltf.scene.getObjectByName(name);
    if (!object) throw new Error(`GLB에 Isaac 링크가 없습니다: ${name}`);
    links.set(name,object);
  }
  for (const visual of value.meshes) {
    const object=gltf.scene.getObjectByName(visual.name);
    if (!object?.isMesh || object.geometry.index.count!==visual.faces*3)
      throw new Error(`URDF visual 형상이 불완전합니다: ${visual.name}`);
  }
  world.add(gltf.scene); modelReady=true;
  status.dataset.visualCount=String(value.visual_count);
  status.dataset.triangleCount=String(value.triangle_count);
  if (latest) applyScene(latest);
  const supported=await navigator.xr?.isSessionSupported('immersive-ar');
  if (!supported) { status.textContent='Quest Browser에서 열어주세요. immersive-ar 지원이 필요합니다.'; return; }
  enter.disabled=false; enter.textContent='AR 시작';
  status.textContent='준비 완료. AR 시작 후 오른손으로 바닥을 가리키고 A를 누르세요.';
}

function connect() {
  ws=new WebSocket(`wss://${location.host}/ws`);
  ws.onmessage=event=> {
    const value=JSON.parse(event.data);
    if (value.kind==='hello') loadModel(value.manifest).catch(error=> {status.textContent=error.message; enter.disabled=true;});
    if (value.kind==='state') {
      sceneReceived=performance.now();
      ui=value.ui; ui.scene_age=value.scene_age;
      const key=ui.last_start ? `${ui.last_start.time}:${ui.last_start.input_sequence}` : '';
      if (key !== lastStartKey) {lastStartKey=key; lastStartReceived=performance.now();}
      if (value.scene && (!latest || latest.sequence!==value.scene.sequence || latest.boot_time!==value.scene.boot_time)) applyScene(value.scene);
      if (placed && ui.input_sequence>=placementSequence &&
          (ui.minimum_space>space || (ui.space===space && !ui.placed)))
        clearPlacement('공간 앵커가 변경되었습니다.');
    }
  };
  ws.onclose=()=> {
    clearPlacement('서버 연결이 끊겼습니다.',false);
    status.textContent='서버 연결을 다시 시도합니다. 로봇은 HOLD합니다.';
    setTimeout(connect,2000);
  };
  ws.onerror=()=> {status.textContent='HTTPS/WSS 연결을 확인하세요.';};
}

enter.onclick=async()=> {
  enter.disabled=true;
  try {
    session=await navigator.xr.requestSession('immersive-ar', {requiredFeatures:['local-floor'], optionalFeatures:['anchors']});
    session.addEventListener('end',()=> {
      clearPlacement('AR 세션 종료'); send({kind:'end'}); session=null;
      hud.visible=false; handMarkers.forEach(m=>m.visible=false);
      [...controllerAxes,...eeAxes].forEach(a=>a.visible=false);
      document.querySelector('#panel').style.display='block'; enter.disabled=false;
    });
    session.addEventListener('visibilitychange',()=> {
      if (session.visibilityState!=='visible') send({kind:'pause',reason:'XR session hidden'});
    });
    await renderer.xr.setSession(session);
    renderer.xr.getReferenceSpace().addEventListener('reset',()=>clearPlacement('XR 원점 재설정 감지'));
    clearPlacement('공간 배치 준비'); shift=[0,0]; yaw=(manifest?.placement_yaw_deg??180)*Math.PI/180;
    document.querySelector('#panel').style.display='none';
  } catch(error) {status.textContent=`AR 시작 실패: ${error.message}`; session=null; enter.disabled=false;}
};

async function createAnchor(frame) {
  const generation=++anchorGeneration;
  if (!frame.createAnchor) return;
  try {
    const q=new THREE.Quaternion().setFromAxisAngle(new THREE.Vector3(0,1,0),yaw);
    const created=await frame.createAnchor(new XRRigidTransform({x:origin[0],y:origin[1],z:origin[2]},
      {x:q.x,y:q.y,z:q.z,w:q.w}),renderer.xr.getReferenceSpace());
    if (generation!==anchorGeneration || !placed) created.delete(); else anchor=created;
  } catch { /* local-floor remains the explicit fallback for this session */ }
}

function updateHUD(viewer, left, right, now) {
  if (!viewer) {hud.visible=false; return;}
  // Keep scene/attachment updates running, but remove the guidance panel
  // from the headset during collection. It returns on stop/review/reset.
  hud.visible=ui.episode?.state!=='RECORDING';
  const p=viewer.transform.position, q=viewer.transform.orientation;
  const rotation=new THREE.Quaternion(q.x,q.y,q.z,q.w);
  hud.position.set(0,-.28,-.9).applyQuaternion(rotation).add(new THREE.Vector3(p.x,p.y,p.z));
  hud.quaternion.copy(rotation);
  if (now-lastHUD<100) return; lastHUD=now;
  const context=hudCanvas.getContext('2d'); context.clearRect(0,0,1024,430);
  context.fillStyle='rgba(10,20,32,.86)'; context.fillRect(0,0,1024,430);
  context.font='bold 38px sans-serif'; context.fillStyle='#eaf5ff';
  let title=placed?(ui.episode?.state ?? 'UNRECORDED'):'PLACE WORLD: A to lock';
  const stale=(ui.scene_age ?? Infinity)+(now-sceneReceived)/1000>.5;
  if (stale) title='ISAAC SCENE STALE / HOLD';
  context.fillText(title,24,55);
  let distances=[],axisErrors=[];
  ['left','right'].forEach((side,i)=> {
    const hand=i===0?right:left, wrist=latest?.links?.[side+'_dof7_link'];
    const tip=wrist && manifest?.tcp_offset ? graspTip(wrist,manifest.tcp_offset) : null;
    const d=hand?.tracked && !hand.emulated && tip ? distanceInRobot(hand.matrix,inverse.elements,tip):Infinity;
    distances.push(d);
    const angles=hand?.tracked && wrist?axisErrorsInRobot(hand.matrix,inverse.elements,wrist,side):[Infinity,Infinity];
    axisErrors.push(angles);
    spheres[i].material.color.setHex(!Number.isFinite(d)||stale?0x89939c:d<=(manifest?.attach_radius??.04)+1e-9?0x49ee9b:0xffba55);
    spheres[i].visible=Boolean(latest) && ui.episode?.state!=='RECORDING';
  });
  context.font='32px sans-serif'; context.fillStyle='#b7d9ed';
  context.fillText(`L: ${Number.isFinite(distances[0])?(distances[0]*100).toFixed(2):'--'} cm   R: ${Number.isFinite(distances[1])?(distances[1]*100).toFixed(2):'--'} cm   <= ${(manifest?.attach_radius??.04)*100} cm`,24,110);
  context.fillText(`A/B 정렬 오차 L: ${axisErrors[0].map(a=>Number.isFinite(a)?a.toFixed(1):'--').join('/')}°  R: ${axisErrors[1].map(a=>Number.isFinite(a)?a.toFixed(1):'--').join('/')}°  (시작 각도 제한 없음)`,24,158);
  const waiting=placed && (!ui.episode || ui.episode.state==='READY');
  context.fillStyle=waiting && !ui.start_gate?.allowed ? '#ffba55' : '#b7d9ed';
  context.fillText(waiting?`서버: ${startGateText(ui.start_gate)}`:(ui.control ?? 'Waiting for Isaac').slice(0,58),24,206);
  const last=ui.last_start, showLast=last && now-lastStartReceived<8000 && waiting;
  if (showLast) {
    context.fillStyle=last.allowed?'#49ee9b':'#ffba55';
    const d=last.distances_m;
    context.fillText(`A 입력: ${last.allowed?'시작 승인':'시작 거부'}  L: ${d?(d[0]*100).toFixed(2):'--'} cm  R: ${d?(d[1]*100).toFixed(2):'--'} cm`,24,254);
    context.fillText(startGateText(last),24,302);
  } else {
    context.fillStyle='#b7d9ed';
    context.fillText(placed?'A: start/save   B: stop   X: discard':'Point right controller at floor; sticks adjust pose',24,254);
  }
  context.fillStyle='#b7d9ed';
  context.fillText(`Frames: ${ui.episode?.frames ?? 0}   Saved: ${ui.episode?.saved_episodes ?? 0}   Robot cameras only`,24,362);
  hudTexture.needsUpdate=true;
}

renderer.setAnimationLoop((now,frame)=> {
  const dt=Math.min((now-lastFrame)/1000,.05); lastFrame=now;
  if (frame && session) {
    const reference=renderer.xr.getReferenceSpace(), viewer=frame.getViewerPose(reference);
    const sources=Array.from(session.inputSources);
    const leftSource=sources.find(s=>s.handedness==='left'), rightSource=sources.find(s=>s.handedness==='right');
    const left=readController(leftSource,frame,reference), right=readController(rightSource,frame,reference);
    const visible=session.visibilityState==='visible';
    const valid=visible && left?.tracked && right?.tracked && !left.emulated && !right.emulated;
    const presses=edges.update(left,right,valid);
    if (!placed && right?.tracked) {
      const ray=rightSource.targetRaySpace?frame.getPose(rightSource.targetRaySpace,reference):null;
      const floor=floorPoint(ray?.transform.matrix,right.matrix);
      yaw+=(Math.abs(right.axes[0])>.2?right.axes[0]:0)*dt;
      if (left) {
        shift[0]+=(Math.abs(left.axes[0])>.2?left.axes[0]:0)*dt*.3;
        shift[1]+=(Math.abs(left.axes[1])>.2?left.axes[1]:0)*dt*.3;
      }
      origin=[floor[0]+shift[0],manifest?.placement_height??.7,floor[2]+shift[1]]; setWorld(origin,yaw);
      if (presses[0]==='a' && modelReady && ws?.readyState===WebSocket.OPEN) {
        placed=true; placementSequence=sequence; createAnchor(frame);
      }
    } else if (anchor) {
      const pose=frame.getPose(anchor.anchorSpace,reference);
      if (!pose) clearPlacement('공간 앵커 추적이 끊겼습니다.');
      else {
        const anchorMatrix=new THREE.Matrix4().fromArray(pose.transform.matrix);
        // Keep Isaac +Z vertical even if the spatial anchor has tiny tracked
        // roll/pitch changes. Only its position and yaw define placement.
        const e=anchorMatrix.elements;
        setWorld([e[12],e[13],e[14]],Math.atan2(e[8],e[10]));
      }
    }
    originMarker.visible=!placed;
    [left,right].forEach((hand,i)=> {
      handMarkers[i].visible=Boolean(hand?.tracked);
      controllerAxes[i].visible=Boolean(hand?.tracked);
      if (hand?.tracked) {controllerAxes[i].matrix.fromArray(hand.matrix);controllerAxes[i].matrixWorldNeedsUpdate=true;}
      if (hand?.tracked) handMarkers[i].position.set(hand.matrix[12],hand.matrix[13],hand.matrix[14]);
    });
    if (now-lastInput>=1000/60 || presses.length) {
      send({kind:'input',sequence:sequence++,space,placed,world:world.matrix.toArray(),
        model_id:modelReady?manifest.model_id:null,visible,head_tracked:Boolean(viewer),left,right});
      lastInput=now;
    }
    updateHUD(viewer,left,right,now);
  }
  renderer.render(scene,camera);
});
window.addEventListener('resize',()=> {
  camera.aspect=window.innerWidth/window.innerHeight; camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth,window.innerHeight);
});
setWorld(origin,yaw);
connect();
