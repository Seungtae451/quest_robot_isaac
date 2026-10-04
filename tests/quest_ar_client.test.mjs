import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
import {ButtonEdges,worldMatrix,floorPoint,distanceInRobot,readController,startGateText} from '../web/quest_ar/core.mjs';
import {GLTFLoader} from '../outputs/quest_ar/assets/vendor/three/examples/jsm/loaders/GLTFLoader.js';
import {Matrix4,Vector3} from '../outputs/quest_ar/assets/vendor/three/build/three.module.js';

const edges=new ButtonEdges();
const hand=()=>({state:{aButton:false,bButton:false}});
const left=hand(),right=hand();
right.state.aButton=true;
assert.deepEqual(edges.update(left,right,true),[]); // held A on enter cannot place/start
right.state.aButton=false; edges.update(left,right,true);
right.state.aButton=true; assert.deepEqual(edges.update(left,right,true),['a']);
assert.deepEqual(edges.update(left,right,true),[]);
edges.update(left,right,false); assert.deepEqual(edges.update(left,right,true),[]);

const world=new Matrix4().fromArray(worldMatrix([1,0,-.5],.7));
const pose=new Matrix4().makeTranslation(.4,.18,.57).premultiply(world);
assert.ok(distanceInRobot(pose.elements,world.clone().invert().elements,[.4,.18,.57])<1e-12);
assert.deepEqual(floorPoint(null,pose.elements),[pose.elements[12],0,pose.elements[14]]);
const source={gripSpace:{},gamepad:{mapping:'xr-standard',buttons:Array.from({length:6},()=>({value:0,pressed:false})),axes:[0,0,.1,-.2]}};
source.gamepad.buttons[4].pressed=true;
const controller=readController(source,{getPose:()=>({transform:{matrix:pose.elements},emulatedPosition:false})},{});
assert.equal(controller.state.aButton,true);
assert.deepEqual(controller.axes,[.1,-.2]);
assert.equal(startGateText({allowed:false,reasons:['LEFT_OUTSIDE_5CM','RIGHT_OUTSIDE_5CM']}),
             '왼손 추적점 5cm 밖 · 오른손 추적점 5cm 밖');
assert.equal(startGateText({allowed:false,reasons:['HOME_JOINT_ERROR']}),'초기 관절 위치 오차');
assert.match(startGateText({allowed:true,reasons:[]}),/연결 준비 완료/);

// Parse the exported asset using the real browser loader, not a hand-written
// GLB decoder. All 19 measured link names must be directly addressable.
const bytes=await readFile(new URL('../outputs/quest_ar/assets/robot.glb',import.meta.url));
const gltf=await new GLTFLoader().parseAsync(bytes.buffer.slice(bytes.byteOffset,bytes.byteOffset+bytes.byteLength),'');
const manifest=JSON.parse(await readFile(new URL('../outputs/quest_ar/assets/manifest.json',import.meta.url),'utf8'));
const meshes=new Map(); gltf.scene.traverse(o=>{if(o.isMesh) meshes.set(o.name,o);});
assert.equal(manifest.geometry_mode,'full_urdf');
assert.equal(manifest.simplified,false);
assert.equal(meshes.size,manifest.visual_count);
const sourceRoot=new URL('../',`file://${manifest.source_urdf}`);
for (const name of manifest.links) {
  const link=gltf.scene.getObjectByName(name);
  assert.ok(link?.isObject3D && !link.isMesh,name);
  assert.ok(link.children.length>0);
}
let faces=0;
for (const visual of manifest.meshes) {
  const mesh=meshes.get(visual.name);
  assert.ok(mesh,visual.name);
  assert.equal(mesh.parent.name,visual.link);
  const geometry=mesh.geometry;
  geometry.computeBoundingBox();
  assert.ok(geometry.boundingBox.min.toArray().every(Number.isFinite));
  // Independently compare EVERY original binary STL face to the geometry that
  // the actual GLTFLoader renders. Current URDF visual origins/scales are identity.
  const source=await readFile(new URL(visual.source_mesh,sourceRoot));
  const count=source.readUInt32LE(80);
  assert.equal(source.length,84+count*50);
  assert.equal(geometry.index.count,count*3,visual.name);
  const positions=geometry.attributes.position.array,indices=geometry.index.array;
  for(let face=0;face<count;face++) {
    for(let corner=0;corner<3;corner++) {
      const index=indices[face*3+corner]*3;
      for(let axis=0;axis<3;axis++) {
        assert.equal(positions[index+axis],source.readFloatLE(84+face*50+12+corner*12+axis*4),
                     `${visual.name} face ${face}`);
      }
    }
  }
  faces+=count;
}
assert.equal(faces,manifest.triangle_count);
assert.equal(gltf.scene.getObjectByName('base_link').children.length,2);
// Measured transforms move all visuals together, including both base pieces.
const base=gltf.scene.getObjectByName('base_link');
base.position.set(.2,.3,.4); base.updateMatrixWorld(true);
for (const visual of base.children) {
  assert.deepEqual(visual.getWorldPosition(new Vector3()).toArray(),[.2,.3,.4]);
}
console.log(`Quest AR client passed: ${manifest.links.length} links, ${meshes.size} visuals, ${faces} source triangles EXACTLY preserved; button/mapping checks passed.`);
