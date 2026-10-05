// Pure helpers shared with deterministic desktop tests. Units are metres.
export const XR_FROM_ISAAC = [0, 0, -1, -1, 0, 0, 0, 1, 0]; // column-major

export function readController(source, frame, reference) {
  if (!source?.gripSpace || !source.gamepad || source.gamepad.mapping !== 'xr-standard') return null;
  const pose = frame.getPose(source.gripSpace, reference);
  const buttons = source.gamepad.buttons;
  return {
    matrix: pose ? Array.from(pose.transform.matrix) : null,
    tracked: Boolean(pose), emulated: pose ? Boolean(pose.emulatedPosition) : true,
    state: {triggerValue: buttons[0]?.value ?? 0, squeezeValue: buttons[1]?.value ?? 0,
            aButton: Boolean(buttons[4]?.pressed), bButton: Boolean(buttons[5]?.pressed)},
    axes: [source.gamepad.axes[2] ?? 0, source.gamepad.axes[3] ?? 0],
  };
}

export class ButtonEdges {
  constructor() { this.previous = [false, false, false]; this.armed = false; }
  reset() { this.previous = [false, false, false]; this.armed = false; }
  update(left, right, valid) {
    if (!valid) { this.reset(); return []; }
    const current = [right.state.aButton, right.state.bButton, left.state.aButton];
    const edges = [];
    if (!this.armed) this.armed = !current.some(Boolean);
    else for (const i of [2, 1, 0]) if (current[i] && !this.previous[i]) edges.push(['a', 'b', 'x'][i]);
    this.previous = current;
    return edges;
  }
}

export function worldMatrix(origin, yaw) {
  const c = Math.cos(yaw), s = Math.sin(yaw);
  return [-s, 0, -c, 0, -c, 0, s, 0, 0, 1, 0, 0, ...origin, 1];
}

export function floorPoint(rayMatrix, fallback) {
  if (rayMatrix) {
    const y = rayMatrix[13], dy = -rayMatrix[9];
    const t = -y / dy;
    if (dy < -.05 && t > 0 && t < 5)
      return [rayMatrix[12] - t * rayMatrix[8], 0, rayMatrix[14] - t * rayMatrix[10]];
  }
  return [fallback[12], 0, fallback[14]];
}

export function distanceInRobot(gripMatrix, worldInverse, wrist) {
  const x = gripMatrix[12], y = gripMatrix[13], z = gripMatrix[14];
  const p = [0, 1, 2].map(i => worldInverse[i] * x + worldInverse[4+i] * y + worldInverse[8+i] * z + worldInverse[12+i]);
  return Math.hypot(...p.map((v, i) => v - wrist[i]));
}

export function graspTip(pose,offset) {
  const q=pose.slice(3),length=Math.hypot(...q);
  const [w,x,y,z]=q.map(v=>v/length),[a,b,c]=offset;
  return [pose[0]+(1-2*(y*y+z*z))*a+2*(x*y-w*z)*b+2*(x*z+w*y)*c,
          pose[1]+2*(x*y+w*z)*a+(1-2*(x*x+z*z))*b+2*(y*z-w*x)*c,
          pose[2]+2*(x*z-w*y)*a+2*(y*z+w*x)*b+(1-2*(x*x+y*y))*c];
}

export function startGateText(gate) {
  if (!gate) return '서버 판단 대기';
  if (gate.allowed) return '연결 준비 완료 · A로 시작';
  const labels = {
    LEFT_OUTSIDE_5CM:'오른손→로봇 왼팔 끝 5cm 밖', RIGHT_OUTSIDE_5CM:'왼손→로봇 오른팔 끝 5cm 밖',
    LEFT_OUTSIDE_4CM:'오른손→로봇 L 중앙 4cm 밖', RIGHT_OUTSIDE_4CM:'왼손→로봇 R 중앙 4cm 밖',
    LEFT_OUTSIDE_1CM:'오른손→로봇 L 중앙 1cm 밖', RIGHT_OUTSIDE_1CM:'왼손→로봇 R 중앙 1cm 밖',
    LEFT_AXES_MISALIGNED:'로봇 L의 A/B 대응 축 10° 밖', RIGHT_AXES_MISALIGNED:'로봇 R의 A/B 대응 축 10° 밖',
    PLACE_WORLD:'첫 A로 공간 고정', NO_PRESS_POSE:'버튼 순간 위치 없음',
    SCENE_MISSING:'Isaac 장면 대기', SCENE_STALE:'Isaac 장면 지연',
    TRACKING_OR_SCENE_STALE:'추적/장면 지연', ISAAC_FEEDBACK_STALE:'Isaac 응답 대기',
    ISAAC_RESTARTED:'Isaac 재연결 대기', NEUTRAL_NOT_READY:'중립 기준 준비 중',
    RECORDER_UNAVAILABLE:'녹화 서버 응답 대기', HOME_NOT_READY:'HOME 준비 대기',
    HOME_COMMAND_STALE:'HOME 유지 명령 대기', HOME_JOINT_ERROR:'초기 관절 위치 오차',
    HOME_STILL_MOVING:'HOME 이동 중', HOME_GRIPPER_NOT_OPEN:'초기 그리퍼 열림 대기',
    EPISODE_COMMAND_PENDING:'이전 버튼 처리 중',
  };
  return gate.reasons.map(reason => labels[reason] ?? reason).join(' · ');
}


export function axisErrorsInRobot(gripMatrix, worldInverse, pose, robotSide='left') {
  const [w,x,y,z]=pose.slice(3).map(v=>v/Math.hypot(...pose.slice(3)));
  const eeX=[1-2*(y*y+z*z),2*(x*y+w*z),2*(x*z-w*y)];
  const eeZ=[2*(x*z+w*y),2*(y*z-w*x),1-2*(x*x+y*y)];
  const sign=robotSide==='left'?1:-1;
  return [[eeX,1,sign],[eeZ,0,-sign]].map(([axis,j,direction])=> {
    const c=[0,1,2].map(i=>direction*[0,1,2].reduce((v,k)=>v+worldInverse[k*4+i]*gripMatrix[j*4+k],0));
    return Math.acos(Math.max(-1,Math.min(1,axis.reduce((v,a,i)=>v+a*c[i],0))))*180/Math.PI;
  });
}
