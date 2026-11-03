// Dual NERO browser dashboard.
//
// The 3D viewer loads the same xacro-expanded URDF used by ROS. Motion requests
// go to MoveIt through /api/move and /api/park; feedback arrives over /ws at
// 10 Hz. Camera tiles use MJPEG so the RealSense nodes remain the only process
// opening the camera devices.

const statusEl = document.getElementById('status');
const summaryEl = document.getElementById('safety-summary');
const motionStatusEl = document.getElementById('motion-status');
const bannerEl = document.getElementById('banner');
const viewport = document.getElementById('viewport');
const toastEl = document.getElementById('toast');

const app = {
  config: null,
  jointNames: [],
  limits: [],
  sides: {},
  ws: null,
  wsTimer: null,
  cameras: {},
  robot: null,
  three: null,
  scene: null,
  camera: null,
  renderer: null,
  controls: null,
  motion: null,
  bothSupported: false,
  bothBusy: false,
  bothAction: null,
  toastTimer: null,
};

const $ = (selector) => document.querySelector(selector);

function showToast(message, kind = '') {
  toastEl.textContent = message;
  toastEl.className = `toast ${kind}`.trim();
  clearTimeout(app.toastTimer);
  app.toastTimer = setTimeout(() => toastEl.classList.add('hidden'), 4500);
}

function showBanner(message) {
  bannerEl.textContent = message;
  bannerEl.classList.remove('hidden');
}

async function postJson(url, body, timeoutMs = 125000) {
  const controller = new AbortController();
  const timeout = setTimeout(() => controller.abort(), timeoutMs);
  let response;
  try {
    response = await fetch(url, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(body || {}),
      signal: controller.signal,
    });
  } catch (error) {
    if (error.name === 'AbortError') {
      throw new Error('request timed out; check the robot or use Cancel motion');
    }
    throw error;
  } finally {
    clearTimeout(timeout);
  }
  const text = await response.text();
  let data = {};
  try {
    data = text ? JSON.parse(text) : {};
  } catch (_) {
    data = { error: text || `HTTP ${response.status}` };
  }
  if (!response.ok && !data.error) data.error = `HTTP ${response.status}`;
  return data;
}

function setPill(side, role, text, tone = '') {
  const panel = document.querySelector(`.panel[data-side="${side}"]`);
  if (!panel) return;
  const pill = panel.querySelector(`[data-role="${role}"]`);
  if (!pill) return;
  pill.textContent = text;
  pill.className = `pill ${tone}`.trim();
}

function updateMotionUi(motion) {
  if (!motion) return;
  app.motion = motion;
  let text = 'MoveIt: unavailable';
  let tone = 'warn';
  const dryRun = motion.dry_run === true;
  if (motion.available) {
    if (motion.busy) {
      const side = motion.active_side ? ` ${motion.active_side}` : '';
      text = `MoveIt: planning${side}${dryRun ? ' (DRY-RUN)' : ''}`;
      tone = 'warn';
    } else {
      text = dryRun ? 'MoveIt: ready (DRY-RUN)' : 'MoveIt: ready (LIVE)';
      tone = dryRun ? 'warn' : 'ok';
    }
  }
  motionStatusEl.title = dryRun
    ? 'The trajectory bridge is in dry-run mode: Send and Park do not command the arms.'
    : 'The trajectory bridge is in hardware mode: Send and Park command the arms.';
  motionStatusEl.textContent = text;
  motionStatusEl.className = `safety-summary ${tone}`;
}

function refreshMotionButtons() {
  const sides = app.config?.sides || [];
  const sideBusy = sides.some((side) => app.sides[side]?.motionBusy);
  const anyBusy = sideBusy || app.bothBusy;
  for (const side of sides) {
    const state = app.sides[side];
    const sendButton = document.querySelector(`.panel[data-side="${side}"] .btn-send`);
    const parkButton = document.querySelector(`.panel[data-side="${side}"] .btn-park`);
    if (sendButton) {
      sendButton.disabled = anyBusy;
      sendButton.textContent = state?.motionBusy && state.motionAction === 'move'
        ? 'Sending…' : 'Send';
    }
    if (parkButton) {
      parkButton.disabled = anyBusy || app.config?.park?.available?.[side] === false;
      parkButton.textContent = state?.motionBusy && state.motionAction === 'park'
        ? 'Parking…' : 'Park';
    }
  }
  const bothSend = $('#both-send');
  const bothPark = $('#both-park');
  if (bothSend) {
    bothSend.disabled = anyBusy || !app.bothSupported;
    bothSend.textContent = app.bothBusy && app.bothAction === 'move'
      ? 'Sending both…' : 'Both Send';
  }
  if (bothPark) {
    bothPark.disabled = anyBusy || !app.bothSupported
      || app.config?.park?.available?.both === false;
    bothPark.textContent = app.bothBusy && app.bothAction === 'park'
      ? 'Parking both…' : 'Both Park';
  }
}

function setMotionBusy(side, busy, action = 'move') {
  const state = app.sides[side];
  if (!state) return;
  state.motionBusy = busy;
  state.motionAction = busy ? action : null;
  refreshMotionButtons();
}

function setBothMotionBusy(busy, action = 'move') {
  app.bothBusy = busy;
  app.bothAction = busy ? action : null;
  for (const side of app.config?.sides || []) {
    const state = app.sides[side];
    if (!state) continue;
    state.motionBusy = busy;
    state.motionAction = busy ? action : null;
  }
  refreshMotionButtons();
}

function bothArmsReady() {
  if (!app.bothSupported) return false;
  const sides = app.config?.sides || [];
  return sides.length === 2 && sides.every((side) => {
    const state = app.sides[side];
    return state
      && state.gate
      && state.enabled === true
      && state.feedbackAge !== null
      && state.feedbackAge <= 1.5;
  });
}

function updateSafetyUi(side) {
  const state = app.sides[side];
  const panel = document.querySelector(`.panel[data-side="${side}"]`);
  if (!state || !panel) return;

  const feedbackTone = state.feedbackAge === null ? ''
    : state.feedbackAge > 1.0 ? 'warn' : 'ok';
  const feedbackText = state.feedbackAge === null ? 'fb –'
    : state.feedbackAge > 1.0 ? `fb stale ${state.feedbackAge.toFixed(1)}s` : 'fb live';
  setPill(side, 'feedback', feedbackText, feedbackTone);
  setPill(side, 'gate', state.gate ? 'gate open' : 'gate closed',
    state.gate ? 'ok' : 'warn');

  let enabledText = 'motors –';
  let enabledTone = '';
  if (state.enabled === true) {
    enabledText = 'motors enabled';
    enabledTone = 'ok';
  } else if (state.enabled === false) {
    enabledText = 'motors disabled';
    enabledTone = 'warn';
  }
  setPill(side, 'enabled', enabledText, enabledTone);

  const health = state.health;
  let healthText = 'arm –';
  let healthTone = '';
  const healthFresh = health
    && health.arm_status !== null
    && typeof health.arm_status_age_sec === 'number'
    && health.arm_status_age_sec <= 2.0;
  if (healthFresh) {
    const faults = [...(health.motor_faults || [])];
    if (health.err_status) faults.push(`arm err ${health.err_status}`);
    if ((health.joint_limit_flags || []).some(Boolean)) faults.push('joint limit');
    if ((health.communication_flags || []).some(Boolean)) faults.push('joint comm');
    if (faults.length) {
      healthText = `fault: ${faults[0]}`;
      healthTone = 'bad';
    } else if (health.arm_status === 6) {
      if (health.enabled_joints === null) {
        healthText = 'brake state unknown';
        healthTone = 'warn';
      } else if (health.enabled_joints > 0) {
        healthText = 'brake stuck';
        healthTone = 'bad';
      } else {
        healthText = 'brakes set';
        healthTone = 'warn';
      }
    } else if (health.arm_status === 0) {
      healthText = 'arm ok';
      healthTone = 'ok';
    } else {
      healthText = `arm ${health.arm_status_text || health.arm_status}`;
      healthTone = 'warn';
    }
  }
  setPill(side, 'health', healthText, healthTone);

  const gateButton = panel.querySelector('.btn-gate');
  gateButton.textContent = state.gate ? 'Close gate' : 'Open gate';
  gateButton.classList.toggle('active', state.gate);

  const enableButton = panel.querySelector('.btn-enable');
  enableButton.textContent = state.enabled === true ? 'Disable motors' : 'Enable motors';
  enableButton.classList.toggle('active', state.enabled === true);

  const clearButton = panel.querySelector('.btn-clear-faults');
  const recoverButton = panel.querySelector('.btn-recover');
  if (clearButton) {
    clearButton.disabled = Boolean(state.healthBusy) || !healthFresh;
    clearButton.textContent = state.healthBusy === 'clear' ? 'Clearing…' : 'Clear faults';
  }
  if (recoverButton) {
    recoverButton.disabled = Boolean(state.healthBusy) || !healthFresh
      || state.gate || state.motionBusy;
    recoverButton.textContent = state.healthBusy === 'recover' ? 'Recovering…' : 'Recover arm';
  }

  const leftGate = app.sides.left?.gate ? 'open' : 'closed';
  const rightGate = app.sides.right?.gate ? 'open' : 'closed';
  summaryEl.textContent = `gates: left ${leftGate} · right ${rightGate}`;
  const bothState = document.getElementById('both-state');
  if (bothState) {
    const ready = bothArmsReady();
    bothState.textContent = !app.bothSupported
      ? 'restart backend' : ready ? 'both ready' : 'both not ready';
    bothState.className = `pill ${ready ? 'ok' : 'warn'}`;
  }
}

function applyArmPose(side) {
  if (!app.robot || !app.sides[side]) return;
  const state = app.sides[side];
  for (let index = 0; index < app.jointNames.length; index += 1) {
    const joint = app.robot.joints[`${side}_${app.jointNames[index]}`];
    if (joint) joint.setJointValue(state.cmd[index]);
  }
}

function setSlider(side, index, value, updatePose = true) {
  const state = app.sides[side];
  const row = state?.rowEls[index];
  if (!row) return;
  const [low, high] = app.limits[index];
  const clamped = Math.max(low, Math.min(high, Number(value)));
  state.cmd[index] = clamped;
  const input = row.querySelector('input');
  const command = row.querySelector('.cmd');
  input.value = String(clamped);
  command.textContent = clamped.toFixed(3);
  if (updatePose) applyArmPose(side);
}

function syncSlidersFromFeedback(side, force = false) {
  const state = app.sides[side];
  if (!state) return;
  if (!force && state.poseInitialized) return;
  if (state.fb.some((value) => value === null)) return;
  for (let index = 0; index < state.fb.length; index += 1) {
    setSlider(side, index, state.fb[index], false);
  }
  state.poseInitialized = true;
  applyArmPose(side);
}

function buildSliders() {
  for (const side of app.config.sides) {
    const panel = document.querySelector(`.panel[data-side="${side}"]`);
    if (!panel) continue;
    const state = app.sides[side];
    const container = panel.querySelector('.sliders');
    container.innerHTML = '';

    for (let index = 0; index < app.jointNames.length; index += 1) {
      const [low, high] = app.limits[index];
      const row = document.createElement('div');
      row.className = 'joint-row';
      row.innerHTML = `
        <label>J${index + 1}</label>
        <input type="range" min="${low}" max="${high}" step="0.005" value="${state.cmd[index]}" />
        <span class="cmd">${state.cmd[index].toFixed(3)}</span>
        <span class="fb">fb –</span>
      `;
      const input = row.querySelector('input');
      const command = row.querySelector('.cmd');
      input.addEventListener('input', () => {
        const value = Number(input.value);
        state.cmd[index] = value;
        command.textContent = value.toFixed(3);
        applyArmPose(side);
      });
      container.appendChild(row);
      state.rowEls.push(row);
    }

    panel.querySelector('.btn-send').addEventListener('click', () => sendCommand(side));
    const parkButton = panel.querySelector('.btn-park');
    const parkAvailable = app.config.park?.available?.[side] !== false;
    parkButton.disabled = !parkAvailable;
    parkButton.title = parkAvailable
      ? 'Plan and execute the configured support pose with MoveIt'
      : (app.config.park?.errors?.[side] || 'Park pose is not configured');
    parkButton.addEventListener('click', () => parkSide(side));
    panel.querySelector('.btn-sync').addEventListener('click', () => {
      if (state.fb.some((value) => value === null)) {
        showToast(`${side}: no complete feedback yet`, 'error');
        return;
      }
      state.poseInitialized = false;
      syncSlidersFromFeedback(side, true);
    });
    panel.querySelector('.btn-zero').addEventListener('click', () => {
      for (let index = 0; index < app.jointNames.length; index += 1) {
        setSlider(side, index, 0, false);
      }
      applyArmPose(side);
    });
    panel.querySelector('.btn-gate').addEventListener('click', () => toggleGate(side));
    panel.querySelector('.btn-enable').addEventListener('click', () => toggleMotors(side));
    panel.querySelector('.btn-estop').addEventListener('click', () => stopSide(side));
    panel.querySelector('.btn-clear-faults').addEventListener('click', () => clearFaults(side));
    panel.querySelector('.btn-recover').addEventListener('click', () => recoverArm(side));
  }

  const bothParkAvailable = app.bothSupported
    && app.config.park?.available?.both !== false;
  const bothParkButton = $('#both-park');
  if (bothParkButton) {
    bothParkButton.disabled = !bothParkAvailable;
    bothParkButton.title = bothParkAvailable
      ? 'Plan and execute both configured Park poses in one synchronized MoveIt trajectory'
      : (!app.bothSupported
        ? 'Restart the webapp backend to enable synchronized both-arm motion'
        : (app.config.park?.errors?.both || 'Both Park poses are not configured'));
  }
  $('#both-send').addEventListener('click', sendBoth);
  bothParkButton.addEventListener('click', parkBoth);
  $('#stop-all').addEventListener('click', stopAll);
  $('#cancel-motion').addEventListener('click', cancelMotion);
  refreshMotionButtons();
}

async function sendCommand(side) {
  const state = app.sides[side];
  if (!state || state.motionBusy) return;
  if (!state?.gate) {
    showToast(`${side}: open the control gate before sending`, 'error');
    return;
  }
  if (state.enabled !== true) {
    showToast(`${side}: enable motors before sending`, 'error');
    return;
  }
  if (state.feedbackAge === null || state.feedbackAge > 1.5) {
    showToast(`${side}: feedback is missing or stale; refusing motion`, 'error');
    return;
  }
  if (app.motion?.available === false) {
    showToast('MoveIt is unavailable; start the MoveIt action server', 'error');
    return;
  }

  const dryRun = app.motion?.dry_run === true;
  setMotionBusy(side, true, 'move');
  showToast(
    `${side}: MoveIt ${dryRun ? 'dry-run ' : ''}planning and executing…`,
    dryRun ? 'warn' : '',
  );
  let result;
  try {
    result = await postJson('/api/move', { side, positions: state.cmd });
  } catch (error) {
    result = { ok: false, error: error.message || 'motion request failed' };
  } finally {
    setMotionBusy(side, false, 'move');
  }
  if (!result.ok) {
    showToast(`${side}: ${result.error || 'MoveIt motion failed'}`, 'error');
    return;
  }
  const resultDryRun = typeof result.dry_run === 'boolean' ? result.dry_run : dryRun;
  showToast(
    `${side}: ${result.message || 'MoveIt motion completed'}`,
    resultDryRun ? 'warn' : 'ok',
  );
}

async function parkSide(side) {
  const state = app.sides[side];
  if (!state || state.motionBusy) return;
  if (app.config.park?.available?.[side] === false) {
    showToast(`${side}: ${app.config.park.errors?.[side] || 'park pose unavailable'}`, 'error');
    return;
  }
  if (!state.gate) {
    showToast(`${side}: open the control gate before parking`, 'error');
    return;
  }
  if (state.enabled !== true) {
    showToast(`${side}: enable motors before parking`, 'error');
    return;
  }
  if (state.feedbackAge === null || state.feedbackAge > 1.5) {
    showToast(`${side}: feedback is missing or stale; refusing park`, 'error');
    return;
  }
  const dryRun = app.motion?.dry_run === true;
  const confirmed = window.confirm(
    `Send the ${side} arm to its configured PARK pose?\n\n`
    + 'MoveIt will plan and execute a collision-checked motion. The workspace '
    + 'must still be clear of people and unmodelled obstacles.'
    + (dryRun ? '\n\nDRY-RUN is active: the robot will NOT move.' : ''),
  );
  if (!confirmed) return;

  setMotionBusy(side, true, 'park');
  showToast(
    `${side}: MoveIt ${dryRun ? 'dry-run ' : ''}planning park motion…`,
    dryRun ? 'warn' : '',
  );
  let result;
  try {
    result = await postJson('/api/park', { side });
  } catch (error) {
    result = { ok: false, error: error.message || 'park request failed' };
  } finally {
    setMotionBusy(side, false, 'park');
  }
  if (!result.ok) {
    showToast(`${side}: ${result.error || 'park request failed'}`, 'error');
    return;
  }
  if (Array.isArray(result.positions) && result.positions.length === app.jointNames.length) {
    for (let index = 0; index < result.positions.length; index += 1) {
      setSlider(side, index, result.positions[index], false);
    }
    state.poseInitialized = true;
    applyArmPose(side);
  }
  const resultDryRun = typeof result.dry_run === 'boolean' ? result.dry_run : dryRun;
  showToast(
    `${side}: ${result.message || 'MoveIt park motion completed'}`,
    resultDryRun ? 'warn' : 'ok',
  );
}

function bothReadinessError() {
  if (app.bothBusy) return 'a both-arm motion is already active';
  for (const side of app.config.sides) {
    const state = app.sides[side];
    if (!state) return `${side}: arm state is unavailable`;
    if (state.motionBusy) return `${side}: another motion is already active`;
    if (!state.gate) return `${side}: open the control gate before moving`;
    if (state.enabled !== true) return `${side}: enable motors before moving`;
    if (state.feedbackAge === null || state.feedbackAge > 1.5) {
      return `${side}: feedback is missing or stale; refusing motion`;
    }
  }
  if (app.motion?.available === false) {
    return 'MoveIt is unavailable; start the MoveIt action server';
  }
  return null;
}

async function sendBoth() {
  if (!app.bothSupported) {
    showToast('restart the webapp backend to enable both-arm motion', 'error');
    return;
  }
  if (app.bothBusy) return;
  const readinessError = bothReadinessError();
  if (readinessError) {
    showToast(readinessError, 'error');
    return;
  }

  const dryRun = app.motion?.dry_run === true;
  setBothMotionBusy(true, 'move');
  showToast(
    `both arms: MoveIt ${dryRun ? 'dry-run ' : ''}planning and executing one synchronized trajectory…`,
    dryRun ? 'warn' : '',
  );
  let result;
  try {
    result = await postJson('/api/move', {
      side: 'both',
      positions: {
        left: app.sides.left.cmd,
        right: app.sides.right.cmd,
      },
    });
  } catch (error) {
    result = { ok: false, error: error.message || 'both-arm motion request failed' };
  } finally {
    setBothMotionBusy(false, 'move');
  }
  if (!result.ok) {
    showToast(`both arms: ${result.error || 'MoveIt motion failed'}`, 'error');
    return;
  }
  const resultDryRun = typeof result.dry_run === 'boolean' ? result.dry_run : dryRun;
  showToast(
    result.message || 'both arms: MoveIt motion completed',
    resultDryRun ? 'warn' : 'ok',
  );
}

async function parkBoth() {
  if (!app.bothSupported) {
    showToast('restart the webapp backend to enable both-arm motion', 'error');
    return;
  }
  if (app.bothBusy) return;
  if (app.config.park?.available?.both === false) {
    showToast(app.config.park.errors?.both || 'both Park poses are unavailable', 'error');
    return;
  }
  const readinessError = bothReadinessError();
  if (readinessError) {
    showToast(readinessError, 'error');
    return;
  }

  const dryRun = app.motion?.dry_run === true;
  const confirmed = window.confirm(
    'Send both arms to their configured PARK poses in one synchronized motion?\n\n'
    + 'MoveIt will plan and collision-check the complete 14-joint trajectory. '
    + 'The workspace must still be clear of people and unmodelled obstacles.'
    + (dryRun ? '\n\nDRY-RUN is active: the robot will NOT move.' : ''),
  );
  if (!confirmed) return;

  setBothMotionBusy(true, 'park');
  showToast(
    `both arms: MoveIt ${dryRun ? 'dry-run ' : ''}planning synchronized Park motion…`,
    dryRun ? 'warn' : '',
  );
  let result;
  try {
    result = await postJson('/api/park', { side: 'both' });
  } catch (error) {
    result = { ok: false, error: error.message || 'both-arm park request failed' };
  } finally {
    setBothMotionBusy(false, 'park');
  }
  if (!result.ok) {
    showToast(`both arms: ${result.error || 'park request failed'}`, 'error');
    return;
  }
  if (result.positions && !Array.isArray(result.positions)) {
    for (const side of app.config.sides) {
      const positions = result.positions[side];
      if (!Array.isArray(positions) || positions.length !== app.jointNames.length) continue;
      for (let index = 0; index < positions.length; index += 1) {
        setSlider(side, index, positions[index], false);
      }
      app.sides[side].poseInitialized = true;
      applyArmPose(side);
    }
  }
  const resultDryRun = typeof result.dry_run === 'boolean' ? result.dry_run : dryRun;
  showToast(
    result.message || 'both arms: MoveIt park motion completed',
    resultDryRun ? 'warn' : 'ok',
  );
}

async function toggleGate(side) {
  const state = app.sides[side];
  const enabled = !state.gate;
  const result = await postJson('/api/control_gate', { side, enabled });
  if (!result.ok) {
    showToast(`${side}: ${result.error || 'gate request failed'}`, 'error');
    return;
  }
  state.gate = Boolean(result.enabled);
  updateSafetyUi(side);
  showToast(`${side}: control gate ${state.gate ? 'opened' : 'closed'}`,
    state.gate ? 'ok' : '');
}

async function toggleMotors(side) {
  const state = app.sides[side];
  const enabled = state.enabled !== true;
  if (!enabled && !window.confirm(`Disable ${side} motors?`)) return;
  const result = await postJson('/api/enable', { side, enabled });
  if (!result.ok) {
    showToast(`${side}: ${result.error || 'enable request failed'}`, 'error');
    return;
  }
  state.enabled = Boolean(result.enabled);
  updateSafetyUi(side);
  showToast(`${side}: motors ${state.enabled ? 'enabled' : 'disabled'}`,
    state.enabled ? 'ok' : '');
}

async function clearFaults(side) {
  const state = app.sides[side];
  if (!state || state.healthBusy) return;
  if (state.motionBusy || app.motion?.busy) {
    showToast(`${side}: cancel the active motion before clearing faults`, 'error');
    return;
  }
  const confirmed = window.confirm(
    `Send the NERO joint fault-clear command to the ${side} arm?\n\n`
    + 'Only do this after the cause of the fault has been removed. The arm '
    + 'must be supported and the workspace clear. This does not reset the '
    + 'motion controller, release brakes, or move the arm.',
  );
  if (!confirmed) return;

  state.healthBusy = 'clear';
  updateSafetyUi(side);
  let result;
  try {
    result = await postJson('/api/clear_faults', { side });
  } catch (error) {
    result = { ok: false, error: error.message || 'fault-clear request failed' };
  } finally {
    state.healthBusy = null;
    updateSafetyUi(side);
  }
  if (!result.ok) {
    showToast(`${side}: ${result.error || 'fault-clear request failed'}`, 'error');
    return;
  }
  showToast(`${side}: ${result.message || 'joint fault-clear command sent'}`, 'ok');
}

async function recoverArm(side) {
  const state = app.sides[side];
  if (!state || state.healthBusy) return;
  if (state.gate) {
    showToast(`${side}: close the control gate before recovering`, 'error');
    return;
  }
  if (state.motionBusy || app.motion?.busy) {
    showToast(`${side}: cancel the active motion before recovering`, 'error');
    return;
  }
  const confirmed = window.confirm(
    `Recover the ${side} arm?\n\n`
    + 'This will reset the motion controller, clear latched joint faults, and '
    + 'RE-ENABLE the motors. Enabling applies torque and may move the arm '
    + 'suddenly.\n\n'
    + 'The arm must be physically supported, the gate must be closed, and the '
    + 'workspace must be clear. This does not send the arm to Park.',
  );
  if (!confirmed) return;

  state.healthBusy = 'recover';
  updateSafetyUi(side);
  let result;
  try {
    result = await postJson('/api/recover', { side });
  } catch (error) {
    result = { ok: false, error: error.message || 'recovery request failed' };
  } finally {
    state.healthBusy = null;
    updateSafetyUi(side);
  }
  if (!result.ok) {
    showToast(`${side}: ${result.error || 'recovery request failed'}`, 'error');
    return;
  }
  state.enabled = true;
  updateSafetyUi(side);
  showToast(`${side}: ${result.message || 'arm recovered and motors enabled'}`, 'ok');
}

async function stopSide(side) {
  const state = app.sides[side];
  state.gate = false;
  updateSafetyUi(side);
  const result = await postJson('/api/emergency_stop', { side });
  if (!result.ok) {
    showToast(`${side}: ${result.error || 'emergency stop failed'}`, 'error');
    return;
  }
  const suffix = result.gate_closed ? '' : ` (gate: ${result.gate_error || 'not closed'})`;
  showToast(`${side}: emergency stop sent; gate closed${suffix}`, 'error');
}

async function stopAll() {
  for (const side of app.config.sides) {
    app.sides[side].gate = false;
    updateSafetyUi(side);
  }
  const result = await postJson('/api/emergency_stop_all', {});
  if (!result.ok) {
    const details = Object.entries(result.results || {})
      .filter(([, sideResult]) => !sideResult.ok)
      .map(([side, sideResult]) => `${side}: ${sideResult.error || 'failed'}`)
      .join('; ');
    showToast(details ? `Stop both incomplete — ${details}` : 'emergency stop failed', 'error');
    return;
  }
  showToast('both arms: emergency stop sent; gates closed', 'error');
}

async function cancelMotion() {
  const side = app.motion?.active_side || null;
  if (!side) {
    showToast('no active MoveIt motion to cancel', 'error');
    return;
  }
  let result;
  try {
    result = await postJson('/api/cancel_motion', { side }, 10000);
  } catch (error) {
    result = { ok: false, error: error.message || 'cancel request failed' };
  }
  if (!result.ok) {
    showToast(result.error || 'cancel request failed', 'error');
    return;
  }
  showToast(result.message || `${side}: cancellation requested`, 'warn');
}

function updateFeedback(side, payload) {
  const state = app.sides[side];
  if (!state || !payload) return;
  state.feedbackAge = typeof payload.feedback_age_sec === 'number'
    ? payload.feedback_age_sec : null;
  state.gate = Boolean(payload.gate_open);
  state.enabled = typeof payload.motor_enabled === 'boolean'
    ? payload.motor_enabled : null;
  state.health = payload.health || null;

  const byName = {};
  const names = payload.names || [];
  const positions = payload.positions || [];
  for (let index = 0; index < names.length; index += 1) {
    byName[names[index]] = positions[index];
  }
  for (let index = 0; index < app.jointNames.length; index += 1) {
    const value = byName[app.jointNames[index]];
    const row = state.rowEls[index];
    if (!row) continue;
    const fb = row.querySelector('.fb');
    if (state.feedbackAge !== null && Number.isFinite(Number(value))) {
      state.fb[index] = Number(value);
      fb.textContent = `fb ${Number(value).toFixed(3)}`;
    } else {
      fb.textContent = 'fb –';
    }
  }
  syncSlidersFromFeedback(side);
  updateSafetyUi(side);
}

function connectWebSocket() {
  clearTimeout(app.wsTimer);
  const protocol = location.protocol === 'https:' ? 'wss:' : 'ws:';
  const socket = new WebSocket(`${protocol}//${location.host}/ws`);
  app.ws = socket;

  socket.addEventListener('open', () => {
    statusEl.textContent = 'connected';
    statusEl.className = 'status connected';
  });
  socket.addEventListener('close', () => {
    statusEl.textContent = 'disconnected';
    statusEl.className = 'status disconnected';
    app.wsTimer = setTimeout(connectWebSocket, 1000);
  });
  socket.addEventListener('error', () => {
    statusEl.textContent = 'disconnected';
    statusEl.className = 'status disconnected';
  });
  socket.addEventListener('message', (event) => {
    let message;
    try {
      message = JSON.parse(event.data);
    } catch (_) {
      return;
    }
    if (message.type === 'state' && message.data) {
      updateMotionUi(message.motion);
      for (const side of app.config.sides) updateFeedback(side, message.data[side]);
    } else if (message.type === 'motion_result' && message.data) {
      const side = message.data.side;
      if (!message.data.ok && side) {
        showToast(`${side}: ${message.data.error || 'MoveIt motion failed'}`, 'error');
      }
    } else if (message.type === 'error') {
      showToast(message.message || 'command rejected', 'error');
    }
  });
}

function streamUrl(camera, mode) {
  return `/stream/${encodeURIComponent(camera)}/${encodeURIComponent(mode)}?t=${Date.now()}`;
}

function setupCameras() {
  const configured = new Map((app.config.cameras || []).map((camera) => [camera.name, camera]));
  for (const card of document.querySelectorAll('.camera')) {
    const name = card.dataset.cam;
    const meta = configured.get(name);
    const status = card.querySelector('.cam-status');
    const image = card.querySelector('img');
    const stats = card.querySelector('.cam-stats');
    const modeSwitch = card.querySelector('.mode-switch');
    modeSwitch.innerHTML = '';
    image.removeAttribute('src');

    if (!meta || !meta.modes?.length) {
      status.textContent = 'not configured';
      status.className = 'cam-status failed';
      stats.textContent = 'no ROS topic configured';
      app.cameras[name] = { meta, card, image, status, stats, mode: null };
      continue;
    }

    const preferred = meta.modes.includes('color') ? 'color' : meta.modes[0];
    const state = { meta, card, image, status, stats, mode: preferred, retryTimer: null };
    app.cameras[name] = state;

    for (const mode of meta.modes) {
      const button = document.createElement('button');
      button.textContent = mode;
      button.classList.toggle('selected', mode === preferred);
      button.addEventListener('click', () => {
        if (state.mode === mode) return;
        state.mode = mode;
        clearTimeout(state.retryTimer);
        for (const other of modeSwitch.querySelectorAll('button')) {
          other.classList.toggle('selected', other.textContent === mode);
        }
        startCameraStream(state);
      });
      modeSwitch.appendChild(button);
    }

    image.addEventListener('load', () => {
      status.textContent = 'live';
      status.className = 'cam-status live';
    });
    image.addEventListener('error', () => {
      status.textContent = 'offline';
      status.className = 'cam-status failed';
      stats.textContent = 'no frames · retrying';
      clearTimeout(state.retryTimer);
      state.retryTimer = setTimeout(() => startCameraStream(state), 2000);
    });

    startCameraStream(state);
  }
}

function startCameraStream(state) {
  if (!state.mode) return;
  const name = state.meta.name;
  state.status.textContent = 'connecting';
  state.status.className = 'cam-status pending';
  state.image.src = streamUrl(name, state.mode);
}

async function pollCameraStats() {
  while (true) {
    await new Promise((resolve) => setTimeout(resolve, 1000));
    try {
      const response = await fetch('/api/cameras', { cache: 'no-store' });
      const body = await response.json();
      for (const [name, state] of Object.entries(app.cameras)) {
        const camera = body.cameras?.[name];
        if (!camera || !state.mode) continue;
        const info = camera.streams?.[state.mode];
        if (!info || !info.seq) {
          state.stats.textContent = 'no frames · check ROS topic';
          continue;
        }
        const [height, width] = info.shape || [0, 0];
        const age = info.age_sec === null ? '–' : `${info.age_sec.toFixed(1)}s`;
        state.stats.textContent = `cap ${info.fps.toFixed(1)} fps · ${width}x${height} · age ${age}`;
        if (info.age_sec === null || info.age_sec > 1.5) {
          state.status.textContent = 'stale';
          state.status.className = 'cam-status failed';
        } else {
          state.status.textContent = 'live';
          state.status.className = 'cam-status live';
        }
      }
    } catch (_) {
      // The WebSocket status already reports whether the server is reachable.
    }
  }
}

function resizeViewport() {
  if (!app.renderer || !app.camera) return;
  const width = Math.max(1, viewport.clientWidth);
  const height = Math.max(1, viewport.clientHeight);
  app.renderer.setSize(width, height, false);
  app.camera.aspect = width / height;
  app.camera.updateProjectionMatrix();
}

async function init3d() {
  const THREE = await import('three');
  const { OrbitControls } = await import('three/addons/controls/OrbitControls.js');
  const { ColladaLoader } = await import('three/addons/loaders/ColladaLoader.js');
  const { STLLoader } = await import('three/addons/loaders/STLLoader.js');
  const URDFLoader = (await import('urdf-loader')).default;
  app.three = THREE;

  THREE.Object3D.DEFAULT_UP.set(0, 0, 1);
  const scene = new THREE.Scene();
  scene.background = new THREE.Color(0x35424e);
  app.scene = scene;

  const camera = new THREE.PerspectiveCamera(42, 1, 0.01, 50);
  camera.up.set(0, 0, 1);
  camera.position.set(-1.95, -1.55, 1.25);
  app.camera = camera;

  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
  renderer.outputColorSpace = THREE.SRGBColorSpace;
  viewport.appendChild(renderer.domElement);
  app.renderer = renderer;

  const controls = new OrbitControls(camera, renderer.domElement);
  controls.target.set(0, 0, 0.35);
  controls.minDistance = 0.7;
  controls.maxDistance = 6.0;
  controls.update();
  app.controls = controls;

  scene.add(new THREE.AmbientLight(0xffffff, 0.65));
  const key = new THREE.DirectionalLight(0xffffff, 1.0);
  key.position.set(-2, -2, 3);
  scene.add(key);
  const fill = new THREE.DirectionalLight(0xffffff, 0.45);
  fill.position.set(2, 2, 1.5);
  scene.add(fill);

  const grid = new THREE.GridHelper(3.6, 18, 0x7b8794, 0x53606d);
  grid.rotation.x = Math.PI / 2;
  grid.position.z = -0.002;
  scene.add(grid);
  scene.add(new THREE.AxesHelper(0.18));

  const loadingManager = new THREE.LoadingManager();
  const urdfLoader = new URDFLoader(loadingManager);
  urdfLoader.packages = {
    agx_arm_description: '/pkg/agx_arm_description',
  };
  urdfLoader.parseVisual = true;
  urdfLoader.parseCollision = false;
  urdfLoader.loadMeshCb = (path, manager, done) => {
    const extension = path.split('.').pop().toLowerCase();
    if (extension === 'dae') {
      new ColladaLoader(manager).load(
        path,
        (collada) => done(collada.scene),
        undefined,
        (error) => done(null, error),
      );
    } else if (extension === 'stl') {
      new STLLoader(manager).load(
        path,
        (geometry) => {
          const material = new THREE.MeshStandardMaterial({
            color: 0xa7adb4, roughness: 0.58, metalness: 0.15,
          });
          done(new THREE.Mesh(geometry, material));
        },
        undefined,
        (error) => done(null, error),
      );
    } else {
      done(null, new Error(`unsupported mesh type: ${extension}`));
    }
  };

  const robot = await new Promise((resolve, reject) => {
    urdfLoader.load('/robot_urdf', resolve, undefined, reject);
  });
  app.robot = robot;
  scene.add(robot);
  for (const side of app.config.sides) {
    applyArmPose(side);
    syncSlidersFromFeedback(side, true);
  }

  const animate = () => {
    requestAnimationFrame(animate);
    resizeViewport();
    controls.update();
    renderer.render(scene, camera);
  };
  resizeViewport();
  animate();
}

function tickClock() {
  const now = new Date();
  const pad = (value, width = 2) => String(value).padStart(width, '0');
  const element = document.getElementById('wallclock');
  element.textContent = `${pad(now.getHours())}:${pad(now.getMinutes())}:${pad(now.getSeconds())}.${pad(now.getMilliseconds(), 3)}`;
  requestAnimationFrame(tickClock);
}

async function boot() {
  tickClock();
  const response = await fetch('/api/config', { cache: 'no-store' });
  if (!response.ok) throw new Error(`config request failed: HTTP ${response.status}`);
  app.config = await response.json();
  app.bothSupported = (app.config.move_targets || []).includes('both');
  app.jointNames = app.config.joint_names;
  app.limits = app.config.limits;

  for (const side of app.config.sides) {
    app.sides[side] = {
      cmd: new Array(app.jointNames.length).fill(0),
      fb: new Array(app.jointNames.length).fill(null),
      rowEls: [],
      gate: Boolean(app.config.initial_gate_open),
      enabled: null,
      feedbackAge: null,
      poseInitialized: false,
      motionBusy: false,
      motionAction: null,
      health: null,
      healthBusy: null,
    };
    updateSafetyUi(side);
  }

  updateMotionUi(app.config.motion);
  buildSliders();
  setupCameras();
  connectWebSocket();
  pollCameraStats();

  try {
    await init3d();
  } catch (error) {
    console.error('3D view failed', error);
    showBanner('3D view unavailable (CDN or URDF load failed). Joint controls remain functional.');
  }
}

boot().catch((error) => {
  console.error('boot failed', error);
  statusEl.textContent = 'boot error';
  statusEl.className = 'status disconnected';
  showBanner(`Dashboard failed to start: ${error.message}`);
});
