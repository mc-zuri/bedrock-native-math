'use strict'

const f = Math.fround

const constants = {
  DEG_TO_RAD:  f(Math.PI / 180),
  BDS_EPSILON: f(1.1920928955078125e-7),
  GROUND_FRIC: f(0.6 * 0.91),
  AIR_FRIC_XZ: f(0.91),
  Y_DECAY:     f(0.98),
  GRAVITY:     f(0.08),
  LMV_DAMP:    f(0.98)
}

function sinf (deg) {
  return f(Math.sin(f(f(deg) * constants.DEG_TO_RAD)))
}

function cosf (deg) {
  return f(Math.cos(f(f(deg) * constants.DEG_TO_RAD)))
}

function sincosf (deg) {
  const angle = f(f(deg) * constants.DEG_TO_RAD)
  return { sin: f(Math.sin(angle)), cos: f(Math.cos(angle)) }
}

function moveRelative (vx, vy, vz, yawDeg, strafe, up, forward, speed) {
  let _vx = f(vx)
  let _vy = f(vy)
  let _vz = f(vz)
  const _s = f(strafe)
  const _u = f(up)
  const _fwd = f(forward)
  const _sp = f(speed)

  const distSq = f(f(_s * _s) + f(_u * _u + _fwd * _fwd))
  if (distSq < f(0.000099999997)) return { x: _vx, y: _vy, z: _vz }

  let dist = f(Math.sqrt(distSq))
  if (dist < 1) dist = 1
  const scale = f(_sp / dist)

  const angle = f(f(yawDeg) * constants.DEG_TO_RAD)
  const s = f(Math.sin(angle))
  const c = f(Math.cos(angle))

  const strafeS = f(_s * scale)
  const forwardS = f(_fwd * scale)

  _vy = f(_vy + f(_u * scale))
  _vx = f(_vx + f(f(strafeS * c) - f(forwardS * s)))
  _vz = f(_vz + f(f(forwardS * c) + f(strafeS * s)))
  return { x: _vx, y: _vy, z: _vz }
}

function applyFriction (v, k) {
  const _v = f(v)
  return Math.abs(_v) > constants.BDS_EPSILON ? f(_v * f(k)) : 0
}

function fround (v) {
  return f(v)
}

module.exports = {
  sinf,
  cosf,
  sincosf,
  moveRelative,
  applyFriction,
  fround,
  constants
}
