'use strict'

const assert = require('assert')
const m = require('..')
const fallback = require('../lib/fallback')

const isNative = !m.__fallback
console.log(`mode: ${isNative ? 'native' : 'JS fallback'}`)

assert.strictEqual(m.sinf(0), 0)
assert.strictEqual(m.cosf(0), 1)

assert.strictEqual(m.constants.DEG_TO_RAD, 0.01745329238474369)
assert.strictEqual(m.constants.BDS_EPSILON, 1.1920928955078125e-7)

{
  const r = m.moveRelative(0, 0, 0, 0, 0, 0, 0.98, 0.1)
  assert.strictEqual(r.x, 0)
  assert.strictEqual(r.y, 0)
  assert.ok(Math.abs(r.z - 0.098) < 1e-6, `vel.z = ${r.z}`)
}

assert.strictEqual(m.applyFriction(1e-8, 0.5), 0)
assert.strictEqual(m.applyFriction(0.1, 0.5), m.fround(0.05))

assert.strictEqual(m.fround(0.1), 0.10000000149011612)
assert.strictEqual(m.fround(0.98), 0.9800000190734863)

if (isNative) {
  assert.strictEqual(m.sinf(0), fallback.sinf(0))
  assert.strictEqual(m.cosf(0), fallback.cosf(0))
  assert.strictEqual(m.constants.DEG_TO_RAD, fallback.constants.DEG_TO_RAD)
  const a = m.moveRelative(0, -0.0784, 0, 0, 0, 0, 0.98, 0.1)
  const b = fallback.moveRelative(0, -0.0784, 0, 0, 0, 0, 0.98, 0.1)
  assert.ok(Math.abs(a.z - b.z) < 1e-6, `native vs fallback z mismatch: ${a.z} vs ${b.z}`)
}

console.log('all assertions passed')
