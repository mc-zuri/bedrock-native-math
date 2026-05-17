'use strict'

let impl

try {
  impl = require('./build/Release/bedrock_native_math.node')
} catch (err) {
  impl = require('./lib/fallback')
  impl.__fallback = true
  impl.__loadError = err && err.message ? err.message : String(err)
}

module.exports = impl
