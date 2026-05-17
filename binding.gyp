{
  "targets": [
    {
      "target_name": "bedrock_native_math",
      "sources": [ "src/bedrock_native_math.cpp" ],
      "include_dirs": [
        "<!@(node -p \"require('node-addon-api').include\")"
      ],
      "defines": [ "NAPI_DISABLE_CPP_EXCEPTIONS" ],
      "cflags!":    [ "-fno-exceptions" ],
      "cflags_cc!": [ "-fno-exceptions" ],
      "cflags_cc":  [ "-ffast-math" ],
      "msvs_settings": {
        "VCCLCompilerTool": {
          "FloatingPointModel": "2",
          "Optimization": "2",
          "RuntimeLibrary": "0",
          "ExceptionHandling": "0"
        }
      },
      "xcode_settings": {
        "GCC_ENABLE_CPP_EXCEPTIONS": "NO",
        "OTHER_CFLAGS": [ "-ffast-math" ]
      }
    }
  ]
}
