#!/bin/sh
# ブラウザのデモ（docs/）用の WASM をビルドする。検出器は隣の vehicle_det_cpp（VEHICLE_DET で変えられる）。
#   sh wasm/build.sh            → docs/tj.js, docs/tj.wasm
set -e
cd "$(dirname "$0")/.."
EMSDK="${EMSDK:-/c/prog/emsdk/emsdk}"
VD="${VEHICLE_DET:-../vehicle_det_cpp}"
EMCC="$EMSDK/upstream/emscripten/emcc.exe"
[ -x "$EMCC" ] || EMCC="$EMSDK/upstream/emscripten/emcc"
"$EMCC" -std=c++20 -O3 -msimd128 -DNDEBUG -DUSE_EIGEN -I. -I"$VD" -I"$VD/pure" -I"$VD/pure/third_party" -I"$VD/pure/third_party/eigen_flat" \
  -s MODULARIZE=1 -s EXPORT_NAME=createTj -s ALLOW_MEMORY_GROWTH=1 -s INITIAL_MEMORY=268435456 -s STACK_SIZE=16777216 \
  -s EXPORTED_RUNTIME_METHODS='["HEAPU8","UTF8ToString","stringToUTF8","lengthBytesUTF8"]' -s EXPORTED_FUNCTIONS='["_malloc","_free"]' \
  wasm/tj_wasm.cpp -o docs/tj.js
echo "built docs/tj.js"
