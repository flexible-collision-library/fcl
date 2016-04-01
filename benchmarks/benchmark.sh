#!/bin/bash

# Launch this file at the source tree, with 3 build folders:
# - build-eigen: uses Eigen for math operations
#   cmake -DFCL_USE_EIGEN=ON  -DFCL_USE_SSE=OFF
# - build-orig: uses the native FCL math classes
#   cmake -DFCL_USE_EIGEN=OFF -DFCL_USE_SSE=OFF
# - build-sse: uses the native FCL SSE math classes
#   cmake -DFCL_USE_EIGEN=OFF -DFCL_USE_SSE=ON

run_tests () {
# To be run in build folder
  for t in "broadphase" "distance" "octomap" "geometric_shapes"; do
    echo "--- Start test $t ---"
    ./test/test_fcl_${t}
    echo "--- End test $t ---"
  done
}

for d in "build-eigen" "build-orig" "build-sse"; do
  echo "======= Start $d =======";
  cd $d
  run_tests > "benchmark.${d}.log"
  cd ..
  echo "======= End $d ======="
done
