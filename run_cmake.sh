#!/usr/bin/env bash
export CENVDIR=$(pwd)/c_env

mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_PREFIX_PATH=$CENVDIR \
      -DFCL_INCLUDE_DIRS=$CENVDIR/include/fcl \
      -DFCL_LIBRARIES=$CENVDIR/lib/libfcl.so \
      -DCCD_INCLUDE_DIRS=$CENVDIR/include/ccd \
      -DCCD_LIBRARIES=$CENVDIR/lib/libccd.so \
      ..
cd ..