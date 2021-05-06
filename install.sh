#!/usr/bin/env bash

echo "update everything"
sudo apt-get update

echo "installing dependencies"
sudo apt-get install -y python3-dev python3-pip python3-numpy
sudo apt-get install -y python3-tk
sudo apt-get install -y libglew-dev
sudo apt-get install -y build-essential cmake pkg-config
sudo apt-get install -y libxi-dev libxmu-dev freeglut3-dev
sudo apt-get install -y python3-venv
sudo apt-get install -y qtbase5-dev

echo "installing c libaries"

export PROJECTDIR=$(pwd)
export ENVDIR=$PROJECTDIR/c_env

mkdir -p $ENVDIR
mkdir -p $ENVDIR/include
mkdir -p $ENVDIR/lib
mkdir -p $ENVDIR/lib/cmake
mkdir -p $ENVDIR/src

install_library() {
    cd $ENVDIR/src
    git clone --depth 1 --branch $3 $2
    echo "==== Installing $1 at $ENVDIR ===="
    mkdir $1/build
    cd $1/build
    if [ -f ../CMakeLists.txt ]; then
        cmake -DCMAKE_BUILD_TYPE=Release \
              -DCMAKE_PREFIX_PATH=$ENVDIR \
              -DCMAKE_INSTALL_PREFIX=$ENVDIR \
              -DCMAKE_INSTALL_RPATH_USE_LINK_PATH=TRUE \
              -DCMAKE_INSTALL_RPATH=$ENVDIR \
              $4 \
              ..
    elif [ -f Makefile ]; then
        cd ..
    fi

    make -j8 install
}
install_boost() {
	cd $ENVDIR/src
  wget https://dl.bintray.com/boostorg/release/1.69.0/source/boost_1_69_0.tar.gz
  tar -xzf ./boost_1_69_0.tar.gz
  rm ./boost_1_69_0.tar.gz
  cd ./boost_1_69_0
	# git clone --branch boost-1.69.0 https://github.com/boostorg/boost.git
	# cd $ENVDIR/src/boost
 #  git submodule update --init --depth 1 --recursive
	./bootstrap.sh --with-python=python3 --with-libraries=atomic,chrono,filesystem,python,system,regex,program_options
	./b2 --with-python --with-filesystem --with-regex --with-system --with-program_options --prefix=$ENVDIR install
}

install_library tinyxml2 https://github.com/leethomason/tinyxml2 8.0.0
install_library eigen https://gitlab.com/libeigen/eigen 3.3.7
install_library libccd https://github.com/danfis/libccd v2.0
install_library assimp https://github.com/assimp/assimp v4.0.1
install_library octomap https://github.com/OctoMap/octomap v1.8.1
install_library fcl https://github.com/flexible-collision-library/fcl 0.6.1
install_library bullet3 https://github.com/bulletphysics/bullet3 2.89 \
    "-DBUILD_SHARED_LIBS=ON -DCMAKE_POSITION_INDEPENDENT_CODE=ON"
install_boost
install_library dart-ltspd https://github.com/snumrl/dart-ltspd master \
"-DDART_ENABLE_SIMD=ON -DFCL_INCLUDE_DIRS=$ENVDIR/include/fcl -DBULLET_INCLUDE_DIRS=$ENVDIR/include/bullet"


cd $PROJECTDIR

echo "installing python env"

python3 -m venv py_env
. ./py_env/bin/activate
pip3 install --upgrade pip
pip3 install -r requirements.txt
