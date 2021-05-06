cmake_minimum_required(VERSION 2.8.6)
project(PM)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING
      "Choose the type of build, options are: Debug Release
RelWithDebInfo MinSizeRel."
      FORCE)
endif(NOT CMAKE_BUILD_TYPE)

set(CMAKE_CXX_STANDARD 14)

add_compile_options(-fPIC)
add_compile_options(-std=gnu++17)
add_compile_options(-Wdeprecated-declarations)
add_compile_options(-lm)

add_definitions(-DPM_DIR="${CMAKE_HOME_DIRECTORY}")
set(CMAKE_MODULE_PATH "${CMAKE_HOME_DIRECTORY}/cmake_module")

find_package(DART REQUIRED COMPONENTS gui collision-bullet CONFIG)	
find_package(Boost 1.69 EXACT COMPONENTS filesystem python36 numpy36 program_options)
IF(NOT Boost_FOUND)
	find_package(Boost 1.69 EXACT COMPONENTS filesystem python38 numpy38 program_options)
ENDIF()

find_package(PythonLibs 3.6 REQUIRED)
find_package(PythonInterp 3.6 REQUIRED)
find_package(PythonLibs REQUIRED)
find_package(TinyXML2 REQUIRED)

add_subdirectory( sim )
add_subdirectory( network )
add_subdirectory( render )
