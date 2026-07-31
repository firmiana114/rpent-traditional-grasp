cmake_minimum_required(VERSION 3.20)
project(rpent_g1_trac_ik LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(
  RPENT_DEPS_ROOT
  ""
  CACHE PATH
  "Optional root containing locally extracted Debian development packages"
)

if(RPENT_DEPS_ROOT)
  find_package(
    Eigen3
    REQUIRED
    CONFIG
    NO_DEFAULT_PATH
    PATHS "${RPENT_DEPS_ROOT}/usr/share/eigen3/cmake"
  )
  find_path(
    OROCOS_KDL_INCLUDE_DIR
    kdl/chain.hpp
    REQUIRED
    NO_DEFAULT_PATH
    PATHS "${RPENT_DEPS_ROOT}/usr/include"
  )
  find_library(
    OROCOS_KDL_LIBRARY
    NAMES orocos-kdl
    REQUIRED
    NO_DEFAULT_PATH
    PATHS
      "${RPENT_DEPS_ROOT}/usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}"
      "${RPENT_DEPS_ROOT}/usr/lib"
  )
  find_path(
    NLOPT_INCLUDE_DIR
    nlopt.hpp
    REQUIRED
    NO_DEFAULT_PATH
    PATHS "${RPENT_DEPS_ROOT}/usr/include"
  )
  find_library(
    NLOPT_CXX_LIBRARY
    NAMES nlopt_cxx
    REQUIRED
    NO_DEFAULT_PATH
    PATHS
      "${RPENT_DEPS_ROOT}/usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}"
      "${RPENT_DEPS_ROOT}/usr/lib"
  )
  find_library(
    NLOPT_C_LIBRARY
    NAMES nlopt
    REQUIRED
    NO_DEFAULT_PATH
    PATHS
      "${RPENT_DEPS_ROOT}/usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}"
      "${RPENT_DEPS_ROOT}/usr/lib"
  )
else()
  find_package(Eigen3 REQUIRED)
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(OROCOS_KDL REQUIRED orocos-kdl)
  find_path(NLOPT_INCLUDE_DIR nlopt.hpp REQUIRED)
  find_library(NLOPT_CXX_LIBRARY NAMES nlopt_cxx nlopt REQUIRED)
endif()

add_executable(
  g1_trac_ik
  src/g1_trac_ik.cpp
  vendor/trac_ik/src/kdl_tl.cpp
  vendor/trac_ik/src/nlopt_ik.cpp
  vendor/trac_ik/src/trac_ik.cpp
)
target_compile_definitions(g1_trac_ik PRIVATE TRAC_IK_STANDALONE)
target_include_directories(
  g1_trac_ik
  PRIVATE
    include
    vendor/trac_ik/include
)
target_include_directories(
  g1_trac_ik
  SYSTEM
  PRIVATE
    ${OROCOS_KDL_INCLUDE_DIRS}
    ${OROCOS_KDL_INCLUDE_DIR}
    ${NLOPT_INCLUDE_DIR}
)
target_compile_options(g1_trac_ik PRIVATE -Wall -Wextra -Wpedantic)
target_link_directories(g1_trac_ik PRIVATE ${OROCOS_KDL_LIBRARY_DIRS})
target_link_libraries(
  g1_trac_ik
  PRIVATE
    Eigen3::Eigen
    ${OROCOS_KDL_LIBRARIES}
    ${OROCOS_KDL_LIBRARY}
    ${NLOPT_CXX_LIBRARY}
    ${NLOPT_C_LIBRARY}
    pthread
)
if(RPENT_DEPS_ROOT)
  set(
    RPENT_LOCAL_LIBRARY_DIR
    "${RPENT_DEPS_ROOT}/usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}"
  )
  set_target_properties(
    g1_trac_ik
    PROPERTIES
      BUILD_RPATH "${RPENT_LOCAL_LIBRARY_DIR}"
      INSTALL_RPATH "\$ORIGIN/../.deps/usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}"
  )
endif()

enable_testing()
add_test(
  NAME g1_left_trac_ik
  COMMAND
    g1_trac_ik
    ${CMAKE_CURRENT_SOURCE_DIR}/../robot/chains/g1_left_arm.chain
    --self-test
)
add_test(
  NAME g1_right_trac_ik
  COMMAND
    g1_trac_ik
    ${CMAKE_CURRENT_SOURCE_DIR}/../robot/chains/g1_right_arm.chain
    --self-test
)
