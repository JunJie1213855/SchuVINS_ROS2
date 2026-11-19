# Fetch source-only dependencies at fixed revisions. Their original Catkin
# build files are deliberately not used. Override FETCHCONTENT_SOURCE_DIR_*
# for offline builds (see README_ROS2.md).
if(POLICY CMP0169)
  cmake_policy(SET CMP0169 OLD)
endif()
include(FetchContent)
FetchContent_Declare(minkindr
  GIT_REPOSITORY https://github.com/ethz-asl/minkindr.git
  GIT_TAG 564f12639a8447d4d3e5e7707851424302941056)
FetchContent_Declare(fast
  GIT_REPOSITORY https://github.com/uzh-rpg/fast.git
  GIT_TAG 1153981a7d0263909e998789a5b5029412c28f5a)
FetchContent_Declare(opengv_src
  GIT_REPOSITORY https://github.com/laurentkneip/opengv.git
  GIT_TAG 91f4b19c73450833a40e463ad3648aae80b3a7f3)
foreach(dep minkindr fast opengv_src)
  FetchContent_GetProperties(${dep})
  if(NOT ${dep}_POPULATED)
    FetchContent_Populate(${dep})
  endif()
endforeach()
add_library(schurvins_fast STATIC
  ${fast_SOURCE_DIR}/src/fast_10.cpp
  ${fast_SOURCE_DIR}/src/fast_10_score.cpp
  ${fast_SOURCE_DIR}/src/nonmax_3x3.cpp)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64|i.86")
  target_sources(schurvins_fast PRIVATE ${fast_SOURCE_DIR}/src/faster_corner_10_sse.cpp)
  target_compile_options(schurvins_fast PRIVATE -msse2)
endif()
set_target_properties(schurvins_fast PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(schurvins_fast PUBLIC ${fast_SOURCE_DIR}/include PRIVATE ${fast_SOURCE_DIR}/src)
# Source list from the pinned OpenGV revision; exclude its legacy build rules.
set(opengv_sources
  ${opengv_src_SOURCE_DIR}/src/absolute_pose/modules/main.cpp
  ${opengv_src_SOURCE_DIR}/src/absolute_pose/modules/gp3p/code.cpp
  ${opengv_src_SOURCE_DIR}/src/absolute_pose/modules/gp3p/init.cpp
  ${opengv_src_SOURCE_DIR}/src/absolute_pose/modules/gp3p/reductors.cpp
  ${opengv_src_SOURCE_DIR}/src/absolute_pose/modules/gp3p/spolynomials.cpp
  ${opengv_src_SOURCE_DIR}/src/absolute_pose/modules/Epnp.cpp
  ${opengv_src_SOURCE_DIR}/src/absolute_pose/modules/gpnp1/code.cpp
  ${opengv_src_SOURCE_DIR}/src/absolute_pose/modules/gpnp1/init.cpp
  ${opengv_src_SOURCE_DIR}/src/absolute_pose/modules/gpnp1/reductors.cpp
  ${opengv_src_SOURCE_DIR}/src/absolute_pose/modules/gpnp1/spolynomials.cpp
  ${opengv_src_SOURCE_DIR}/src/absolute_pose/modules/gpnp2/code.cpp
  ${opengv_src_SOURCE_DIR}/src/absolute_pose/modules/gpnp2/init.cpp
  ${opengv_src_SOURCE_DIR}/src/absolute_pose/modules/gpnp2/reductors.cpp
  ${opengv_src_SOURCE_DIR}/src/absolute_pose/modules/gpnp2/spolynomials.cpp
  ${opengv_src_SOURCE_DIR}/src/absolute_pose/modules/gpnp3/code.cpp
  ${opengv_src_SOURCE_DIR}/src/absolute_pose/modules/gpnp3/init.cpp
  ${opengv_src_SOURCE_DIR}/src/absolute_pose/modules/gpnp3/reductors.cpp
  ${opengv_src_SOURCE_DIR}/src/absolute_pose/modules/gpnp3/spolynomials.cpp
  ${opengv_src_SOURCE_DIR}/src/absolute_pose/modules/gpnp4/code.cpp
  ${opengv_src_SOURCE_DIR}/src/absolute_pose/modules/gpnp4/init.cpp
  ${opengv_src_SOURCE_DIR}/src/absolute_pose/modules/gpnp4/reductors.cpp
  ${opengv_src_SOURCE_DIR}/src/absolute_pose/modules/gpnp4/spolynomials.cpp
  ${opengv_src_SOURCE_DIR}/src/absolute_pose/modules/gpnp5/code.cpp
  ${opengv_src_SOURCE_DIR}/src/absolute_pose/modules/gpnp5/init.cpp
  ${opengv_src_SOURCE_DIR}/src/absolute_pose/modules/gpnp5/reductors.cpp
  ${opengv_src_SOURCE_DIR}/src/absolute_pose/modules/gpnp5/spolynomials.cpp
  ${opengv_src_SOURCE_DIR}/src/absolute_pose/modules/upnp2.cpp
  ${opengv_src_SOURCE_DIR}/src/absolute_pose/modules/upnp4.cpp
  ${opengv_src_SOURCE_DIR}/src/relative_pose/modules/main.cpp
  ${opengv_src_SOURCE_DIR}/src/relative_pose/modules/fivept_nister/modules.cpp
  ${opengv_src_SOURCE_DIR}/src/relative_pose/modules/fivept_stewenius/modules.cpp
  ${opengv_src_SOURCE_DIR}/src/relative_pose/modules/fivept_kneip/code.cpp
  ${opengv_src_SOURCE_DIR}/src/relative_pose/modules/fivept_kneip/init.cpp
  ${opengv_src_SOURCE_DIR}/src/relative_pose/modules/fivept_kneip/reductors.cpp
  ${opengv_src_SOURCE_DIR}/src/relative_pose/modules/fivept_kneip/spolynomials.cpp
  ${opengv_src_SOURCE_DIR}/src/relative_pose/modules/sixpt/modules2.cpp
  ${opengv_src_SOURCE_DIR}/src/relative_pose/modules/eigensolver/modules.cpp
  ${opengv_src_SOURCE_DIR}/src/relative_pose/modules/ge/modules.cpp
  ${opengv_src_SOURCE_DIR}/src/math/cayley.cpp
  ${opengv_src_SOURCE_DIR}/src/math/quaternion.cpp
  ${opengv_src_SOURCE_DIR}/src/math/arun.cpp
  ${opengv_src_SOURCE_DIR}/src/math/Sturm.cpp
  ${opengv_src_SOURCE_DIR}/src/math/roots.cpp
  ${opengv_src_SOURCE_DIR}/src/math/gauss_jordan.cpp
  ${opengv_src_SOURCE_DIR}/src/absolute_pose/methods.cpp
  ${opengv_src_SOURCE_DIR}/src/absolute_pose/CentralAbsoluteAdapter.cpp
  ${opengv_src_SOURCE_DIR}/src/absolute_pose/NoncentralAbsoluteAdapter.cpp
  ${opengv_src_SOURCE_DIR}/src/absolute_pose/NoncentralAbsoluteMultiAdapter.cpp
  ${opengv_src_SOURCE_DIR}/src/relative_pose/methods.cpp
  ${opengv_src_SOURCE_DIR}/src/relative_pose/CentralRelativeAdapter.cpp
  ${opengv_src_SOURCE_DIR}/src/relative_pose/CentralRelativeWeightingAdapter.cpp
  ${opengv_src_SOURCE_DIR}/src/relative_pose/NoncentralRelativeAdapter.cpp
  ${opengv_src_SOURCE_DIR}/src/relative_pose/CentralRelativeMultiAdapter.cpp
  ${opengv_src_SOURCE_DIR}/src/relative_pose/NoncentralRelativeMultiAdapter.cpp
  ${opengv_src_SOURCE_DIR}/src/triangulation/methods.cpp
  ${opengv_src_SOURCE_DIR}/src/point_cloud/methods.cpp
  ${opengv_src_SOURCE_DIR}/src/point_cloud/PointCloudAdapter.cpp
  ${opengv_src_SOURCE_DIR}/src/sac_problems/absolute_pose/AbsolutePoseSacProblem.cpp
  ${opengv_src_SOURCE_DIR}/src/sac_problems/absolute_pose/MultiNoncentralAbsolutePoseSacProblem.cpp
  ${opengv_src_SOURCE_DIR}/src/sac_problems/relative_pose/CentralRelativePoseSacProblem.cpp
  ${opengv_src_SOURCE_DIR}/src/sac_problems/relative_pose/NoncentralRelativePoseSacProblem.cpp
  ${opengv_src_SOURCE_DIR}/src/sac_problems/relative_pose/RotationOnlySacProblem.cpp
  ${opengv_src_SOURCE_DIR}/src/sac_problems/relative_pose/TranslationOnlySacProblem.cpp
  ${opengv_src_SOURCE_DIR}/src/sac_problems/relative_pose/EigensolverSacProblem.cpp
  ${opengv_src_SOURCE_DIR}/src/sac_problems/relative_pose/MultiCentralRelativePoseSacProblem.cpp
  ${opengv_src_SOURCE_DIR}/src/sac_problems/relative_pose/MultiNoncentralRelativePoseSacProblem.cpp
  ${opengv_src_SOURCE_DIR}/src/sac_problems/point_cloud/PointCloudSacProblem.cpp
  ${opengv_src_SOURCE_DIR}/src/absolute_pose/MACentralAbsolute.cpp
  ${opengv_src_SOURCE_DIR}/src/absolute_pose/MANoncentralAbsolute.cpp
  ${opengv_src_SOURCE_DIR}/src/relative_pose/MACentralRelative.cpp
  ${opengv_src_SOURCE_DIR}/src/relative_pose/MANoncentralRelative.cpp
  ${opengv_src_SOURCE_DIR}/src/relative_pose/MANoncentralRelativeMulti.cpp
  ${opengv_src_SOURCE_DIR}/src/point_cloud/MAPointCloud.cpp
)
add_library(opengv STATIC ${opengv_sources})
set_target_properties(opengv PROPERTIES POSITION_INDEPENDENT_CODE ON)
target_include_directories(opengv PUBLIC ${opengv_src_SOURCE_DIR}/include ${EIGEN3_INCLUDE_DIR}/unsupported)
target_link_libraries(opengv PUBLIC Eigen3::Eigen)
