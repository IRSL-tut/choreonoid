option(
  BUILD_PIECEWISE_RIGID_CONTINUOUS_TRACK
  "Building the piecewise-rigid continuous track library"
  ON)
if(NOT BUILD_PIECEWISE_RIGID_CONTINUOUS_TRACK)
  return()
endif()

set(target CnoidPiecewiseRigidContinuousTrack)

set(headers
  PiecewiseRigidContinuousTrack.h
  exportdecl.h
  )

file(MAKE_DIRECTORY ${PROJECT_BINARY_DIR}/include/cnoid)
choreonoid_make_header_public(PiecewiseRigidContinuousTrack.h)

make_gettext_mofiles(${target} mofiles)
choreonoid_add_library(${target} SHARED
  PiecewiseRigidContinuousTrack.cpp
  ${mofiles}
  HEADERS ${headers})
target_link_libraries(${target} PUBLIC CnoidBody)
