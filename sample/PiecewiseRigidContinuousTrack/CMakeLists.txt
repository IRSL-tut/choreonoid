if(NOT BUILD_PIECEWISE_RIGID_CONTINUOUS_TRACK OR
   (NOT BUILD_PHYSX_PLUGIN AND NOT BUILD_BULLET_PLUGIN))
  return()
endif()

install_project_files(
  PRCTrackVehicle.cnoid
  PRCTrackTank.cnoid
  )
