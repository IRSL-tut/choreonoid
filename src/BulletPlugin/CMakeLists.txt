# Detect Bullet availability to decide the default value of BUILD_BULLET_PLUGIN.
# The user can still override this by specifying BUILD_BULLET_PLUGIN explicitly.
# On Ubuntu, the required package can be installed by the following command.
#   sudo apt install libbullet-dev
set(DEFAULT_BUILD_BULLET_PLUGIN OFF)
if(BULLET_DIR)
  set(DEFAULT_BUILD_BULLET_PLUGIN ON)
elseif(UNIX)
  pkg_check_modules(BULLET QUIET bullet)
  if(BULLET_FOUND)
    set(DEFAULT_BUILD_BULLET_PLUGIN ON)
  endif()
endif()
option(BUILD_BULLET_PLUGIN "Building BulletPlugin" ${DEFAULT_BUILD_BULLET_PLUGIN})

if(NOT BUILD_BULLET_PLUGIN)
  return()
endif()

set(BULLET_DIR ${BULLET_DIR} CACHE PATH "set the top directory of the Bullet Physics library")

if(BULLET_DIR)
  # Use a custom Bullet build installed in BULLET_DIR
  set(ENV{PKG_CONFIG_PATH} ${BULLET_DIR}/lib/pkgconfig)
  pkg_check_modules(BULLET bullet)
  if(NOT BULLET_FOUND)
    find_package(Bullet PATHS ${BULLET_DIR}/lib/cmake/bullet NO_DEFAULT_PATH)
    if(NOT BULLET_FOUND)
      message(FATAL_ERROR "Bullet was not found in BULLET_DIR (${BULLET_DIR}).")
    endif()
    set(BULLET_INCLUDE_DIRS ${BULLET_DIR}/${BULLET_INCLUDE_DIRS})
    set(BULLET_LIBRARY_DIRS ${BULLET_DIR}/${BULLET_LIBRARY_DIRS})
    set(BULLET_LIBRARIES BulletSoftBody BulletDynamics BulletCollision LinearMath)
    if(BULLET_USE_DOUBLE_PRECISION)
      set(BULLET_CFLAGS_OTHER -DBT_USE_DOUBLE_PRECISION)
    endif()
  endif()
elseif(UNIX)
  if(NOT BULLET_FOUND)
    pkg_check_modules(BULLET REQUIRED bullet)
  endif()
else()
  message(FATAL_ERROR "Please specify the directory of the Bullet Physics library to BULLET_DIR.")
endif()

include_directories(${BULLET_INCLUDE_DIRS})
link_directories(${BULLET_LIBRARY_DIRS})

set(target CnoidBulletPlugin)

set(sources
  BulletPlugin.cpp
  BulletSimulatorItem.cpp
  BtContinuousTrack.cpp
  BtContinuousTrackSimulator.cpp
  )

set(headers
  BulletPlugin.h
  BulletSimulatorItem.h
  exportdecl.h
  )

make_gettext_mofiles(${target} mofiles)
choreonoid_add_plugin(${target} ${sources} ${mofiles} HEADERS ${headers})
target_link_libraries(${target} PUBLIC CnoidBodyPlugin PRIVATE ${BULLET_LIBRARIES})
if(BULLET_CFLAGS_OTHER)
  target_compile_options(${target} PRIVATE ${BULLET_CFLAGS_OTHER})
endif()

if(ENABLE_PYTHON)
  choreonoid_add_python_binding_subdirectory()
endif()
