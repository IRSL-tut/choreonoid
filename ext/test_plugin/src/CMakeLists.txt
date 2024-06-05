set(target CnoidTestPlugin)

### only for linux
set(sources
  TestPlugin.cpp
  )
set(headers
  TestPlugin.h
  )
if (NOT WIN32)
  add_definitions(-Wno-deprecated-declarations)
endif()
#add_definitions(-DIRSL_DEBUG)

#set(OPENVR_DIR "c:/testbuild")
#find_library(OPENVR_LIBRARIES
#  NAMES
#    openvr_api64
#  PATHS
#    ${OPENVR_DIR}/bin
#    ${OPENVR_DIR}/lib
#  PATH_SUFFIXES
#    win${PLATFORM}
#  NO_DEFAULT_PATH
#  NO_CMAKE_FIND_ROOT_PATH
#)
#set(OPENVR_INCLUDE_DIR ${OPENVR_DIR}/include/openvr)
set(OPENVR_DIR "c:/Users/leus/src/openvr")
find_library(OPENVR_LIBRARIES
  NAMES
    openvr_api
  PATHS
    ${OPENVR_DIR}/bin
    ${OPENVR_DIR}/lib
  PATH_SUFFIXES
    win64
  NO_DEFAULT_PATH
  NO_CMAKE_FIND_ROOT_PATH
)
set(OPENVR_INCLUDE_DIR ${OPENVR_DIR}/headers)

set(OPENVR_FOUND TRUE)

if(${CMAKE_CURRENT_SOURCE_DIR} STREQUAL ${CMAKE_SOURCE_DIR})
  # Build as a master project
  add_definitions(-DEXT_BUNDLE) ## TODO : write node/choreonoid_jupyter.cpp
  set(CMAKE_CXX_STANDARD ${CHOREONOID_CXX_STANDARD})
  choreonoid_add_plugin(${target} ${sources} HEADERS ${headers})
  find_library(LIB_PYTHON_PLUGIN CnoidPythonPlugin HINTS ${CHOREONOID_PLUGIN_DIR})
  Message ("plugin: ${CHOREONOID_PLUGIN_DIR}, lib: ${LIB_PYTHON_PLUGIN}")
  target_link_libraries(${target} Choreonoid::CnoidUtil ${LIB_PYTHON_PLUGIN} ${xeuslib} Threads::Threads)
  ## install
  install(FILES ../../share/jupyter/kernels/choreonoid/kernel.json DESTINATION ${CHOREONOID_SHARE_SUBDIR}/jupyter/kernels/choreonoid)
else()
  # Build as a bundled project
  Message ("Build : TestPlugin / under choreonoid/ext")
  ## build
  ##add_definitions(-DEXT_BUNDLE)
  add_definitions(-D_WIN32)
  set(CMAKE_CXX_FLAGS_DEBUG   "${CMAKE_CXX_FLAGS_DEBUG} /W2 /DEBUG")
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MP /INCREMENTAL:NO")
  choreonoid_add_plugin(${target} ${sources} HEADERS ${headers})
  target_link_libraries(${target} PUBLIC CnoidUtil CnoidGLSceneRenderer ${OPENVR_LIBRARIES})
  Message ("OpenVR: ${OPENVR_LIBRARIES}")
  include_directories(${OPENVR_INCLUDE_DIR})
endif()
