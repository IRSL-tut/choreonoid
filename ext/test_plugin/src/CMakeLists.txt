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
  choreonoid_add_plugin(${target} ${sources} HEADERS ${headers})
  target_link_libraries(${target} PUBLIC CnoidUtil )
endif()
