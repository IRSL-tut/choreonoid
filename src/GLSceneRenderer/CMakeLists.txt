set(sources
  GLSLProgram.cpp
  ShaderPrograms.cpp
  GLSceneRenderer.cpp
  CnoidGLSceneRenderer.cpp # This file must be placed at the last position
  )

set(headers
  GLSLProgram.h
  ShaderPrograms.h
  GLSceneRenderer.h
  exportdecl.h
  )

set(target CnoidGLSceneRenderer)

choreonoid_qt_add_resources(RC_SRCS GLSceneRenderer.qrc)

make_gettext_mofiles(${target} mofiles)

set(libraries PUBLIC CnoidUtil PRIVATE ${CHOREONOID_QT_CORE_LIBRARY} CnoidOpenGL)

if(CNOID_ENABLE_FREE_TYPE)
  set(sources ${sources} GLFreeType.cpp)
  if(MSVC)
    set(libraries ${libraries} PRIVATE Freetype::Freetype)
  else()
    set(libraries ${libraries} PRIVATE PkgConfig::FreeType2)
  endif()
endif()

choreonoid_add_library(${target} SHARED ${sources} ${mofiles} ${RC_SRCS} HEADERS ${headers})
target_link_libraries(${target} ${libraries})
