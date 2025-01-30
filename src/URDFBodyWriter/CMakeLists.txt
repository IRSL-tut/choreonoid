if(NOT ENABLE_URDF)
  return()
endif()

include_directories(${PUGIXML_STATIC_INCLUDE_DIRS})
if(PUGIXML_STATIC_LIBRARY_DIRS)
  link_directories(${PUGIXML_STATIC_LIBRARY_DIRS})
endif()

if ( ${CHOREONOID_VERSION_STRING} VERSION_LESS "2.1.0" )
  add_definitions(-DUSE_OLD_OPTION)
endif()

choreonoid_add_library(CnoidURDFBodyWriter SHARED URDFBodyWriter.cpp HEADERS URDFBodyWriter.h)
target_link_libraries(CnoidURDFBodyWriter CnoidBody ${PUGIXML_STATIC_LIBRARIES} CnoidAssimpSceneWriter)

choreonoid_add_executable(choreonoid_body2urdf body2urdf.cpp)
target_link_libraries(choreonoid_body2urdf CnoidBody CnoidURDFBodyWriter ${Boost_PROGRAM_OPTIONS_LIBRARY})
