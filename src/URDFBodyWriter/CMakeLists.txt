if(NOT ENABLE_URDF)
  return()
endif()

include_directories(${PUGIXML_STATIC_INCLUDE_DIRS})
if(PUGIXML_STATIC_LIBRARY_DIRS)
  link_directories(${PUGIXML_STATIC_LIBRARY_DIRS})
endif()

choreonoid_add_library(CnoidURDFBodyWriter SHARED URDFBodyWriter.cpp HEADERS URDFBodyWriter.h)
target_link_libraries(CnoidURDFBodyWriter CnoidBody ${PUGIXML_STATIC_LIBRARIES} CnoidAssimpSceneWriter)

add_executable(testxml test_xml.cpp)
target_link_libraries(testxml ${PUGIXML_STATIC_LIBRARIES})

add_executable(test_assimp test_assimp.cpp)
target_link_libraries(test_assimp CnoidBody CnoidAssimpSceneWriter)

add_executable(body2urdf body2urdf.cpp)
target_link_libraries(body2urdf CnoidBody CnoidURDFBodyWriter ${Boost_PROGRAM_OPTIONS_LIBRARY})
