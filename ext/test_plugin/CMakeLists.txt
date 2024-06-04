cmake_minimum_required(VERSION 3.10)

if(${CMAKE_CURRENT_SOURCE_DIR} STREQUAL ${CMAKE_SOURCE_DIR})
  # Build as a master project
  Message ("Build : TestPlugin / separated from choreonoid/ext")
  project(TestPlugin)
  find_package(Choreonoid REQUIRED)

  add_subdirectory(src/plugin)
else ()
  # Build at choreonoid/ext
  add_subdirectory(src)
endif()
