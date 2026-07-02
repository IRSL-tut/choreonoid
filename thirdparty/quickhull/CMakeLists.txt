# QuickHull static library.
# The library is linked into other Choreonoid shared libraries, so it must
# be compiled with -fPIC. Symbols are hidden so QuickHull's classes do not
# get exported through the linking shared library's ABI.

add_library(quickhull STATIC
  QuickHull.cpp
)

target_include_directories(quickhull PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
)

set_target_properties(quickhull PROPERTIES
  POSITION_INDEPENDENT_CODE ON
  CXX_VISIBILITY_PRESET hidden
  VISIBILITY_INLINES_HIDDEN ON
  FOLDER "Thirdparty"
)
