/**
   This file instantiates the implementation of the stb_image header-only library.
   Only the HDR (.hdr) loader is enabled because the other image formats are
   handled by libpng / libjpeg / libtiff in ImageIO.cpp.
*/

#define STB_IMAGE_IMPLEMENTATION
#define STBI_ONLY_HDR
#include <stb/stb_image.h>
