/**
   This file instantiates the implementation of the stb_image header-only library.
   The HDR (.hdr) loader is used for loading HDR image files, and the PNG / JPEG
   loaders are used for loading images from the image file data on memory, such as
   the images embedded in glTF files. Loading the other image format files is
   handled by libpng / libjpeg / libtiff in ImageIO.cpp.
*/

#define STB_IMAGE_IMPLEMENTATION
#define STBI_ONLY_HDR
#define STBI_ONLY_PNG
#define STBI_ONLY_JPEG
#include <stb/stb_image.h>
