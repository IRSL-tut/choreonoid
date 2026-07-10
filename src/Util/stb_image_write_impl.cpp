/**
   This file instantiates the implementation of the stb_image_write header-only
   library. The PNG writer is used for encoding the texture images embedded in
   glTF files output by GLTFSceneWriter.
*/

#define STB_IMAGE_WRITE_IMPLEMENTATION
#define STBI_WRITE_NO_STDIO
#include <stb/stb_image_write.h>
