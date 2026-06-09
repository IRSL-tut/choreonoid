#ifndef CNOID_UTIL_IMAGE_H
#define CNOID_UTIL_IMAGE_H

#include "NullOut.h"
#include <string>
#include <vector>
#include <cassert>
#include "exportdecl.h"

namespace cnoid {

class CNOID_EXPORT Image
{
public:
    /**
       Pixel data type of an image.
       UInt8 is the conventional 8bit/component format. Float32 is for HDR images
       (e.g. Radiance .hdr) that store 32bit floating-point values per component.
    */
    enum PixelType { UInt8, Float32 };

    Image();
    Image(const Image& org);
    virtual ~Image();

    Image& operator=(const Image& rhs);

    void reset();
    bool empty() const { return pixels_.empty(); }

    /**
       The raw byte buffer of the pixel data.
       Note that this returns the head of the raw bytes regardless of the pixel type.
       Clients that assume 8bit components must check pixelType() == UInt8.
    */
    unsigned char* pixels() { return &pixels_.front(); }
    const unsigned char* pixels() const { return &pixels_.front(); }

    //! The pixel data interpreted as 32bit floating-point values (valid only when pixelType() == Float32).
    float* floatPixels() {
        assert(pixelType_ == Float32);
        return reinterpret_cast<float*>(pixels_.data());
    }
    const float* floatPixels() const {
        assert(pixelType_ == Float32);
        return reinterpret_cast<const float*>(pixels_.data());
    }

    int width() const { return width_; }
    int height() const { return height_; }
    int numComponents() const { return numComponents_; }
    bool hasAlphaComponent() const { return (numComponents() % 2) == 0; }

    PixelType pixelType() const { return pixelType_; }
    //! The byte size of a single component (1 for UInt8, 4 for Float32).
    int bytesPerComponent() const { return (pixelType_ == Float32) ? 4 : 1; }

    void setSize(int width, int height, int nComponents);
    void setSize(int width, int height, int nComponents, PixelType pixelType);
    void setSize(int width, int height);

    void clear();
    void applyVerticalFlip();

    bool load(const std::string& filename, std::ostream& os = nullout());
    bool save(const std::string& filename, std::ostream& os = nullout()) const;

private:
    std::vector<unsigned char> pixels_;
    int width_;
    int height_;
    int numComponents_;
    PixelType pixelType_;
};

}

#endif
