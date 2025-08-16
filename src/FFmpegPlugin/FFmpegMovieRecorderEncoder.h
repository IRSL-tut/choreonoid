#ifndef CNOID_FFMPEG_PLUGIN_FFMPEG_MOVIE_RECORDER_ENCODER_H
#define CNOID_FFMPEG_PLUGIN_FFMPEG_MOVIE_RECORDER_ENCODER_H

#include <cnoid/MovieRecorder>

namespace cnoid {

class FFmpegMovieRecorderEncoder : public MovieRecorderEncoder
{
public:
    virtual std::string formatName() const override;
    virtual bool initializeEncoding(int width, int height, int frameRate) override;
    virtual bool doEncoding(std::string fileBasename) override;

private:
    class Impl;

    int width;
    int height;
    int frameRate;
};

}

#endif
