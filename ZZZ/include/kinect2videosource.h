#ifndef KINECT2VIDEOSOURCE
#define KINECT2VIDEOSOURCE

#include "Kinect.h"
#include "bufferwrapper.h"

#include <vector>
#include <array>
#include <iostream>

class Kinect2VideoSource
{
public:

    enum ImageType{
        IMAGE_RGB = 0x01,
        IMAGE_DEPTH = 0x02,
        IMAGE_IR = 0x03
    };

    struct FrameSpec{
        int width_;
        int height_;
    };

    Kinect2VideoSource(){
        sensor_ =NULL;
        color_ =NULL;
        depth_ = NULL;
    }

    bool open();
    bool isOpened() const;
    bool close();

    bool retrieve(unsigned char *buffer, ImageType imgType);

    void map(const unsigned char *depth, const unsigned char *color, unsigned char *mapped) const;
    void map2color(const unsigned char *depth, ColorSpacePoint *points) const;
    void map(const unsigned char *depth, const unsigned char *color, pc &cloud) const;

    CameraIntrinsics getDepthIntrinsics()
    {
        return dparam_;
    }

    int width(ImageType type) const;
    int height(ImageType type) const;

    int pixelSize(ImageType type) const;
    int expectedBufferSize(ImageType type) const;

private:

    void setDesc(IFrameDescription &desc, FrameSpec &spec);

    IKinectSensor *sensor_;
    IColorFrameReader *color_;
    IDepthFrameReader *depth_;
    ICoordinateMapper *mapper_;
    IInfraredFrameReader *ir_;

    DepthSpacePoint *points_;
    CameraIntrinsics dparam_;

    FrameSpec colorSpec_, depthSpec_, irSpec_;

    bool running_;
};

#endif
