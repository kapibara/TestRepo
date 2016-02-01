
#include <iostream>
#include <chrono>

#include "stdafx.h"
#include "kinect2videosource.h"
#include "bufferwrapper.h"

using namespace std::chrono;

bool Kinect2VideoSource::open()
{
    HRESULT hr;
    IColorFrameSource* colorFrameSource;
    IDepthFrameSource* depthFrameSource;
    IInfraredFrameSource* irFrameSource;
    running_ = true;

    hr = GetDefaultKinectSensor(&sensor_);
    running_ = running_ & SUCCEEDED(hr);

    hr = sensor_->Open();
    running_ = running_ & SUCCEEDED(hr);
    if(!running_){
        std::cerr << "could not open the sensor" << std::endl;
    }

    hr = sensor_->get_CoordinateMapper(&mapper_);
    running_ = running_ & SUCCEEDED(hr);
    if(!running_){
        std::cerr << "failed to obtain coordinate mapper" << std::endl;
    }


    hr = sensor_->get_ColorFrameSource(&colorFrameSource);
    running_ = running_ & SUCCEEDED(hr);
    if(!running_){
        std::cerr << "could not obtain color frame source" << std::endl;
    }

    hr = colorFrameSource->OpenReader(&color_);
    running_ = running_ & SUCCEEDED(hr);
    if(!running_){
        std::cerr << "could not open color reader" << std::endl;
    }

    hr = sensor_->get_DepthFrameSource(&depthFrameSource);
    running_ = running_ & SUCCEEDED(hr);
    if(!running_){
        std::cerr << "could not open depth frame source" << std::endl;
    }

    hr = depthFrameSource->OpenReader(&depth_);
    running_ = running_ & SUCCEEDED(hr);
    if(!running_){
        std::cerr << "could not open color reader" << std::endl;
    }

    hr = sensor_->get_InfraredFrameSource(&irFrameSource);
    running_ = running_ & SUCCEEDED(hr);
    if(!running_){
        std::cerr << "could not open infrared frame source" << std::endl;
    }

    hr = irFrameSource->OpenReader(&ir_);
    running_ = running_ & SUCCEEDED(hr);
    if(!running_){
        std::cerr << "could not open infrared reader" << std::endl;
    }

    IColorFrame* colorFrame = NULL;
    IDepthFrame* depthFrame = NULL;
    IInfraredFrame* irFrame = NULL;

    IFrameDescription* frameDesc =NULL;

    unsigned int nBufferSize;
    void *buffer;

    /*busy-wait for color frame --- no need to wait after the device is running it seems*/
    bool keep_trying = true;

    steady_clock::time_point t1 = steady_clock::now();
    steady_clock::time_point t2;

    while(keep_trying && (duration_cast<duration<double>>(t2 - t1)).count() < 10){

        hr = color_->AcquireLatestFrame(&colorFrame);

        keep_trying = keep_trying & !SUCCEEDED(hr);

        t2 = steady_clock::now();
    }

    running_ = running_ & SUCCEEDED(hr);

    colorFrame->AccessRawUnderlyingBuffer(&nBufferSize, (BYTE **)&buffer);
    std::cerr <<"color frame buffer size: " << nBufferSize << "size: "<<sizeof(BYTE) << std::endl;

    hr = colorFrame->get_FrameDescription(&frameDesc);
    running_ = running_ & SUCCEEDED(hr);

    setDesc(*frameDesc, colorSpec_);
    SafeRelease(colorFrame);
    SafeRelease(frameDesc);

    if(!running_){
        std::cerr << "could not obtain color frame width/height" << std::endl;
    }

    hr = depth_->AcquireLatestFrame(&depthFrame);
    running_ = running_ & SUCCEEDED(hr);

    depthFrame->AccessUnderlyingBuffer(&nBufferSize, (UINT16 **)&buffer);
    std::cerr <<"depth frame buffer size: " << nBufferSize << std::endl;

    hr = depthFrame->get_FrameDescription(&frameDesc);
    running_ = running_ & SUCCEEDED(hr);
    setDesc(*frameDesc, depthSpec_);
    SafeRelease(depthFrame);
    SafeRelease(frameDesc);


    if(!running_){
        std::cerr << "could not obtain depth frame width/height" << std::endl;
    }

    hr = ir_->AcquireLatestFrame(&irFrame);
    running_ = running_ & SUCCEEDED(hr);

    irFrame->AccessUnderlyingBuffer(&nBufferSize, (UINT16 **)&buffer);
    std::cerr <<"ir frame buffer size: " << nBufferSize << std::endl;

    hr = irFrame->get_FrameDescription(&frameDesc);
    running_ = running_ & SUCCEEDED(hr);
    setDesc(*frameDesc, irSpec_);
    SafeRelease(irFrame);
    SafeRelease(frameDesc);

    if(!running_){
        std::cerr << "could not obtain ir frame width/height" << std::endl;
    }

    /*have to be here - before sensor is running it returns uninitialized parameters*/
    hr = mapper_->GetDepthCameraIntrinsics(&dparam_);
    running_ = running_ & SUCCEEDED(hr);
    if(!running_){
        std::cerr << "could not open depth reader" << std::endl;
    }

    std::cerr << dparam_.PrincipalPointX << " vs " << depthSpec_.width_ <<
                 dparam_.PrincipalPointY << " vs " << depthSpec_.height_ << std::endl;

    points_ = new DepthSpacePoint[colorSpec_.width_*colorSpec_.height_];

    return running_;
}

/*that is blocking call!*/
bool Kinect2VideoSource::retrieve(unsigned char *buffer, ImageType imgType)
{
    bool success = true;
    HRESULT hr;
    IColorFrame* colorFrame = NULL;
    IDepthFrame* depthFrame = NULL;
    IInfraredFrame* irFrame = NULL;

    switch(imgType)
    {
        case IMAGE_RGB:
            hr = color_->AcquireLatestFrame(&colorFrame);
            success = success & SUCCEEDED(hr);
            if (!success){
                return false;
            }
            //muhaha, consistency and unified interfaces are everywhere
            hr = colorFrame->CopyConvertedFrameDataToArray(pixelSize(IMAGE_RGB)*colorSpec_.width_*colorSpec_.height_,buffer,ColorImageFormat_Rgba);
            success = success & SUCCEEDED(hr);
            SafeRelease(colorFrame);
            break;
        case IMAGE_DEPTH:
            hr = depth_->AcquireLatestFrame(&depthFrame);
            success = success & SUCCEEDED(hr);
            if (!success){
                return false;
            }
            hr = depthFrame->CopyFrameDataToArray(depthSpec_.width_*depthSpec_.height_,(UINT16 *)buffer);
            success = success & SUCCEEDED(hr);
            SafeRelease(depthFrame);
            break;
        case IMAGE_IR:
            hr = ir_->AcquireLatestFrame(&irFrame);
            success = success & SUCCEEDED(hr);
            if (!success){
                return false;
            }
            hr = irFrame->CopyFrameDataToArray(irSpec_.width_*irSpec_.height_,(UINT16 *)buffer);
            success = success & SUCCEEDED(hr);
            SafeRelease(irFrame);
            break;
        default:
            success = false;
    };

    return success;
}

int Kinect2VideoSource::pixelSize(ImageType type) const
{
    switch(type)
    {
    case IMAGE_RGB:
            return 4* sizeof(BYTE);
    case IMAGE_DEPTH:
            return 2* sizeof(BYTE);
    case IMAGE_IR:
            return 2* sizeof(BYTE);
    default:
        return 0;
    }
}

void Kinect2VideoSource::map(const unsigned char *depth, const unsigned char *color, pc &cloud) const
{
    mapper_->MapColorFrameToDepthSpace(depthSpec_.width_*depthSpec_.height_,
                                   (UINT16 *)depth,
                                   colorSpec_.width_*colorSpec_.height_,
                                   points_);

    BufferWrapper<unsigned short,1> dwrapper = BufferWrapper<unsigned short,1>((UINT16 *)depth,depthSpec_.width_,depthSpec_.height_);
    int cps = pixelSize(IMAGE_RGB);

    for(int i= 0; i< colorSpec_.width_*colorSpec_.height_; i++){
        if (points_[i].X != -std::numeric_limits<float>::infinity() && points_[i].Y != -std::numeric_limits<float>::infinity())
        {
            int depthX = (int)(floor(points_[i].X + 0.5));
            int depthY = (int)(floor(points_[i].Y + 0.5));
            if(depthX < 0 | depthX > depthSpec_.width_){
                std::cout << "x point out of range" << std::endl;
                continue;
            }
            if(depthY < 0 | depthY > depthSpec_.height_){
                std::cout << "y point out of range" << std::endl;
                 continue;
            }
            float depth =(float)dwrapper.at(depthY,depthX);
            if(depth == 0){
                //invalid depth
                continue;
            }
            std::array<float,3> rgb_p = {(float)*(color + cps*i),
                        (float)*(color + cps*i+1),
                        (float)*(color + cps*i+2)};

            std::array<float,3> d_p = {(float)(depthX - dparam_.PrincipalPointX)*depth/dparam_.FocalLengthX,
                           (float)(depthY - dparam_.PrincipalPointY)*depth/dparam_.FocalLengthY,
                           (float)depth};

            //for now we do not use calibration
            cloud.push_back(std::make_pair(d_p,rgb_p));
        }

    }
}

void Kinect2VideoSource::map2color(const unsigned char *depth, ColorSpacePoint *points) const
{
    mapper_->MapDepthFrameToColorSpace(depthSpec_.width_*depthSpec_.height_,
                                       (UINT16 *)depth,
                                        depthSpec_.width_*depthSpec_.height_,
                                        points);
}

void Kinect2VideoSource::map(const unsigned char *depth, const unsigned char *color, unsigned char *mapped) const
{
    mapper_->MapColorFrameToDepthSpace(depthSpec_.width_*depthSpec_.height_,
                                       (UINT16 *)depth,
                                       colorSpec_.width_*colorSpec_.height_,
                                       points_);

    BufferWrapper<unsigned char,3> wrapper = BufferWrapper<unsigned char,3>(mapped,depthSpec_.width_,depthSpec_.height_);
    int color_pixel_size = pixelSize(IMAGE_RGB);

    for(int i= 0; i< colorSpec_.width_*colorSpec_.height_; i++){
        if (points_[i].X != -std::numeric_limits<float>::infinity() && points_[i].Y != -std::numeric_limits<float>::infinity())
        {

            int depthX = (int)(floor(points_[i].X + 0.5));
            int depthY = (int)(floor(points_[i].Y + 0.5));
            if(depthX < 0 | depthX > depthSpec_.width_){
                std::cout << "x point out of range" << std::endl;
                continue;
            }
            if(depthY < 0 | depthY > depthSpec_.height_){
                std::cout << "y point out of range" << std::endl;
                 continue;
            }

            wrapper.at(depthY,depthX,0) = *(color + color_pixel_size*i);
            wrapper.at(depthY,depthX,1) = *(color + color_pixel_size*i+1);
            wrapper.at(depthY,depthX,2) = *(color + color_pixel_size*i+2);

        }
    }
}


int Kinect2VideoSource::expectedBufferSize(ImageType type) const
{
    switch(type)
    {
    case IMAGE_RGB:
            return colorSpec_.width_ * colorSpec_.height_ * pixelSize(type);
    case IMAGE_DEPTH:
            return depthSpec_.width_ * depthSpec_.height_ * pixelSize(type);
    case IMAGE_IR:
            return irSpec_.width_ * irSpec_.height_ * pixelSize(type);
    default:
        return 0;
    }
}

int Kinect2VideoSource::width(ImageType type) const
{
    switch(type)
    {
    case IMAGE_RGB:
            return colorSpec_.width_;
    case IMAGE_DEPTH:
            return  depthSpec_.width_;
    case IMAGE_IR:
            return irSpec_.width_;
    default:
        return -1;
    }
}

int Kinect2VideoSource::height(ImageType type) const
{
    switch(type)
    {
    case IMAGE_RGB:
            return colorSpec_.height_;
    case IMAGE_DEPTH:
            return  depthSpec_.height_;
    case IMAGE_IR:
            return irSpec_.height_;
    default:
        return -1;
    }
}


bool Kinect2VideoSource::isOpened() const
{
    return running_;
}

void Kinect2VideoSource::setDesc(IFrameDescription &desc, FrameSpec &spec){
    HRESULT hr;
    unsigned int val;
    desc.get_LengthInPixels(&val);
    std::cerr << "length in pixels: " <<val<< std::endl;
    desc.get_BytesPerPixel(&val);
     std::cerr << "bytes per pixel: " <<val<< std::endl;

    hr = desc.get_Width(&spec.width_);
    running_ = running_ & SUCCEEDED(hr);
    hr = desc.get_Height(&spec.height_);
    running_ = running_ & SUCCEEDED(hr);
}

bool Kinect2VideoSource::close()
{
    return false;
}
