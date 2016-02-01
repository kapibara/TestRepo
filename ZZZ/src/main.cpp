
#include "kinect2videosource.h"

#include <iostream>
#include <chrono>
#include <string>
#include <fstream>

using namespace std::chrono;

void load_bin(std::ifstream &file,  unsigned char *buffer )
{

    int width=0,height=0;
    file.read((char *)&width,sizeof(int));
    file.read((char *)&height,sizeof(int));

    std::cout << "reading: " <<  width << ";" << height << std::endl;

    file.read((char *)buffer,sizeof(short)*width*height);
}

void save_points(std::ostream &file,  ColorSpacePoint *points, int points_count )
{
    file.write((const char *)&points_count,sizeof(points_count));
    file.write((const char *)points,sizeof(ColorSpacePoint)*points_count);
}

void main(int argc, char **argv)
{
    Kinect2VideoSource kinect2;
    std::cout << "device is open successfully: " << kinect2.open() << std::endl;
    std::cout << "color buffer: " <<kinect2.expectedBufferSize(Kinect2VideoSource::ImageType::IMAGE_RGB) << std::endl;
    std::cout << "depth buffer: " <<kinect2.expectedBufferSize(Kinect2VideoSource::ImageType::IMAGE_DEPTH) << std::endl;
    int points_number = kinect2.width(Kinect2VideoSource::IMAGE_DEPTH)*kinect2.height(Kinect2VideoSource::IMAGE_DEPTH);
    ColorSpacePoint *points = new ColorSpacePoint[points_number];
    unsigned char *buffer = new unsigned char[kinect2.expectedBufferSize(Kinect2VideoSource::ImageType::IMAGE_DEPTH)];
    for(int i=1; i<61; i++){
        std::string path = "D:\\Data\\Kinect2_new_withIR\\r" + std::to_string(i);

        {
            std::ifstream input(path+ "\\ImDepthOrig1.bin",std::ios::binary);

            std::cout << path+ "\\ImDepthOrig1.bin" << ":" << input.is_open() << std::endl;


            load_bin(input, buffer);
            kinect2.map2color(buffer, points);
            input.close();
            std::ofstream output("D:\\Data\\Kinect2_new_withIR\\maps\\map"+std::to_string(i)+".bin",std::ios::binary);
            std::cout << "points_number" << points_number << std::endl;
            save_points(output, points, points_number);
            output.close();
        }
    }


}
