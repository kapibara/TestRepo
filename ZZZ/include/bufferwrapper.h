#ifndef BUFFERWRAPPER
#define BUFFERWRAPPER

#include <vector>
#include <array>

typedef  std::pair<std::array<float,3>,std::array<float,3>> pc_point;
typedef  std::vector<pc_point> pc;

template<typename T, int N>
class BufferWrapper
{

public:
    BufferWrapper(): channels_(N)
    {
        buffer_ = nullptr;
        width_ = 0;
        height_ = 0;
    }

    BufferWrapper(T *buffer, int width, int height): channels_(N)
    {
        buffer_ = buffer;
        width_ = width;
        height_ = height;
    }

    BufferWrapper(const BufferWrapper<T,N> &other): channels_(N){
        buffer_ = other.buffer_;
        width_ = other.width_;
        height_ = other.height_;
    }

    BufferWrapper<T,N> & operator=(const BufferWrapper<T,N> &other){
        buffer_ = other.buffer_;
        width_ = other.width_;
        height_ = other.height_;
        return *this;
    }

    T &at(int row, int col, int channel = 0){
        return buffer_[(row*width_ + col)*N + channel];
    }

    const T &at(int row, int col, int channel = 0) const{
        return buffer_[(row*width_ + col)*N + channel];
    }

    unsigned char channels() const {return channels_;}

    int width() const {return width_;}
    int height() const {return height_;}

private:
    T *buffer_;
    int width_;
    int height_;
    const  unsigned char channels_;
};

#endif
