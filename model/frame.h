#ifndef _STREAM_DATA_H
#define _STREAM_DATA_H

#include <memory>
#include <opencv2/opencv.hpp>

template <typename FrameType>
class Frame
{
private:
    cv::Mat *_mat;
    static int convertMatrixToVectorPos(int width, int heightPos, int widthPos)
    {
        return (width * heightPos) + widthPos;
    }

public:
    FrameType *data;
    int width;
    int height;
    int imgType;
    long seq;

    void clear()
    {
        memset(data, 0, sizeof(FrameType) * (width * height));
    }

    void print()
    {
        for (int i = 0; i < height; i++)
        {
            for (int j = 0; j < width; j++)
                printf("%d", data[i * width + j]);
            printf("\n");
        }
    }

    static Frame *newEmptyFrame(int width, int height)
    {
        FrameType *data = new FrameType[width * height];
        Frame *f = new Frame(width, height, data);

        f->clear();
        return f;
    }

    Frame(int width, int height, FrameType *data, int imgType = 0)
    {
        this->width = width;
        this->height = height;
        this->data = data;
        this->imgType = imgType;
        this->_mat = nullptr;
    }

    ~Frame()
    {
        if (_mat != nullptr)
        {
            _mat->release();
            delete _mat;
        }
        if (data != nullptr)
        {
            // std::cout << "dealloc Frame data on " << reinterpret_cast<void *>(this) << std::endl;
            delete[] data;
        }
    }

    cv::Mat *getMatrix(int type)
    {
        if (this->_mat == nullptr)
            this->_mat = new cv::Mat(height, width, type, this->data);
        return this->_mat;
    }

    const FrameType operator()(int h, int w) const
    {
        if (w < 0 || w > width)
            throw std::invalid_argument("invalid index width for Frame data");

        if (h < 0 || h > height)
            throw std::invalid_argument("invalid index height for Frame data");

        int pos = convertMatrixToVectorPos(width, h, w);

        return data[pos];
    }

    FrameType get(int h, int w)
    {
        int pos = convertMatrixToVectorPos(width, h, w);
        return data[pos];
    }

    void set(int h, int w, FrameType value)
    {
        if (w < 0 || w > width)
            throw std::invalid_argument("invalid index width for Frame data");

        if (h < 0 || h > height)
            throw std::invalid_argument("invalid index height for Frame data");

        int pos = convertMatrixToVectorPos(width, h, w);

        data[pos] = value;
    }

    void copyFrom(Frame<FrameType> *target)
    {
        for (int i = 0; i < width * height; i++)
            this->data[i] = target->data[i];
    }
};

typedef Frame<unsigned char> StreamData;

#endif
