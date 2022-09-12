#ifndef _IMAGE_UTILS_H
#define _IMAGE_UTILS_H

#include <opencv2/opencv.hpp>
#include <vector>
#include "../model/stream_data.h"

class ImageUtils
{
public:
    static cv::Mat *convertToOpenCVMatrix(StreamData *data)
    {
        return new cv::Mat(data->height, data->width, data->mattype, data->data);
    }

    static StreamData *decodeImageData(char *buffer, int length)
    {
        std::vector<char> imgv(buffer, buffer + length);
        cv::Mat img = cv::imdecode(cv::Mat(imgv), 1);

        StreamData *result = new StreamData();
        result->data = img.data;
        result->width = img.cols;
        result->height = img.rows;
        result->raw_length = length;
        result->mattype = img.type();
        return result;
    }
};

#endif