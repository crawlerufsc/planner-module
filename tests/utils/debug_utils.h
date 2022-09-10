#ifndef _IMG_DEBUG_UTILS
#define _IMG_DEBUG_UTILS

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

class ImageDebugUtils
{
private:
    bool initWindow;

    std::string type2str(int type)
    {
        std::string r;

        uchar depth = type & CV_MAT_DEPTH_MASK;
        uchar chans = 1 + (type >> CV_CN_SHIFT);

        switch (depth)
        {
        case CV_8U:
            r = "8U";
            break;
        case CV_8S:
            r = "8S";
            break;
        case CV_16U:
            r = "16U";
            break;
        case CV_16S:
            r = "16S";
            break;
        case CV_32S:
            r = "32S";
            break;
        case CV_32F:
            r = "32F";
            break;
        case CV_64F:
            r = "64F";
            break;
        default:
            r = "User";
            break;
        }

        r += "C";
        r += (chans + '0');

        return r;
    }

    void describeMatrix(cv::Mat &mat)
    {
        std::string typestr = type2str(mat.type());
        printf("Matrix: %s %dx%d \n", typestr.c_str(), mat.cols, mat.rows);
    }

public:
    ImageDebugUtils()
    {
        initWindow = false;
    }

    void showImage(const char *title, char *data, size_t length)
    {
        if (data == nullptr)
            return;

        std::vector<char> imgv(data, data + length);
        cv::Mat img = cv::imdecode(cv::Mat(imgv), 1);
        describeMatrix(img);

        if (!initWindow)
        {
            cv::namedWindow(title);
        }

        initWindow = true;
        cv::imshow(title, img);
        cv::waitKey(5);
    }
};

#endif