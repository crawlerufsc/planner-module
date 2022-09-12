#include <stdio.h>
#include <string>
#include <vector>
#include <string>
#include <thread>
#include <chrono>
#include <opencv2/opencv.hpp>
#include "dataset_stream_reader.h"
#include "../utils/image_utils.h"

void DataStreamReader::delayFrameRate()
{
    if (frameRate <= 0)
        return;

    // fprintf (stdout, "sleeping for %d miliseconds\n",(1000/frameRate));
    std::this_thread::sleep_for(std::chrono::milliseconds(1000 / frameRate));
}

DataStreamReader::DataStreamReader(uint32_t width, uint32_t height, uint32_t frameRate)
{
    this->input = new std::vector<std::string>();
    this->width = width;
    this->height = height;
    this->pos = 0;
    this->repeat_for = 1;
    this->repeat_for_pos = 0;
    this->frameRate = frameRate;
}

DataStreamReader::~DataStreamReader()
{
    delete input;
}

DataStreamReader *DataStreamReader::addSource(std::string path)
{
    if (FileUtils::fileExists(path))
        input->push_back(path);

    return this;
}

uint32_t DataStreamReader::GetWidth()
{
    return width;
}

uint32_t DataStreamReader::GetHeight()
{
    return height;
}
uint32_t DataStreamReader::GetFrameRate()
{
    return frameRate;
}
DataStreamReader *DataStreamReader::repeatFrame(uint32_t times)
{
    this->repeat_for = times;
    return this;
}

bool DataStreamReader::initialize()
{
    return true;
}
StreamData * DataStreamReader::onRequestNextFrame()
{
    if (pos >= input->size())
    {
        pos = 0;
    }

    std::string filename = input->at(pos);

    StreamData *result = new StreamData();
    //std::cout << "reading: " << filename << "\n";

    FileData *p = FileUtils::readFile(filename);

    std::vector<char> imgv(p->data, p->data + p->length);
    cv::Mat img = cv::imdecode(cv::Mat(imgv), 1);

    result->data = img.data;
    result->width = img.cols;
    result->height = img.rows;
    result->raw_length = p->length;
    //std::cout << "success\n";

    this->repeat_for_pos++;

    if (this->repeat_for_pos >= this->repeat_for)
    {
        this->repeat_for_pos = 0;
        this->pos++;
    }

    delayFrameRate();

    FileData *p = FileUtils::readFile(filename);
    return ImageUtils::decodeImageData(p->data, p->length);
}

void DataStreamReader::onTerminate()
{
}

