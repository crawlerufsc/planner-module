#ifndef _DATA_STREAM_READER_H
#define _DATA_STREAM_READER_H


#include "../utils/file_utils.h"
#include "../model/stream_data.h"
#include "../control/direct_process_pipeline.h"

class DataStreamReader : public DirectProcessPipeline<StreamData>
{
private:
    std::vector<std::string> *input;
    uint32_t width;
    uint32_t height;
    uint32_t frameRate;
    uint32_t repeat_for;
    uint32_t repeat_for_pos;
    int pos;

    void delayFrameRate();

public:
    DataStreamReader(uint32_t width, uint32_t height, uint32_t frameRate);
    ~DataStreamReader();

    DataStreamReader *addSource(std::string path);
    uint32_t GetWidth();
    uint32_t GetHeight();
    uint32_t GetFrameRate();
    DataStreamReader *repeatFrame(uint32_t times);

    bool initialize() override;
    StreamData * onRequestNextFrame() override;
    void onTerminate() override;
};

#endif