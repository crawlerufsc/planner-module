
// https://youtu.be/yggi7QjfOUM?t=864
// https://www.geeksforgeeks.org/multithreading-in-cpp/
// https://stackoverflow.com/questions/266168/simple-example-of-threading-in-c
// https://www.rabbitmq.com/tutorials/tutorial-one-dotnet.html

// Include atomic std library
#include <atomic>

// Include gstreamer library
#include <gst/gst.h>
#include <gst/app/app.h>

// Include OpenCV library
#include <opencv2/opencv.hpp>
#include <unistd.h>
#include <stdio.h>
#include <sstream>
#include "../../communication/dataset_stream_reader.h"
#include "../../model/stream_data.h"
#include "../../planning/ompl_planner.h"

#include <execinfo.h>
#include <signal.h>
#include <stdlib.h>

DataStreamReader *buildInputStreamReader()
{
    DataStreamReader *reader = new DataStreamReader(160, 200, 120);

    for (int i = 1; i < 291; i++)
    {
        std::stringstream ss;
        ss << "/home/cristiano/Documents/Projects/Mestrado/Project/dataset/18/og_images/occupancy_grid_output";

        if (i < 10)
            ss << "0";
        if (i < 100)
            ss << "0";

        ss << i << ".png";

        reader->addSource(ss.str());
    }

    return reader;
}

void handler(int sig)
{
    void *array[10];
    size_t size;

    // get void*'s for all entries on the stack
    size = backtrace(array, 10);

    // print out all the frames to stderr
    fprintf(stderr, "Error: signal %d:\n", sig);
    backtrace_symbols_fd(array, size, STDERR_FILENO);
    exit(1);
}

OMPLPlanner *planner;

static void onProcess(DirectProcessPipeline<StreamData> *reader, StreamData *data)
{
    planner->plan(data);
}

int main(int argc, char *argv[])
{
    signal(SIGSEGV, handler);
    planner = OMPLPlanner::GetInstance(160, 200, 5, 1, 40);

    DataStreamReader* reader = buildInputStreamReader();

    reader->withOnProcessCallback(onProcess)
        ->run();

    OMPLPlanner::ClearInstance();
    return 0;
}
