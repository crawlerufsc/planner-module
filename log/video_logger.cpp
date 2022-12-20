#include "video_logger.h"
#include "../model/global_definitions.h"
#include <resource_manager.h>
#include <network_stream_logger.h>

VideoLogger::VideoLogger(std::string resourceName, const char *topic) : PubSubClient(BROKER_IP, BROKER_PORT, topic)
{
    this->resourceName = resourceName;
}

VideoLogger::~VideoLogger()
{
    stop();
}

void VideoLogger::onReceived(std::string topic, std::string payload)
{
    if (payload == "start")
        start();
    else
        stop();
};
void VideoLogger::onStop()
{
    stop();
};

void VideoLogger::start()
{
    if (logger != nullptr)
        return;

    logger = new NetworkStreamLogger(
        "/home/cristiano/original_vision_output.mkv",
        BROKER_IP,
        BROKER_PORT,
        PLANNER_IP,
        LOCALPORT_STREAM_LOGGER_ORIGINAL);
    
    logger->withStreamRequestUri(PUBSUB_STREAM_REQUEST_URI_ORIGINAL);

    printf("Start logging original image to file: ~/original_vision_output.mkv\n");
    logger->requestStreamStart();
}
void VideoLogger::stop()
{
    if (logger == nullptr)
        return;
    printf("Stop logging original image\n");
    logger->requestStreamStop();
    delete logger;
    logger = nullptr;
}