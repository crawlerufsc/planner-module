#include "video_logger.h"
#include "../model/global_definitions.h"


VideoLogger::VideoLogger() : PubSubClient(BROKER_IP, BROKER_PORT)
{
    original = nullptr;
    segmented = nullptr;
    og = nullptr;
    isInitialized = false;
}
VideoLogger::~VideoLogger()
{
    stopLogOriginal();
    stopLogSegmented();
    stopLogOccupancyGrid();
}

bool VideoLogger::initialize()
{
    if (isInitialized) return true;

    if (!blockUntilConnected(2000))
        return false;

    subscribeTo(PUBSUB_API_ORIGINAL_STREAM_LOGGING_REQUEST_URI);
    subscribeTo(PUBSUB_API_SEGMENTED_STREAM_LOGGING_REQUEST_URI);
    subscribeTo(PUBSUB_API_OCCUPANCYGRID_STREAM_LOGGING_REQUEST_URI);

    isInitialized = true;
    return isInitialized;
}

void VideoLogger::onReceived(std::string topic, std::string payload)
{
    if (topic == PUBSUB_API_ORIGINAL_STREAM_LOGGING_REQUEST_URI)
    {
        requestStreamOriginal(payload == "start");
    }
    else if (topic == PUBSUB_API_SEGMENTED_STREAM_LOGGING_REQUEST_URI)
    {
        requestStreamSegmented(payload == "start");
    }
    else if (topic == PUBSUB_API_OCCUPANCYGRID_STREAM_LOGGING_REQUEST_URI)
    {
        requestStreamOccupancyGrid(payload == "start");
    }
};

void VideoLogger::startLogOriginal()
{
    if (original != nullptr)
        stopLogOriginal();

    original = new NetworkStreamLogger(
        FILE_LOG_STREAM_ORIGINAL,
        BROKER_IP,
        BROKER_PORT,
        PLANNER_IP,
        LOCALPORT_STREAM_LOGGER_ORIGINAL);

    original->withStreamUri(PUBSUB_STREAM_REQUEST_URI_ORIGINAL, PUBSUB_STREAM_RESPONSE_URI);
    original->build();
    original->requestStreamStart();
}
void VideoLogger::startLogSegmented()
{
    if (segmented != nullptr)
        stopLogSegmented();

    segmented = new NetworkStreamLogger(
        FILE_LOG_STREAM_SEGMENTED,
        BROKER_IP,
        BROKER_PORT,
        PLANNER_IP,
        LOCALPORT_STREAM_LOGGER_SEGMENTED);

    segmented->withStreamUri(PUBSUB_STREAM_REQUEST_URI_SEGMENTED, PUBSUB_STREAM_RESPONSE_URI);
    segmented->build();
    segmented->requestStreamStart();
}
void VideoLogger::startLogOccupancyGrid()
{
    if (og != nullptr)
        stopLogOccupancyGrid();

    og = new NetworkStreamLogger(
        FILE_LOG_STREAM_OG,
        BROKER_IP,
        BROKER_PORT,
        PLANNER_IP,
        LOCALPORT_STREAM_LOGGER_OCCUPANCYGRID);

    og->withStreamUri(PUBSUB_STREAM_REQUEST_URI_OCCUPANCYGRID, PUBSUB_STREAM_RESPONSE_URI);
    og->build();
    og->requestStreamStart();
}

void VideoLogger::stopLogOriginal()
{
    if (original == nullptr)
        return;

    original->requestStreamStop();
    delete original;
    original = nullptr;
}
void VideoLogger::stopLogSegmented()
{
    if (segmented == nullptr)
        return;

    segmented->requestStreamStop();
    delete segmented;
    segmented = nullptr;
}
void VideoLogger::stopLogOccupancyGrid()
{
    if (og == nullptr)
        return;

    og->requestStreamStop();
    delete og;
    og = nullptr;
}

void VideoLogger::requestStreamOriginal(bool start)
{
    if (start)
        startLogOriginal();
    else
        stopLogOriginal();
}
void VideoLogger::requestStreamSegmented(bool start)
{
    if (start)
        startLogSegmented();
    else
        stopLogSegmented();
}
void VideoLogger::requestStreamOccupancyGrid(bool start)
{
    if (start)
        startLogOccupancyGrid();
    else
        stopLogOccupancyGrid();
}

void VideoLogger::onStop()
{
    stopLogOriginal();
    stopLogSegmented();
    stopLogOccupancyGrid();
};
