#ifndef _VIDEO_LOGGER_H
#define _VIDEO_LOGGER_H

#include <pubsub_client.h>
#include <string>
#include <network_stream_logger.h>

class VideoLogger : PubSubClient
{
private:
    NetworkStreamLogger *original;
    NetworkStreamLogger *segmented;
    NetworkStreamLogger *og;
    bool isInitialized;

    void startLogOriginal();
    void startLogSegmented();
    void startLogOccupancyGrid();
    void stopLogOriginal();
    void stopLogSegmented();
    void stopLogOccupancyGrid();
    void requestStreamOriginal(bool);
    void requestStreamSegmented(bool);
    void requestStreamOccupancyGrid(bool);

protected:
    void onReceived(std::string topic, std::string payload) override;
    void onStop() override;

public:
    VideoLogger();
    ~VideoLogger();
    bool initialize();
};

#endif