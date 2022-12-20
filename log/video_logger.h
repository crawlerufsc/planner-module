#ifndef _VIDEO_LOGGER_H
#define _VIDEO_LOGGER_H

#include <functional>
#include <pubsub_client.h>
#include <string>
#include <network_stream_logger.h>

class VideoLogger : PubSubClient
{
private:
    NetworkStreamLogger * logger;
    std::string resourceName;
    void start();
    void stop();

protected:
    void onReceived(std::string topic, std::string payload) override;
    void onStop() override;

public:
    VideoLogger(std::string resourceName, const char *topic);
    ~VideoLogger();

};

#endif