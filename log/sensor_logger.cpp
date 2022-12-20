#include "sensor_logger.h"
#include <resource_manager.h>
#include <crawler_hal.h>
#include "../model/global_definitions.h"

#define SENSOR_LOGGER 111

SensorLogger::SensorLogger(const char *filename) : PubSubClient(BROKER_IP, BROKER_PORT, PUBSUB_API_SENSOR_LOGGING_REQUEST_URI)
{
    is_logging = false;
    imuSensorCallback = std::function<void(IMUData *)>([this](IMUData *p) { //
        printf("logging: %s\n", p->toJson().c_str());
    });
    gpsSensorCallback = std::function<void(GPSData *)>([this](GPSData *p) { //
        printf("logging: %s\n", p->toJson().c_str());
    });
}

void SensorLogger::start()
{
    if (is_logging)
        return;
    CrawlerHAL *hal = CrawlerHAL::getInstance();
    hal->addIMUCallbackHandler(SENSOR_LOGGER, imuSensorCallback);
    hal->addGPSCallbackHandler(SENSOR_LOGGER, gpsSensorCallback);
    is_logging = true;
}
void SensorLogger::stop()
{
    if (!is_logging)
        return;
    CrawlerHAL *hal = CrawlerHAL::getInstance();
    hal->removeIMUCallbackHandler(SENSOR_LOGGER);
    hal->removeGPSCallbackHandler(SENSOR_LOGGER);
    is_logging = false;
}

void SensorLogger::onReceived(std::string topic, std::string payload)
{
    if (payload == "start")
    {
        start();
    }
    else
    {
        stop();
    }
};
void SensorLogger::onStop()
{
    stop();
};
