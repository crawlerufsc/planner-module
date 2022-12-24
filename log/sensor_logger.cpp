#include "sensor_logger.h"
#include <resource_manager.h>
#include <crawler_hal.h>
#include <sys/time.h>

#include "../model/global_definitions.h"

#define SENSOR_LOGGER 111

SensorLogger::SensorLogger(std::string filename) : PubSubClient(BROKER_IP, BROKER_PORT, PUBSUB_API_SENSOR_LOGGING_REQUEST_URI)
{
    is_logging = false;
    this->filename = filename;
    imuSensorCallback = std::function<void(IMUData *)>([this](IMUData *p) { //
        appendToFile(SENSOR_IMU, p->toJson());
    });
    gpsSensorCallback = std::function<void(GPSData *)>([this](GPSData *p) { //
        appendToFile(SENSOR_GPS, p->toJson());
    });
}

long SensorLogger::currentTimestamp()
{
    struct timeval te;
    gettimeofday(&te, NULL);
    return te.tv_sec * 1000LL + te.tv_usec / 1000;
}

void SensorLogger::appendToFile(int sensorId, std::string msg)
{
    std::ofstream f;
    f.open(filename, std::ios::out | std::ios::app);
    f << sensorId << "|" << currentTimestamp() << "|" << msg << "\n";
    f.close();
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
