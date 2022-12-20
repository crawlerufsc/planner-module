#ifndef _SENSOR_LOGGER_H
#define _SENSOR_LOGGER_H

#include <functional>
#include <pubsub_client.h>
#include "imu_data.h"
#include "gps_data.h"

class SensorLogger : PubSubClient
{
private:
    std::function<void(IMUData *)> imuSensorCallback;
    std::function<void(GPSData *)> gpsSensorCallback;
    void start();
    void stop();
    bool is_logging;

protected:
    void onReceived(std::string topic, std::string payload) override;
    void onStop() override;

public:
    SensorLogger(const char *filename);
};

#endif