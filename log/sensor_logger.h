#ifndef _SENSOR_LOGGER_H
#define _SENSOR_LOGGER_H

#include <functional>
#include <pubsub_client.h>
#include "imu_data.h"
#include "gps_data.h"
#include <iostream>
#include <fstream>
class SensorLogger : PubSubClient
{
private:
    std::string filename;
    std::function<void(IMUData *)> imuSensorCallback;
    std::function<void(GPSData *)> gpsSensorCallback;
    void start();
    void stop();
    bool is_logging;
    void appendToFile(int sensorId, std::string msg);
    long currentTimestamp();

protected:
    void onReceived(std::string topic, std::string payload) override;
    void onStop() override;

public:
    SensorLogger(std::string filename);
};

#endif