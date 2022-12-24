#ifndef _WEBRTC_API_STREAM_H
#define _WEBRTC_API_STREAM_H

#include <pubsub_client.h>
#include <vector>
#include <network_stream_reader.h>

#include "webrtc.h"
#include "../model/global_definitions.h"

class WebRTCApiStream : public PubSubClient
{
private:
    WebRTCBridge *bridge;
    std::string localIP;
    const char *lastRequestedUri;
    std::string lastSpd;
    bool keepTheSameWebsock;

    
    bool is_listening;

    void requestStream(const char *streamUri, int localPort, bool enable);
    void processStreamRequest(const char *streamUri, int localPort, std::string spd);
    void onRequestStreamResultReceived(std::string payload);
    void startBridging();
    void stopBridging();

protected:
    void onReceived(std::string topic, std::string payload) override;
    void onStop() override;

public:
    WebRTCApiStream(std::string localIP);
    ~WebRTCApiStream();

    void initialize();
    bool isListening();
    std::string getServiceSpd();
};
#endif