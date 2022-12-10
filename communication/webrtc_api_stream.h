#ifndef _WEBRTC_API_STREAM_H
#define _WEBRTC_API_STREAM_H

#include <pubsub_client.h>
#include <vector>
#include <webrtc.h>

#define WEBRTC_API_CLIENT_SDP_ORIGINAL_TOPIC "/stream/original/client-sdp"
#define WEBRTC_API_CLIENT_SDP_SEGMENTED_TOPIC "/stream/segmented/client-sdp"

class WebRTCApiStreamConnection 
{
public:
    WebRTCService<u_char> *service;
    int localPort;

    WebRTCApiStreamConnection(int localPort) {
        service = new WebRTCService<u_char>();
        this->localPort = localPort;
    }

    ~WebRTCApiStreamConnection() {
        delete service;
    }
};

class WebRTCApiStream : public PubSubClient
{
private:
    std::vector<WebRTCApiStreamConnection *> *connections;
    int pubSubPort;
    const char *localIP;
    const char *pubSubHost;

    void requestStream(const char *topic, int port);
    void requestOriginalStream(std::string spi);

protected:
    void onReceived(std::string topic, std::string payload) override;
    void onStop() override;

public:
    WebRTCApiStream(const char *pubSubHost, int pubSubPort, const char *localIP) : PubSubClient(pubSubHost, pubSubPort, WEBRTC_API_CLIENT_SDP_ORIGINAL_TOPIC)
    {
        this->pubSubHost = pubSubHost;
        this->pubSubPort = pubSubPort;
        this->localIP = localIP;
        this->connections = new std::vector<WebRTCApiStreamConnection *>();
    }
    ~WebRTCApiStream()
    {
        for (auto conn : *this->connections) {
            delete conn;
        }
        delete connections;
    }

    static WebRTCApiStream *_instance;

    static bool initialize(const char *pubSubHost, int pubSubPort, const char *localIP)
    {
        if (WebRTCApiStream::_instance != nullptr)
            delete WebRTCApiStream::_instance;

        WebRTCApiStream::_instance = new WebRTCApiStream(pubSubHost, pubSubPort, localIP);

        if (!WebRTCApiStream::_instance->blockUntilConnected(2000))
            return false;

        WebRTCApiStream::_instance->subscribeTo(WEBRTC_API_CLIENT_SDP_SEGMENTED_TOPIC);

        return true;
    }

    static WebRTCApiStream *getInstance()
    {
        return WebRTCApiStream::_instance;
    };

    static bool isAlive();
};
#endif