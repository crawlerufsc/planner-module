#include "webrtc_api_stream.h"

#include <nlohmann/json.hpp>
#include <functional>
#include <frame.h>

using nlohmann::json;

#define WEB_RTC_STREAM_PORT 17720

WebRTCApiStream::WebRTCApiStream(std::string localIP) : PubSubClient(BROKER_IP, BROKER_PORT)
{
    printf("WebRTCApiStream()\n");
    this->bridge = new WebRTCBridge(localIP, WEB_RTC_STREAM_PORT);
    this->localIP = localIP;
    this->lastRequestedUri = nullptr;
    this->keepTheSameWebsock = false;
    initialize();
}
WebRTCApiStream::~WebRTCApiStream()
{
    printf("~WebRTCApiStream()\n");
    this->bridge->stopBridge();
    delete this->bridge;
}

void WebRTCApiStream::initialize()
{
    is_listening = false;

    if (!blockUntilConnected(2000))
        return;

    subscribeTo(WEBRTC_CLIENT_SDP_RESPONSE_ORIGINAL);
    subscribeTo(WEBRTC_CLIENT_SDP_RESPONSE_SEGMENTED);
    subscribeTo(PUBSUB_STREAM_RESPONSE_URI);
    is_listening = true;
}

void WebRTCApiStream::onReceived(std::string topic, std::string payload)
{
    if (topic == WEBRTC_CLIENT_SDP_RESPONSE_ORIGINAL)
    {
        processStreamRequest(PUBSUB_STREAM_REQUEST_URI_ORIGINAL, WEB_RTC_STREAM_PORT, payload);
    }
    else if (topic == WEBRTC_CLIENT_SDP_RESPONSE_SEGMENTED)
    {
        processStreamRequest(PUBSUB_STREAM_REQUEST_URI_SEGMENTED, WEB_RTC_STREAM_PORT, payload);
    }
    else if (topic == PUBSUB_STREAM_RESPONSE_URI)
    {
        onRequestStreamResultReceived(payload);
    }
}
void WebRTCApiStream::onStop()
{
}

void WebRTCApiStream::onRequestStreamResultReceived(std::string payload)
{
    json p = json::parse(payload);

    if (p["targetIP"].get<std::string>() == std::string(localIP) &&
        p["targetPort"].get<int>() == WEB_RTC_STREAM_PORT)
    {
        // if (p["enable"].get<bool>())
        this->bridge->startBridge(lastSpd);
        // else
        // {
        //     this->bridge->stopBridge();
        // }
    }
}

void WebRTCApiStream::stopBridging()
{
    requestStream(lastRequestedUri, WEB_RTC_STREAM_PORT, false);
    this->bridge->stopBridge();
    this->lastRequestedUri = nullptr;
}

void WebRTCApiStream::processStreamRequest(const char *streamUri, int localPort, std::string payload)
{
    if (streamUri == nullptr)
        return;

    if (payload == "__change__")
    {
        stopBridging();
        requestStream(streamUri, localPort, true);
    }
    else if (payload == "__close__")
        stopBridging();
    else
    {
        this->lastSpd = payload;
        requestStream(streamUri, localPort, true);
    }
}

void WebRTCApiStream::requestStream(const char *streamUri, int localPort, bool enable)
{
    json j{
        {"ip", localIP},
        {"port", localPort},
        {"enable", enable},
    };

    publishTo(streamUri, j.dump());
    lastRequestedUri = streamUri;
}

bool WebRTCApiStream::isListening()
{
    return is_listening;
}

std::string WebRTCApiStream::getServiceSpd()
{
    return this->bridge->getSdpService();
}