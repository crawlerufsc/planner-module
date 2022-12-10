#include "webrtc_api_stream.h"
#include <nlohmann/json.hpp>

using nlohmann::json;

WebRTCApiStream *WebRTCApiStream::_instance = nullptr;

void WebRTCApiStream::onReceived(std::string topic, std::string payload)
{
    if (topic == "/stream/original/client-sdp")
    {
        requestOriginalStream(payload);
    }
    
}
void WebRTCApiStream::onStop()
{
}

void WebRTCApiStream::requestOriginalStream(std::string spi)
{
    WebRTCApiStreamConnection *conn = new WebRTCApiStreamConnection(17720);
    conn->service->startServing(this->localIP, 17720);
    conn->service->serveStreamTo(spi);
    this->connections->push_back(conn);
    requestStream("/vision-module/cmd/original", 17720);
}


void WebRTCApiStream::requestStream(const char *topic, int port)
{
    json req{
        {"ip", localIP},
        {"port", port},
        {"enable", true}};

    publishTo(topic, req.dump());
}

bool WebRTCApiStream::isAlive()
{
    return WebRTCApiStream::getInstance() != nullptr;
}