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
    if (this->originalStreamReader != nullptr) {
        delete this->originalStreamReader;
    }
    this->originalStreamReader = requestStream("/vision-module/cmd/original", 17720);
}

void NetworkStreamReader::onProcessOriginalStream(StreamData *frame) {
    printf ("got new original frame: %ld size\n", frame->len);
}


NetworkStreamReader * WebRTCApiStream::requestStream(const char *topic, int port)
{
    NetworkStreamReader *reader = (new NetworkStreamReader(pubSubHost, pubSubPort, localIP, port))
                    ->withBufferSize(1)
                    ->withStreamRequestUri(topic)
                    ->withOnProcessCallback(NetworkStreamReader::onProcessOriginalStream, this);
                    
    reader->connect();
    return reader;   
}

bool WebRTCApiStream::isAlive()
{
    return WebRTCApiStream::getInstance() != nullptr;
}