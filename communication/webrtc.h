#ifndef _WEBRTC_H
#define _WEBRTC_H

#include "rtc/rtc.hpp"

#include <cstddef>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <utility>

#include <nlohmann/json.hpp>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

#include <sstream>
#include <mutex>

//#define DEBUG 1

//using nlohmann::json;
using namespace std::placeholders;

template <typename FrameType>
class WebRTCService
{
private:
    // std::function<void(std::string)> onConnected;
    std::shared_ptr<rtc::PeerConnection> peerConnection;
    std::shared_ptr<rtc::Track> track;
    std::thread *serveStreamThread;
    std::mutex mux;
    std::string sdpService;
    const rtc::SSRC ssrc = 42;
    const char *localIP;
    int localPort;
    bool isRunning;

    void initialize()
    {
        mux.lock();

#ifdef DEBUG
        rtc::InitLogger(rtc::LogLevel::Debug);
#else
        rtc::InitLogger(rtc::LogLevel::None);
#endif

        peerConnection = std::make_shared<rtc::PeerConnection>();

        peerConnection->onGatheringStateChange([this](rtc::PeerConnection::GatheringState state)
                                               {
        	if (state == rtc::PeerConnection::GatheringState::Complete) {
        		auto description = peerConnection->localDescription();
                //std::stringstream ss;
                
        		// json message = {{"type", description->typeString()},
        		//                 {"sdp", std::string(description.value())}};
                
                //ss << message;
                //this->sdpService = ss.str();
                this->sdpService = description.value();
                mux.unlock();
        	} });

        rtc::Description::Video media("video", rtc::Description::Direction::SendOnly);
        media.addH264Codec(96); // Must match the payload type of the external h264 RTP stream
        media.addSSRC(ssrc, "video-send");

        this->track = peerConnection->addTrack(media);

        peerConnection->setLocalDescription();
    }

    int bindUDPEndpoint(const char *localIp, int localPort)
    {
        int sock = socket(AF_INET, SOCK_DGRAM, 0);

        struct sockaddr_in addr = {};
        memset((char *)&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = inet_addr(localIp);
        addr.sin_port = htons(localPort);

        if (bind(sock, reinterpret_cast<const sockaddr *>(&addr), sizeof(addr)) < 0)
            throw std::runtime_error("Failed to bind UDP socket");

        int rcvBufSize = 212992;
        setsockopt(sock, SOL_SOCKET, SO_RCVBUF, reinterpret_cast<const char *>(&rcvBufSize),
                   sizeof(rcvBufSize));

        return sock;
    }

    void streamThr()
    {
        isRunning = true;
        int sock = bindUDPEndpoint(localIP, localPort);

        unsigned char buffer[2048];
        int len;
        while (isRunning && (len = recv(sock, buffer, 2048, 0)) >= 0)
        {
            if (len < sizeof(rtc::RtpHeader) || !track->isOpen())
                continue;

            auto rtp = reinterpret_cast<rtc::RtpHeader *>(buffer);
            rtp->setSsrc(ssrc);

            this->track->send(reinterpret_cast<const std::byte *>(buffer), len);
        }
    }

public:
    WebRTCService()
    {
        isRunning = false;
        initialize();
    }

    ~WebRTCService()
    {
        isRunning = false;
        if (serveStreamThread != nullptr)
        {
            serveStreamThread->join();
            delete serveStreamThread;
        }
    }

    std::string getSdpService()
    {
        std::string p;
        mux.lock();
        if (sdpService.size() > 0)
        {
            p = sdpService;
        }
        mux.unlock();
        return p;
    }

    void serveStreamTo(std::string targetSdpJson)
    {
        printf("serveStreamTo() payload:\n%s\n", targetSdpJson.c_str());
        //json j = json::parse(targetSdpJson);
        //rtc::Description answer(j["sdp"].get<std::string>(), j["type"].get<std::string>());
        rtc::Description answer(targetSdpJson, "answer");
        peerConnection->setRemoteDescription(answer);
    }

    void startServing(const char *localIP, int localPort)
    {
        if (isRunning)
            return;

        this->localIP = localIP;
        this->localPort = localPort;
        serveStreamThread = new std::thread(&WebRTCService::streamThr, this);
    }
};

#endif