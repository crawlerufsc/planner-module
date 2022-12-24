#ifndef _WEBRTC_BRIDGE_H
#define _WEBRTC_BRIDGE_H

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
#include <string>
#include <mutex>

#define DEBUG 1
#define BUFFER_SIZE 2048

// using nlohmann::json;

class WebRTCBridge
{
private:
    std::shared_ptr<rtc::PeerConnection> peerConnection;
    std::shared_ptr<rtc::Track> track;
    std::mutex mux;
    std::thread *bridgeThr;
    std::string sdpService;
    std::string udpStreamReceiveHost;
    rtc::Description::Video *media;
    int udpStreamReceivePort;
    int sock;

    const rtc::SSRC ssrc = 42;
    bool run;

    void initialize()
    {

        mux.lock();

#ifdef DEBUG
        rtc::InitLogger(rtc::LogLevel::Debug);
#else
        rtc::InitLogger(rtc::LogLevel::None);
#endif

        peerConnection = std::make_shared<rtc::PeerConnection>();

        peerConnection->onGatheringStateChange([this](rtc::PeerConnection::GatheringState state) { //
            if (state == rtc::PeerConnection::GatheringState::Complete)
            {
                auto description = peerConnection->localDescription();
                this->sdpService = description.value();
                mux.unlock();
            }
        });

        sock = socket(AF_INET, SOCK_DGRAM, 0);
        struct sockaddr_in addr = {};
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = inet_addr(udpStreamReceiveHost.c_str());
        addr.sin_port = htons(udpStreamReceivePort);

        printf ("binding UDP socket\n");
        if (bind(sock, reinterpret_cast<const sockaddr *>(&addr), sizeof(addr)) < 0)
            throw std::runtime_error("Failed to bind UDP socket on " + udpStreamReceiveHost + ":" + std::to_string(udpStreamReceivePort));


        printf ("setsockopt\n");
        int rcvBufSize = 212992;
        setsockopt(sock, SOL_SOCKET, SO_RCVBUF | SO_REUSEADDR, reinterpret_cast<const char *>(&rcvBufSize),
                   sizeof(rcvBufSize));

        media = new rtc::Description::Video ("video", rtc::Description::Direction::SendOnly);
        media->addH264Codec(96); // Must match the payload type of the external h264 RTP stream
        media->addSSRC(ssrc, "video-send");

        printf("add track\n");
        this->track = peerConnection->addTrack(*media);

        peerConnection->setLocalDescription();
    }

public:
    WebRTCBridge(std::string udpStreamReceiveHost, int udpStreamReceivePort)
    {
        printf ("WebRTCBridge()\n");
        this->run = false;
        this->bridgeThr = nullptr;
        this->udpStreamReceiveHost = udpStreamReceiveHost;
        this->udpStreamReceivePort = udpStreamReceivePort;
        initialize();
    }

    ~WebRTCBridge()
    {
        printf ("~WebRTCBridge()\n");
        if (this->bridgeThr != nullptr)
        {
            stopBridge();
        }
        close(this->sock);
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

    void connectToPeer(std::string targetSdpJson)
    {
        // json j = json::parse(targetSdpJson);
        // rtc::Description answer(j["sdp"].get<std::string>(), j["type"].get<std::string>());
        rtc::Description answer(targetSdpJson, "answer");
        peerConnection->setRemoteDescription(answer);
    }

    void bridgeThrHandler()
    {
        run = true;
        char buffer[BUFFER_SIZE];
        int len = 0;

        while (run && (len = recv(sock, buffer, BUFFER_SIZE, 0)) >= 0)
        {
            if (len < sizeof(rtc::RtpHeader) || !track->isOpen())
                continue;

            auto rtp = reinterpret_cast<rtc::RtpHeader *>(buffer);
            rtp->setSsrc(ssrc);

            track->send(reinterpret_cast<const std::byte *>(buffer), len);
        }
    }

    void stopBridge()
    {
        if (this->bridgeThr == nullptr)
            return;

        this->run = false;

        this->bridgeThr->join();
        delete this->bridgeThr;
        this->bridgeThr = nullptr;
    }

    void startBridge(std::string targetSdpJson)
    {
        if (this->bridgeThr != nullptr)
            stopBridge();

        connectToPeer(targetSdpJson);
        this->bridgeThr = new std::thread(&WebRTCBridge::bridgeThrHandler, this);
    }
};

#endif