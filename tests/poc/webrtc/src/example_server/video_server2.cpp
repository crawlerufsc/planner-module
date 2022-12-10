#include "rtc/rtc.hpp"

#include <cstddef>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <utility>
#include <chrono>
#include <thread>
#include "webrtc.h"

int main()
{
	WebRTCService<unsigned char> service;

	std::cout << "RTP video stream expected on localhost:6000" << std::endl;

	std::cout << "Service SDP:\n" << service.getSdpService() << "\n\n";

	std::cout << "Please copy/paste the answer provided by the browser: " << std::endl;
	std::string sdp;
	std::getline(std::cin, sdp);

	std::cout << "\n\nblz...\n\n";
	service.serveStreamTo(sdp);

	service.openLocalUdpStreamConnection("127.0.0.1", 6000);
}
