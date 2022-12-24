cmake_minimum_required(VERSION 3.7)

set(MEDIA_UWP_RESOURCES
	uwp/Logo.png
	uwp/package.appxManifest
	uwp/SmallLogo.png
	uwp/SmallLogo44x44.png
	uwp/SplashScreen.png
	uwp/StoreLogo.png
	uwp/Windows_TemporaryKey.pfx
)

SET(TARGET_H
	webrtc.h
)

add_executable(video_server video_server.cpp)


set_target_properties(video_server PROPERTIES
    CXX_STANDARD 17
    OUTPUT_NAME video_server)

set_target_properties(video_server PROPERTIES
	XCODE_ATTRIBUTE_PRODUCT_BUNDLE_IDENTIFIER com.github.out0.video_server)

find_package(PkgConfig REQUIRED)
find_package(Threads REQUIRED)
find_package(LibDataChannel REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(OpenCV REQUIRED)

pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)
pkg_check_modules(GST_APP REQUIRED gstreamer-app-1.0)
pkg_check_modules(GST_VIDEO REQUIRED gstreamer-video-1.0)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(/usr/include/gstreamer-1.0)
include_directories(/usr/include/crawler)

# framework lib
find_library(FRAMEWORK_LIB REQUIRED
    NAMES crawler_fw
    HINTS /usr/lib
)

target_link_libraries(video_server 
	LibDataChannel::LibDataChannel 
	Threads::Threads nlohmann_json  
	${GST_APP_LIBRARIES} 
	${GST_VIDEO_LIBRARIES} 
	${OpenCV_LIBS}
	${FRAMEWORK_LIB}
)

if(MSVC)
	add_custom_command(TARGET video_server POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E copy_if_different
		"$<TARGET_FILE_DIR:datachannel>/datachannel.dll"
		$<TARGET_FILE_DIR:video_server>
	)
endif()