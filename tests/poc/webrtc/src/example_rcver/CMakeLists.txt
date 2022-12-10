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


add_executable(video_rcver video_rcver.cpp)


set_target_properties(video_rcver PROPERTIES
    CXX_STANDARD 17
    OUTPUT_NAME video_rcver)

set_target_properties(video_rcver PROPERTIES
	XCODE_ATTRIBUTE_PRODUCT_BUNDLE_IDENTIFIER com.github.out0.video_server)

find_package(Threads REQUIRED)
find_package(LibDataChannel REQUIRED)
find_package(nlohmann_json REQUIRED)

target_link_libraries(video_rcver LibDataChannel::LibDataChannel Threads::Threads nlohmann_json)

if(MSVC)
	add_custom_command(TARGET video_rcver POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E copy_if_different
		"$<TARGET_FILE_DIR:datachannel>/datachannel.dll"
		$<TARGET_FILE_DIR:video_rcver>
	)
endif()