#ifndef _GLOBAL_DEFINITIONS_H
#define _GLOBAL_DEFINITIONS_H

//#define PLANNER_IP "10.42.1.1" 
#define PLANNER_IP "10.42.0.1" 
#define VISION_IP  "10.42.1.2" 
#define BROKER_IP  "10.42.0.1"
#define BROKER_PORT 1883
#define HW_CONTROLLER_USB_DEVICE "/dev/ttyUSB0"

#define RESOURCE_NAME_STREAM_READER_OCCUPANCYGRID "occupancygrid-stream-reader"

#define PUBSUB_STREAM_REQUEST_URI_ORIGINAL "/vision-module/cmd/original"
#define PUBSUB_STREAM_REQUEST_URI_SEGMENTED "/vision-module/cmd/segmented"
#define PUBSUB_STREAM_REQUEST_URI_OCCUPANCYGRID "/vision-module/cmd/og"

#define PUBSUB_STREAM_RESPONSE_URI "/vision-module/status/stream"

#define PUBSUB_API_SENSOR_STATUS_URI "/crawler/status"
#define PUBSUB_API_MANUAL_COMMAND_URI "/crawler/cmd"
#define PUBSUB_API_ORIGINAL_STREAM_LOGGING_REQUEST_URI "/stream/log/original"
#define PUBSUB_API_SEGMENTED_STREAM_LOGGING_REQUEST_URI "/stream/log/segmented"
#define PUBSUB_API_OCCUPANCYGRID_STREAM_LOGGING_REQUEST_URI "/stream/log/og"

#define PUBSUB_API_SENSOR_LOGGING_REQUEST_URI "/sensors/log"

#define WEBRTC_CLIENT_SDP_RESPONSE_ORIGINAL "/stream/webrtc-sdp/original"
#define WEBRTC_CLIENT_SDP_RESPONSE_SEGMENTED "/stream/webrtc-sdp/segmented"
#define WEBRTC_ORIGINAL_STREAM_PORT 17720
#define WEBRTC_SEGMENTED_STREAM_PORT 17720

#define LOCALPORT_STREAM_LOGGER_ORIGINAL 21001
#define LOCALPORT_STREAM_LOGGER_SEGMENTED 21002
#define LOCALPORT_STREAM_LOGGER_OCCUPANCYGRID 21003

#define LOCALPORT_STREAM_READER_OCCUPANCYGRID 20003

#define FILE_LOG_SENSORS "/home/cristiano/sensors.log"
#define FILE_LOG_STREAM_ORIGINAL "/home/cristiano/original_vision_output.mkv"
#define FILE_LOG_STREAM_SEGMENTED "/home/cristiano/segmented_vision_output.mkv"
#define FILE_LOG_STREAM_OG "/home/cristiano/og_vision_output.mkv"

#endif