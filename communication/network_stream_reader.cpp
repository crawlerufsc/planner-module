// Include atomic std library
#include <atomic>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <netdb.h>
#include <string>
#include "network_stream_reader.h"
#include <iostream>

NetworkStreamReader::NetworkStreamReader(const char *serverIP, int serverPort, int localPort)
{
    this->bufferSize = 1;
    this->isRunning = false;
    this->server_ip = serverIP;
    this->server_port = serverPort;
    this->local_port = localPort;
}

NetworkStreamReader::~NetworkStreamReader()
{
    gst_element_set_state(GST_ELEMENT(pipeline), GST_STATE_NULL);

    if (pipeline != nullptr)
        gst_object_unref(GST_OBJECT(pipeline));

    if (bus != nullptr)
        gst_object_unref(bus);
}

NetworkStreamReader *NetworkStreamReader::withBufferSize(int bufferSize)
{
    this->bufferSize = bufferSize;
    return this;
}

StreamData *NetworkStreamReader::readStreamData(GstAppSink *sink)
{
    GstSample *sample = gst_app_sink_pull_sample(sink);
    GstCaps *caps = gst_sample_get_caps(sample);
    GstBuffer *buffer = gst_sample_get_buffer(sample);
    GstStructure *structure = gst_caps_get_structure(caps, 0);

    GstMapInfo map;
    gst_buffer_map(buffer, &map, GST_MAP_READ);

    int width = g_value_get_int(gst_structure_get_value(structure, "width"));
    int height = g_value_get_int(gst_structure_get_value(structure, "height"));
    StreamData *result = new StreamData(width, height, map.data, 0);

    // gst_buffer_unmap(buffer, &map);
    gst_sample_unref(sample);

    return result;
}

bool NetworkStreamReader::buildPipeline()
{
    gst_init(NULL, NULL);

    char confmsg[512];
    sprintf(confmsg, "udpsrc port=%d "
                     "! application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96 "
                     "! rtph264depay ! decodebin ! videoconvert "
                     "! appsink name=sink emit-signals=true sync=false max-buffers=%d drop=true",
            local_port, bufferSize);

    gchar *descr = g_strdup(confmsg);
    GError *error = nullptr;
    pipeline = gst_parse_launch(descr, &error);

    if (error)
    {
        g_print("could not construct pipeline: %s\n", error->message);
        g_error_free(error);
        return false;
    }

    bus = gst_pipeline_get_bus(GST_PIPELINE(pipeline));

    gst_bus_add_watch(bus, &NetworkStreamReader::busCallback, this);

    return true;
}

void NetworkStreamReader::onTerminate()
{
    if (pipeline == nullptr)
        return;

    isRunning = false;
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(pipeline);
}

GstFlowReturn NetworkStreamReader::newSample(GstAppSink *appsink, gpointer data)
{
    NetworkStreamReader *reader = (NetworkStreamReader *)data;
    StreamData *frame = NetworkStreamReader::readStreamData(appsink);
    reader->reportNewFrameReceived(frame);
    return GST_FLOW_OK;
}

NetworkStreamReader *NetworkStreamReader::withOnProcessCallback(void (*onProcess)(StreamData *)) {
    this->onProcess = onProcess;
    return this;
}

bool NetworkStreamReader::initialize()
{
    if (buildPipeline())
    {
        GstElement *sink = gst_bin_get_by_name(GST_BIN(pipeline), "sink");

        gst_app_sink_set_emit_signals((GstAppSink *)sink, true);
        gst_app_sink_set_drop((GstAppSink *)sink, true);
        gst_app_sink_set_max_buffers((GstAppSink *)sink, 1);

        GstAppSinkCallbacks callbacks = {nullptr, NetworkStreamReader::new_preroll, NetworkStreamReader::newSample};
        gst_app_sink_set_callbacks(GST_APP_SINK(sink), &callbacks, this, nullptr);

        gst_element_set_state(GST_ELEMENT(pipeline), GST_STATE_PLAYING);
    }

    return true;
}

void NetworkStreamReader::requestConnectionToStreamServer()
{
    struct sockaddr_in serverAddr;
    isRunning = false;

    int connFd = socket(AF_INET, SOCK_STREAM, 0);

    if (connFd < 0)
    {
        printf("unable to open a TCP socket\n");
        terminate();
        return;
    }

    struct hostent *server = gethostbyname(server_ip);

    if (server == NULL)
    {
        printf("host %s not found\n", server_ip);
        terminate();
        return;
    }

    bzero((char *)&serverAddr, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;

    bcopy((char *)server->h_addr, (char *)&serverAddr.sin_addr.s_addr, server->h_length);

    serverAddr.sin_port = htons(server_port);

    if (connect(connFd, (struct sockaddr *)&serverAddr, sizeof(serverAddr)) < 0)
    {
        printf("can't connect to host %s on port %d\n", server_ip, server_port);
        terminate();
        return;
    }

    std::string port = std::to_string(local_port);
    write(connFd, port.c_str(), port.size());
    close(connFd);

    isRunning = true;
}

bool NetworkStreamReader::isConnected()
{
    return isRunning;
}

void NetworkStreamReader::onRequestNextFrame()
{
    if (!isRunning)
    {
        terminate();
        return;
    }

    GstMessage *message = gst_bus_timed_pop_filtered(bus, GST_CLOCK_TIME_NONE,
                                                     (GstMessageType)(GST_MESSAGE_STATE_CHANGED | GST_MESSAGE_ERROR | GST_MESSAGE_EOS));


    busCallback(this->bus, message, this);
}
gboolean NetworkStreamReader::busCallback(GstBus *bus, GstMessage *message, gpointer data)
{
    NetworkStreamReader *reader = (NetworkStreamReader *)data;

    switch (GST_MESSAGE_TYPE(message))
    {
    case GST_MESSAGE_ERROR:
    {
        GError *err;
        gchar *debug;
        gst_message_parse_error(message, &err, &debug);
        g_print("Error: %s\n", err->message);
        g_error_free(err);
        g_free(debug);
        break;
    }
    case GST_MESSAGE_EOS:
        reader->terminate();
        break;
    }
    return true; // notify again
}