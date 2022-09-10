#ifndef _NETWORK_STREAM_READER_H
#define _NETWORK_STREAM_READER_H

// Include gstreamer library
#include <gst/gst.h>
#include <gst/app/app.h>
#include <stdio.h>
#include <string>
#include "../model/stream_data.h"
#include "../control/callback_process_pipeline.h"

class NetworkStreamReader : public CallbackProcessPipeline<StreamData>
{
private:
    const char *server_ip;
    int server_port;
    int local_port;
    int bufferSize;
    GstElement *pipeline;
    GstBus *bus;

    void (*onProcess)(StreamData *);

    bool buildPipeline();
    bool isRunning;
    bool isReceivingStream;

    bool requestDataFromServer();

    static GstFlowReturn new_preroll(GstAppSink * /*appsink*/, gpointer /*data*/)
    {
        return GST_FLOW_OK;
    }
    
    static gboolean busCallback(GstBus *bus, GstMessage *message, gpointer data);
    static StreamData *readStreamData(GstAppSink *sink);
    static GstFlowReturn newSample(GstAppSink *appsink, gpointer data);

    bool initialize() override;
    void onRequestNextFrame() override;
    void onTerminate() override;


public:
    NetworkStreamReader(const char * fromServerIp, int fromServerPort, int toLocalPort);
    ~NetworkStreamReader();

    NetworkStreamReader *withBufferSize(int bufferSize);
    NetworkStreamReader *withOnProcessCallback(void (*onProcess)(StreamData *));

    void requestConnectionToStreamServer();
    bool isConnected() override;

};

#endif