#include <stdio.h>
#include <glib.h>
#include <gst/gst.h>
#include <stdlib.h>
#include <thread>
#include <string>
#include <sstream>

#include "../../communication/stream_reader.h"
#include "../test_result.h"

static int frame_count = 0;

GstElement *building_source_stream(int port)
{
    GstElement *pipeline;
    GstElement *source;
    GstElement *encoder, *rtph264_payload;
    GstElement *sink;

    pipeline = gst_pipeline_new("stream-test-source");
    source = gst_element_factory_make("videotestsrc", "source");
    encoder = gst_element_factory_make("x264enc", "enc");
    rtph264_payload = gst_element_factory_make("rtph264pay", "rtph264pay");
    sink = gst_element_factory_make("udpsink", "sink");

    if (!pipeline)
    {
        fprintf(stderr, "pipeline failed\n");
        return nullptr;
    }
    if (!source)
    {
        fprintf(stderr, "source failed\n");
        return nullptr;
    }
    if (!encoder)
    {
        fprintf(stderr, "encoder failed\n");
        return nullptr;
    }
    if (!rtph264_payload)
    {
        fprintf(stderr, "rtph264_payload failed\n");
        return nullptr;
    }
    if (!sink)
    {
        fprintf(stderr, "sink failed\n");
        return nullptr;
    }

    g_object_set(G_OBJECT(source), "pattern", 0, NULL);
    g_object_set(G_OBJECT(sink), "host", "10.0.0.149", NULL);
    g_object_set(G_OBJECT(sink), "port", port, NULL);

    gst_bin_add_many(GST_BIN(pipeline), source, encoder, rtph264_payload, sink, NULL);

    if (!gst_element_link_many(source, encoder, rtph264_payload, sink, NULL))
    {
        fprintf(stderr, "link failed\n");
        return nullptr;
    }

    gst_element_set_state(pipeline, GST_STATE_PLAYING);

    return pipeline;
}

GstFlowReturn new_sample(GstAppSink *appsink, gpointer data)
{
    StreamData *frame = StreamReader::readStreamData(appsink);
    // printf("received a new frame with size %d x %d\n", frame->width, frame->height);
    delete frame;
    frame_count++;
    return GST_FLOW_OK;
}

gboolean my_bus_callback(GstBus *bus, GstMessage *message, gpointer data)
{
    return true;
}

void executing_reader_stream(StreamReader *reader)
{
    reader->loopReceive();
}

TestResult *tst_stream_reader()
{
    int test_port = 20000;
    int test_time_s = 2;
    gst_init(NULL, NULL);

    auto producer_pipeline = building_source_stream(test_port);

    if (producer_pipeline == nullptr)
        return TestResult::fail("tst_stream_reader", "pipeline creation failed");

    StreamReader *reader = new StreamReader();
    reader->withBufferSize(1)
        ->withLocalPort(test_port)
        ->withMessageSinkCallback(new_sample)
        ->withBusCallback(my_bus_callback);

    std::thread consumer(executing_reader_stream, reader);
    sleep(test_time_s);
    reader->terminate();
    consumer.join();

    gst_element_set_state(producer_pipeline, GST_STATE_NULL);
    gst_object_unref(producer_pipeline);

    std::ostringstream msg;
    msg << "received " << frame_count << " frames in " << test_time_s << " seconds";
    return new TestResult("tst_stream_reader", msg.str(), frame_count > 0);
}
