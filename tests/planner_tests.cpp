#include <stdio.h>
#include <glib.h>
#include <gst/gst.h>
#include <stdlib.h>
#include <thread>

#include "test_utils.h"
#include "test_result.h"

extern TestResult *tst_stream_reader();

int main(int argc, char **argv)
{
    execute_test(tst_stream_reader);
}

