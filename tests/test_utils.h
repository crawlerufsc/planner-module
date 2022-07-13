#ifndef _TEST_UTILS_H
#define _TEST_UTILS_H

#include <stdio.h>
#include <stdlib.h>
#include <gst/gst.h>

#include "test_result.h"

void execute_test(TestResult *(f)())
{
    TestResult *result = f();
    fprintf(stdout, "%s...", result->testName.c_str());
    
    if (result->result)
        fprintf(stdout, "ok");
    else        
        fprintf(stdout, "fail");
    
    if (result->messageResult.size() > 0)
        fprintf(stdout, "        (%s)", result->messageResult.c_str());

    fprintf(stdout, "\n");
}

void assert(bool condition, const char *fail_msg = "failed")
{
    if (condition)
    {
        fputs(fail_msg, stderr);
        fputs("\n", stderr);
        exit(1);
    }
}
void assert(gboolean condition, const char *fail_msg = "failed")
{
    if (condition)
    {
        fputs(fail_msg, stderr);
        fputs("\n", stderr);
        exit(1);
    }
}

#endif