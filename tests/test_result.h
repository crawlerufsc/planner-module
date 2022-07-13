#ifndef _TEST_RESULT_H
#define _TEST_RESULT_H
#include <stdio.h>
#include <stdlib.h>
#include <gst/gst.h>
#include <string>

class TestResult
{
public:
    std::string testName;
    std::string messageResult;
    bool result;

    TestResult(std::string testName, std::string msg, bool result)
    {
        this->testName = testName;
        this->messageResult = msg;
        this->result = result;
    }

    static TestResult *success(std::string testName, const char *msg)
    {
        TestResult *result = new TestResult(testName, msg, true);
        return result;
    }

    static TestResult *fail(std::string testName, const char *msg)
    {
        TestResult *result = new TestResult(testName, msg, false);
        return result;
    }
};

#endif