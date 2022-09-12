#include <gtest/gtest.h>
#include <string>

#include "../../utils/data_convert.h"

TEST(DataConvert, TestStringConvert) {
    std::string p = "test data conversion";
    auto q = DataConvert::convert(p);

    for (auto &ch1 : p) {
        EXPECT_EQ(ch1, *q++);
    }
}