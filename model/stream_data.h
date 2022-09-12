#ifndef _STREAM_DATA_H
#define _STREAM_DATA_H

class StreamData 
{
public:
    unsigned char *data;
    int width;
    int height;
    int mattype;
    size_t raw_length;
};

#endif
