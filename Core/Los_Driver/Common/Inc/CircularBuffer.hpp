#ifndef CIRCULAR_BUFFER_HPP
#define CIRCULAR_BUFFER_HPP

#include <stdint.h>

class CircularBuffer {
    public:

    uint8_t peek(uint8_t& res, int dist); 
    uint8_t read(uint8_t* buf, int size);

    uint8_t write(uint8_t data);

    bool hasSpace();

    CircularBuffer(uint8_t* buf, int size);
    CircularBuffer();

    private:

    uint8_t* buf;
    int size;

    int writePtr = 0;
    int readPtr = 0;
};


#endif