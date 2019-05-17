#include <time.h>
#include "common.h"


long getTimestamp_ms() {
    struct timespec time;
    clock_gettime(CLOCK_MONOTONIC, &time);
    long ts = time.tv_sec * 1000 + time.tv_nsec / 1000000;
    return ts;
}

