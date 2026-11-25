
#ifndef SD_LOGGER_H
#define SD_LOGGER_H

#include <stdio.h>
#include "sdmmc_cmd.h"

class SDLogger {
public:
    bool init();
    bool write(const char* data);
    void close();

private:
    FILE* log_file = nullptr;
    sdmmc_card_t* card = nullptr;
    bool mounted = false;
};

#endif
