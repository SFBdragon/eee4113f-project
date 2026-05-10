// Shaun's C code.

#include <stdbool.h>

#include "../Inc/netio.h"
#include "../Inc/control.h"




void initialize_networking() {
    Status lora_status = initialize_lora();
    if (wifi_status != STATUS_SUCCESS) {

    }

    Status wifi_status = initialize_wifi();
    if (wifi_status != STATUS_SUCCESS) {

    }
}

bool SHOULD_STREAM_RECORDS = false;
bool should_stream_record() {
    return SHOULD_STREAM_RECORDS;
}

int stream_record(RecordHeader *header, uint8_t* payload) {
    // TODO
}
