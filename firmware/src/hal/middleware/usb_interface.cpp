#include "middleware/usb_interface.h"
#include <stdio.h>
#include <string.h>

void usb_interface_init(void) {}

void usb_interface_task(void) {
    // USB CDC task is handled by stdio
}

int usb_interface_get_command(char *buffer, int len) {
    // Read a full line (including spaces) from stdin so commands like
    // "M FL:1 FR:2 RL:3 RR:4" are accepted. fgets returns NULL on EOF/error.
    if (fgets(buffer, len, stdin) == NULL) {
        return 0;
    }
    // Trim trailing newline and carriage return
    size_t l = strlen(buffer);
    while (l > 0 && (buffer[l - 1] == '\n' || buffer[l - 1] == '\r')) {
        buffer[--l] = '\0';
    }
    return 1;
}

void usb_interface_send_response(const char *response) { printf("%s\n", response); }