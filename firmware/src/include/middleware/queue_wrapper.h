#ifndef QUEUE_WRAPPER_H
#define QUEUE_WRAPPER_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#define QUEUE_SIZE 10

typedef struct {
    uint8_t* buffer;
    int head;
    int tail;
    int size;
    int element_size;
} queue_t;

void queue_init(queue_t* q, int size, int element_size, void* buffer);
bool queue_send(queue_t* q, const void* item);
bool queue_receive(queue_t* q, void* item);

#endif // QUEUE_WRAPPER_H
