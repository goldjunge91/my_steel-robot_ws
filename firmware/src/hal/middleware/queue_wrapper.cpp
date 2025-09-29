#include "middleware/queue_wrapper.h"

void queue_init(queue_t* q, int size, int element_size, void* buffer) {
    q->buffer = (uint8_t*)buffer;
    q->head = 0;
    q->tail = 0;
    q->size = size;
    q->element_size = element_size;
}

bool queue_send(queue_t* q, const void* item) {
    int next = (q->head + 1) % q->size;
    if (next == q->tail) {
        return false; // Queue is full
    }
    memcpy(q->buffer + q->head * q->element_size, item, q->element_size);
    q->head = next;
    return true;
}

bool queue_receive(queue_t* q, void* item) {
    if (q->head == q->tail) {
        return false; // Queue is empty
    }
    memcpy(item, q->buffer + q->tail * q->element_size, q->element_size);
    q->tail = (q->tail + 1) % q->size;
    return true;
}
