#ifndef __CIRCULAR_BUFFER_H
#define __CIRCULAR_BUFFER_H

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// 环形缓冲区结构体
typedef struct {
    uint8_t *buffer;      // 动态分配的缓冲区指针
    uint16_t bufferSize;  // 缓冲区大小
    volatile uint16_t head; // 头指针（写入位置）
    volatile uint16_t tail; // 尾指针（读取位置）
    volatile uint16_t dataCount; // 当前缓冲区中的数据量
} ring_buffer_t;


bool ring_buffer_init(ring_buffer_t *rb, uint16_t bufferSize); // 初始化环形缓冲区
void ring_buffer_free(ring_buffer_t *rb);                     // 释放环形缓冲区
bool ring_buffer_is_empty(ring_buffer_t *rb);                   // 检查缓冲区是否为空
bool ring_buffer_is_full(ring_buffer_t *rb);                    // 检查缓冲区是否已满
uint16_t ring_buffer_get_data_count(ring_buffer_t *rb);          // 获取当前缓冲区中的数据量
bool ring_buffer_write(ring_buffer_t *rb, const uint8_t *data, uint16_t len); // 写入数据
bool ring_buffer_read(ring_buffer_t *rb, uint8_t *data, uint16_t len);        // 读取数据
void ring_buffer_clear(ring_buffer_t *rb);


#endif // __CIRCULAR_BUFFER_H