#include "ring_buffer.h"

/**
 * @brief 初始化环形缓冲区
 * @param rb: 环形缓冲区结构体指针
 * @param bufferSize: 缓冲区大小
 * @return true: 初始化成功; false: 初始化失败
 */
bool ring_buffer_init(ring_buffer_t *rb, uint16_t bufferSize) {
    
	  // 动态分配缓冲区内存
    rb->buffer = (uint8_t *)malloc(bufferSize);
    if (rb->buffer == NULL) {
        return false; // 内存分配失败
    }

    // 初始化其他成员
    rb->bufferSize = bufferSize;
    rb->head = 0;
    rb->tail = 0;
    rb->dataCount = 0;

    return true;
}

void ring_buffer_clear(ring_buffer_t *rb)
{
    rb->head = 0;
    rb->tail = 0;
    rb->dataCount = 0;
}

/**
 * @brief 释放环形缓冲区
 * @param rb: 环形缓冲区结构体指针
 */
void ring_buffer_free(ring_buffer_t *rb) {
    if (rb->buffer != NULL) {
        free(rb->buffer); // 释放动态分配的内存
        rb->buffer = NULL;
    }
    rb->bufferSize = 0;
    rb->head = 0;
    rb->tail = 0;
    rb->dataCount = 0;
}

/**
 * @brief 检查缓冲区是否为空
 * @param rb: 环形缓冲区结构体指针
 * @return true: 缓冲区为空; false: 缓冲区不为空
 */
bool ring_buffer_is_empty(ring_buffer_t *rb) {
    return rb->dataCount == 0;
}

/**
 * @brief 检查缓冲区是否已满
 * @param rb: 环形缓冲区结构体指针
 * @return true: 缓冲区已满; false: 缓冲区未满
 */
bool ring_buffer_is_full(ring_buffer_t *rb) {
    return rb->dataCount == rb->bufferSize;
}

/**
 * @brief 获取当前缓冲区中的数据量
 * @param rb: 环形缓冲区结构体指针
 * @return 当前缓冲区中的数据量
 */
uint16_t ring_buffer_get_data_count(ring_buffer_t *rb) {
    return rb->dataCount;
}

/**
 * @brief 写入数据到环形缓冲区
 * @param rb: 环形缓冲区结构体指针
 * @param data: 要写入的数据指针
 * @param len: 要写入的数据长度
 * @return true: 写入成功; false: 写入失败（缓冲区空间不足）
 */
bool ring_buffer_write(ring_buffer_t *rb, const uint8_t *data, uint16_t len) {
    if (len > rb->bufferSize - rb->dataCount) {
        return false; // 数据块太大，无法写入
    }
		
		// 锁定写位置
		uint16_t head = rb->head;

    // 计算从 head 到缓冲区末尾的可用空间
    uint16_t spaceToEnd = rb->bufferSize - head;

    if (len <= spaceToEnd) {
        // 如果数据块可以一次性写入
        memcpy(&rb->buffer[head], data, len);
    } else {
        // 如果数据块需要分两次写入（跨越缓冲区末尾）
        memcpy(&rb->buffer[head], data, spaceToEnd);
        memcpy(&rb->buffer[0], data + spaceToEnd, len - spaceToEnd);
    }

    // 更新 head 指针和数据量
    rb->head = (head + len) % rb->bufferSize;
    rb->dataCount += len;

    return true;
}

/**
 * @brief 从环形缓冲区读取数据
 * @param rb: 环形缓冲区结构体指针
 * @param data: 存储读取数据的指针
 * @param len: 要读取的数据长度
 * @return true: 读取成功; false: 读取失败（缓冲区数据不足）
 */
bool ring_buffer_read(ring_buffer_t *rb, uint8_t *data, uint16_t len) {
    if (len > rb->dataCount) {
        return false; // 数据块太大，无法读取
    }
		
		// 锁定读位置
		uint16_t tail = rb->tail;
		
    // 计算从 tail 到缓冲区末尾的可用数据
    uint16_t dataToEnd = rb->bufferSize - tail;

    if (len <= dataToEnd) {
        // 如果数据块可以一次性读取
        memcpy(data, &rb->buffer[tail], len);
    } else {
        // 如果数据块需要分两次读取（跨越缓冲区末尾）
        memcpy(data, &rb->buffer[tail], dataToEnd);
        memcpy(data + dataToEnd, &rb->buffer[0], len - dataToEnd);
    }

    // 更新 tail 指针和数据量
    rb->tail = (tail + len) % rb->bufferSize;
    rb->dataCount -= len;

    return true;
}