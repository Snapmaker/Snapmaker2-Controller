//
// Created by David Chen on 2019-07-23.
//

#ifndef MODULES_WHIMSYCWD_MARLIN_SRC_UTILS_RINGBUFFER_H_
#define MODULES_WHIMSYCWD_MARLIN_SRC_UTILS_RINGBUFFER_H_

#include <stdint.h>
#include <stdio.h>
#define DEFAULT_RING_BUFFER_SIZE 128


template <typename T>
class RingBuffer {
 public:
  RingBuffer();

  void Init(int32_t size, T *buffer);

  int32_t InsertOne(const T& element);
  int32_t RemoveOne(T &val);
  int32_t PeekOne(T &val);

  int32_t InsertMulti(T *buffer, int32_t to_insert);
  int32_t RemoveMulti(T *buffer, int32_t to_remove);
  int32_t PeekMulti(T *buffer, int32_t to_peek);

  // compiler will treat the functions who have body defined
  // in Class as inline function by default
  bool IsFull() {
    return (tail_ + 1 == head_) || ((tail_ + 1) == size_ && head_ == 0);
  }

  bool IsEmpty() {
    return head_ == tail_;
  }

  int32_t Available() {
    int32_t delta = (int32_t)(tail_ - head_);
    return (delta < 0)? (delta + size_) : delta;
  }

  int32_t Free() {
    int32_t delta = (int32_t)(head_ - tail_);
    return (delta < 0)? (delta + size_) : delta;
  }

 private:
  int32_t size_;
  int32_t head_;
  int32_t tail_;
  T *data;

  void Deinit();
};



#endif //MODULES_WHIMSYCWD_MARLIN_SRC_UTILS_RINGBUFFER_H_
