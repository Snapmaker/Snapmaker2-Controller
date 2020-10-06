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
  void Init(int32_t size, T *buffer) {
    size_ = size;
    head_ = 0;
    tail_ = 0;
    data = buffer;
  }

  int32_t InsertOne(const T& element) {
    if (IsFull()) {
      return 0;
    }

    data[tail_] = element;
    if (++tail_ >= size_)
      tail_ = 0;

    return 1;
  }

  int32_t RemoveOne(T &val) {
  if (IsEmpty()) {
    return 0;
  }

  val = data[head_];
  if (++head_ >= size_)
    head_ = 0;

  return 1;
}

  int32_t PeekOne(T &val) {
    if (IsEmpty()) {
      return 0;
    }

    val = data[head_];

    return 1;
  }

  int32_t InsertMulti(T *buffer, int32_t to_insert) {
    if (IsFull()) {
      return 0;
    }

    if (Free() < to_insert)
      return 0;

    for (int32_t i = 0; i < to_insert; i++) {
      data[tail_] = buffer[i];
      if (++tail_ >= size_)
        tail_ = 0;
    }

    return to_insert;
  }

  int32_t RemoveMulti(T *buffer, int32_t to_remove) {
    if (IsEmpty()) {
      return 0;
    }

    // if didn't specify number to remove, try to remove all
    if (0 == to_remove) {
      to_remove = Available();
    }
    else if (Available() < to_remove) {
      to_remove = Available();
    }

    for (int32_t i = 0; i < to_remove; i++) {
      buffer[i] = data[head_];
      if (++head_ >= size_) {
        head_ = 0;
      }
    }

    return to_remove;
  }

  int32_t PeekMulti(T *buffer, int32_t to_peek) {
    if (IsEmpty()) {
      return 0;
    }

    if (Available() < to_peek)
      return 0;

    // if didn't specify number to peek, try to peek all
    if (0 == to_peek)
      to_peek = Available();

    int32_t tmp_head = head_;

    for (int32_t i = 0; i < to_peek; i++) {
      buffer[i] = data[tmp_head];
      if (++tmp_head >= size_) {
        tmp_head = 0;
      }
    }

    return to_peek;
  }

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

  void Deinit() {
    size_ = 0;
    head_ = 0;
    tail_ = 0;

    if (data != NULL)
      delete data;
    data = NULL;
  }
};



#endif //MODULES_WHIMSYCWD_MARLIN_SRC_UTILS_RINGBUFFER_H_
