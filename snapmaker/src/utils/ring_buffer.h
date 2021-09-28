/*
 * Snapmaker2-Controller Firmware
 * Copyright (C) 2019-2020 Snapmaker [https://github.com/Snapmaker]
 *
 * This file is part of Snapmaker2-Controller
 * (see https://github.com/Snapmaker/Snapmaker2-Controller)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
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
    is_full_ = false;
    data = buffer;
  }

  int32_t InsertOne(const T& element) {
    if (IsFull()) {
      return 0;
    }

    data[tail_] = element;
    if (++tail_ >= size_)
      tail_ = 0;

    if (tail_ == head_) {
      is_full_ = true;
    }

    return 1;
  }

  int32_t InsertOne() {
    if (IsFull()) {
      return 0;
    }

    if (++tail_ >= size_)
      tail_ = 0;

    if (tail_ == head_) {
      is_full_ = true;
    }

    return 1;
  }

  int32_t ReadOne(T &val) {
    if (IsEmpty()) {
      return 0;
    }

    val = data[head_];
    return 1;
  }

  T * HeadAddress() {
    if (IsEmpty()) {
      return NULL;
    }

    return &data[head_];
  }

  T * TailAddress() {
    if (IsFull()) {
      return NULL;
    }

    return &data[tail_];
  }

  int32_t RemoveOne(T &val) {
    if (IsEmpty()) {
      return 0;
    }

    val = data[head_];
    if (++head_ >= size_)
      head_ = 0;
    is_full_ = false;
    return 1;
  }

  int32_t RemoveOne() {
    if (IsEmpty()) {
      return 0;
    }

    if (++head_ >= size_)
      head_ = 0;
    is_full_ = false;
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

    if (tail_ == size_) {
      is_full_ = true;
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
    is_full_ = false;
    return to_remove;
  }

  // compiler will treat the functions who have body defined
  // in Class as inline function by default
  bool IsFull() {
    return is_full_;
  }

  bool IsEmpty() {
    return (head_ == tail_) && (!is_full_);
  }

  int32_t Available() {
    int32_t delta = (int32_t)(tail_ - head_);
    if (delta == 0 && is_full_) {
      return size_;
    }
    return (delta < 0)? (delta + size_) : delta;
  }

  int32_t Free() {
    int32_t delta = (int32_t)(tail_ - head_);
    if (is_full_) {
      return 0;
    }
    return (delta >= 0)? (size_ - delta) : -delta;
  }

  void Reset() {
    head_ = tail_ = 0;
    is_full_ = false;
  }

 private:
  int32_t size_;
  int32_t head_;
  int32_t tail_;
  bool is_full_;
  T *data;
};



#endif //MODULES_WHIMSYCWD_MARLIN_SRC_UTILS_RINGBUFFER_H_
