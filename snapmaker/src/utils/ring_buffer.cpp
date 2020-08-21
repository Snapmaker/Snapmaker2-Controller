#include "ring_buffer.h"


template<typename T>
void RingBuffer<T>::Init(int32_t size, T *buffer) {
  size_ = size;
  head_ = 0;
  tail_ = 0;
  data = buffer;
}


template<typename T>
void RingBuffer<T>::Deinit() {
  size_ = 0;
  head_ = 0;
  tail_ = 0;

  if (data != NULL)
    delete data;
  data = NULL;
}


template<typename T>
RingBuffer<T>::~RingBuffer() {
  deinit();
}


template<typename T>
int32_t RingBuffer<T>::InsertOne(const T &element) {
  if (isFull()) {
    return 0;
  }

  data[tail_] = element;
  if (++tail_ >= size_)
    tail_ = 0;

  return 1;
}


template<typename T>
int32_t RingBuffer<T>::RemoveOne(T &val) {
  if (isEmpty()) {
    return 0;
  }

  val = data[head_];
  if (++head_ >= size_)
    head_ = 0;

  return 1;
}


template<typename T>
int32_t RingBuffer<T>::PeekOne(T &val) {
  if (isEmpty()) {
    return 0;
  }

  val = data[head_];

  return 1;
}


template<typename T>
int32_t RingBuffer<T>::InsertMulti(T *buffer, int32_t to_insert) {
  if (isFull()) {
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


template<typename T>
int32_t RingBuffer<T>::RemoveMulti(T *buffer, int32_t to_remove) {
  if (isEmpty()) {
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


template<typename T>
int32_t RingBuffer<T>::PeekMulti(T *buffer, int32_t to_peek) {
  if (isEmpty()) {
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
