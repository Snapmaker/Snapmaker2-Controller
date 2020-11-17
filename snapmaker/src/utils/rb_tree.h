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
#ifndef RB_TREE_H_
#define RB_TREE_H_

#include <stdio.h>

enum RBTreeColor {
  RB_TREE_COLOR_RED,
  RB_TREE_COLOR_BLACK
};

template <typename K, typename E>
class RBTree {
  public:
    void Init(K key, E element) {
      key_ = key;
      element_ = element;
      left_ = right_ = parent_ = NULL;
      color_ = RB_TREE_COLOR_RED;
    }

    RBTree<K, E> *Search(K key);
    void Insert(RBTree<K, E> **root);

    void RotateLeft(RBTree<K, E> **root);
    void RotateRight(RBTree<K, E> **root);

    RBTree<K, E> *parent() { return parent_; }
    RBTree<K, E> *left() { return left_; }
    RBTree<K, E> *right() { return right_; }

    void parent(RBTree<K, E> *node) { parent_ = node; }
    void left(RBTree<K, E> *node) { left_ = node; }
    void right(RBTree<K, E> *node) { right_ = node; }

    bool color() { return color_; }
    void color(RBTreeColor color) { color_ = color; }

    K key() { return key_; }
    E element() { return element_; }
  private:
    void CorrectInsertion(RBTree<K, E> **root);

  private:
    K key_;
    E element_;

    RBTreeColor color_;

    RBTree<K, E> *left_;
    RBTree<K, E> *right_;
    RBTree<K, E> *parent_;
};

#endif  // #ifndef RB_TREE_H_
