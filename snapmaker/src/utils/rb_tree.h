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
