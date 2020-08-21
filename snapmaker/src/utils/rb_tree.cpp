#include "rb_tree.h"

/*
 * to rotate left for current node
 * x = current node
 *      parent                         parent
 *     /                               /
 *    x                               y
 *   /  \      --(rotate left)-->    / \
 *  lx   y                          x  ry
 *     /   \                       /  \
 *    ly   ry                     lx  ly
 *
 *
 */
template <typename K, typename E>
void RBTree<K, E>::RotateLeft(RBTree<K, E> **root) {
  RBTree<K, E> *y = right_;

  right_ = y->left();
  if (right_ != NULL) {
    right_->parent(this);
  }

  y->parent(parent_);

  if (parent_ == NULL) {
    *root = y;
  }
  else {
     if (parent_->left() == this) {
       parent_->left(y);
     }
     else {
       parent_->right(y);
     }
  }

  y->left(this);
  parent_ = y;
}


/*
 * to rotate right for current node
 * y = current node
 *            py                               py
 *           /                                /
 *          y                                x
 *         /  \   --(rotate right)-->       /  \
 *        x   ry                           lx   y
 *       / \                                   / \
 *      lx  rx                                rx  ry
 *
 */
template <typename K, typename E>
void RBTree<K, E>::RotateRight(RBTree<K, E> **root) {
  RBTree<K, E> *x = left_;

  // update my left kid to right kid of x
  left_ = x->right();
  if (left_ != NULL) {
    // bind parent of my new left kid to me
    left_->parent(this);
  }

  // update parent of my old left kid to my parent
  x->parent(parent_);
  if (parent_ == NULL) {
    *root = x;
  }
  else {
    if (parent_->right() == this) {
      parent_->right(x);
    }
    else {
      parent_->left(x);
    }

    x->right(this);
    parent_ = x;
  }
}


template <typename K, typename E>
RBTree<K, E> *RBTree<K, E>::Search(K key) {
  RBTree<K, E> *node = this;


  while (node && (key != node->key())) {
    if (key < node->key())
      node = node->left();
    else
      node = node->right();
  }

  return node;
}


template <typename K, typename E>
void RBTree<K, E>::Insert(RBTree<K, E> **root) {
  parent_ = *root;

  while (parent_ != NULL) {
    if (key_ < parent_->key()) {
      parent_ = parent_->left();
    }
    else {
      parent_ = parent_->right();
    }
  }

  if (parent_ != NULL) {
    if (key_ < parent_->key()) {
      parent_->left(this);
    }
    else {
      parent_->right(this);
    }
  }
  else {
    *root = this;
    color_ = RB_TREE_COLOR_BLACK;
    return;
  }

  color_ = RB_TREE_COLOR_RED;
  CorrectInsertion(root);
}


template <typename K, typename E>
void RBTree<K, E>::CorrectInsertion(RBTree<K, E> **root) {
  RBTree<K, E> *gparent;
  RBTree<K, E> *parent;
  RBTree<K, E> *node = this;


  while ((parent = node->parent())) && parent->color() == RB_TREE_COLOR_RED) {
    gparent = parent->parent();

    if (parent == gparent->left()) {

      {
        RBTree<K, E> *uncle = gparent->right();
        if (uncle && uncle->color() == RB_TREE_COLOR_RED) {
          uncle->color(RB_TREE_COLOR_BLACK);
          parent->color(RB_TREE_COLOR_BLACK);
          gparent->color(RB_TREE_COLOR_RED);
          node = gparent;
          continue;
        }
      }

      if (parent->right() == node) {
        RBTree<K, E> *tmp;
        parent->RotateLeft(root);
        tmp = parent;
        parent = node;
        node = tmp;
      }

      parent->color(RB_TREE_COLOR_BLACK);
      gparent->color(RB_TREE_COLOR_BLACK);
      gparent->RotateRight(root);
    }
    else {
      // parent == gparent->right()
      {
        RBTree<K, E> *uncle = gparent->left();
        if (uncle && uncle->color() == RB_TREE_COLOR_RED) {
          uncle->color(RB_TREE_COLOR_BLACK);
          parent->color(RB_TREE_COLOR_BLACK);
          gparent->color(RB_TREE_COLOR_RED);
          node = gparent;
          continue;
        }
      }

      if (parent->left() == node) {
        RBTree<K, E> *tmp;
        parent->RotateRight(root);
        tmp = parent;
        parent = node;
        node = tmp;
      }

      parent->color(RB_TREE_COLOR_BLACK);
      gparent->color(RB_TREE_COLOR_RED);
      gparent->RotateLeft(root);
    }
  }

  *root->color(RB_TREE_COLOR_BLACK);
}

