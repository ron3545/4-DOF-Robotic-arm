#include "DataStructures.hpp"

void Node::push_back(Node* n)
{
    n->next_ = this;
    n->prev_ = prev_;
    prev_->next_ = n;
    prev_ = n;
}

 void Node::unlink()
 {
    Node *next = next_, *prev = prev_;
    next->prev_ = prev;
    prev->next_ = next;
    next_ = this;
    prev_ = this;
 }