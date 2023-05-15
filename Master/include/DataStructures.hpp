#pragma once

//========================================Linked list Impl==================================================
template<class T> class Linked_List;
class Iterator;

class Node
{
    template<class T> friend class Linked_List;
    friend class Iterator;

protected:
    Node *next_, *prev_;

    void push_back(Node* n);
    void unlink();
public:
    Node()
        : next_(this), prev_(this)
    {}

    ~Node() { unlink(); }
};

class Iterator {
protected:
    Node* node_;

    Iterator(Node* node)
        : node_(node)
    {}

public:
    Iterator& operator++() {
        node_ = node_->next_;
        return *this;
    }

    bool operator==(Iterator b) const { return node_ == b.node_; }
    bool operator!=(Iterator b) const { return node_ != b.node_; }
};

template<class T>
class Linked_List {
    class NodeT : public Node {
        friend class Linked_List<T>;
        T value_;
        NodeT(T t) : value_(t) {}
    };

    template<class U>
    class IteratorT : public Iterator {
        friend class Linked_List<T>;
        NodeT* node() const { return static_cast<NodeT*>(node_); }
    public:
        U& operator*() const { return node()->value_; }
        U* operator->() const { return &node()->value_; }
        operator IteratorT<U const>() const { return node_; } // iterator to const_iterator conversion
        IteratorT(Node* node) : Iterator{node} {}
    };

    Node list_;

public:
    using iterator = IteratorT<T>;
    using const_iterator = IteratorT<T const>;

    ~Linked_List() { clear(); }

    bool empty() const { return list_.next_ == &list_; }

    iterator begin() { return list_.next_; }
    iterator end() { return &list_; }

    void push_back(T t) { list_.push_back(new NodeT(t)); }
    void erase(const_iterator i) { delete i.node(); }

    void clear() {
        while(!empty())
            erase(begin());
    }
};