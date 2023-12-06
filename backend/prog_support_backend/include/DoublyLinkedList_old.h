#ifndef DOUBLYLINKEDLIST_H
#define DOUBLYLINKEDLIST_H

#include <iostream>

template<typename T>
struct Node {
    T value;
    Node* prev;
    Node* next;
};

template<typename T>
class DoublyLinkedList {
public:
    DoublyLinkedList() : head(nullptr), tail(nullptr) {}
    
    void insert(T value) {
        Node<T>* newNode = new Node<T> { value, tail, nullptr };
        if (tail) {
            tail->next = newNode;
        } else {
            head = newNode;
        }
        tail = newNode;
    }
    
    bool remove(T value) {
        Node<T>* current = head;
        while (current) {
            if (current->value == value) {
                if (current == head) {
                    head = current->next;
                } else {
                    current->prev->next = current->next;
                }
                if (current == tail) {
                    tail = current->prev;
                } else {
                    current->next->prev = current->prev;
                }
                delete current;
                return true;
            }
            current = current->next;
        }
        return false;
    }
    
    bool removeAt(int index) {
        if (index < 0) {
            return false;
        }
        Node<T>* current = head;
        for (int i = 0; i < index && current; i++) {
            current = current->next;
        }
        if (current) {
            if (current == head) {
                head = current->next;
            } else {
                current->prev->next = current->next;
            }
            if (current == tail) {
                tail = current->prev;
            } else {
                current->next->prev = current->prev;
            }
            delete current;
            return true;
        }
        return false;
    }

    DoublyLinkedList<T> copyUpToIndex(int index) const {
        DoublyLinkedList<T> newList;
        
        // Iterate through the original list up to the specified index
        Node<T>* current = head;
        int count = 0;
        while (current && count <= index) {
            newList.insert(current->value); // Add the current value to the new list
            
            current = current->next;
            count++;
        }
        
        return newList;
    }



    T getHeadValue() const 
    {
        if (head != nullptr) {
            return head->value;
        }
        // Handle the case when the list is empty
        // For simplicity, assuming T is default-constructible
        return T{};
    }
    
    void move(T value, int newIndex) {
        remove(value);
        Node<T>* current = head;
        for (int i = 0; i < newIndex && current; i++) {
            current = current->next;
        }
        if (current) {
            Node<T>* newNode = new Node<T> { value, current->prev, current };
            if (current->prev) {
                current->prev->next = newNode;
            } else {
                head = newNode;
            }
            current->prev = newNode;
        } else {
            insert(value);
        }
    }

    bool move(int oldIndex, int newIndex) {
    if (oldIndex == newIndex || oldIndex < 0 || newIndex < 0) {
        return false;
    }

    Node<T>* current = head;
    Node<T>* target = nullptr;
    Node<T>* source = nullptr;

    for (int i = 0; i < oldIndex && current; i++) {
        current = current->next;
    }
    if (!current) {
        return false;  // oldIndex out of range
    }
    target = current;

    current = head;
    for (int i = 0; i < newIndex && current; i++) {
        current = current->next;
    }
    if (!current) {
        return false;  // newIndex out of range
    }
    source = current;

    // Update pointers for target node
    if (target->prev) {
        target->prev->next = target->next;
    } else {
        head = target->next;  // target node is the head
    }
    if (target->next) {
        target->next->prev = target->prev;
    } else {
        tail = target->prev;  // target node is the tail
    }

    // Insert target node at newIndex
    if (source->prev) {
        source->prev->next = target;
    } else {
        head = target;  // target node is inserted at the beginning
    }
    target->prev = source->prev;
    target->next = source;
    source->prev = target;

    return true;
}

    bool replace(int index, const T& value) {
        Node<T>* current = head;
        for (int i = 0; i < index && current; i++) {
            current = current->next;
        }
        if (!current) {
            return false;
        }
        current->value = value;
        return true;
    }

    template<typename Func, typename... Args>
    bool iterate(Func func, Args&&... args) {
        Node<T>* current = head;
        bool result = true;
        int i = 1;
        while (current != nullptr) {
            std::cout << "Iterating item number " << i << std::endl;
            result = func(current->value, std::forward<Args>(args)...);
            current = current->next;
            i++;
        }
        std::cout << "Exiting loop" << std::endl;
        return result;
    }

    template<typename Func, typename... Args>
    bool iterate_until(Func func, int endPt, Args&&... args) {
        Node<T>* current = head;
        bool result = true;
        int i = 0;
        while (current != nullptr && i <= endPt) {
            std::cout << "Iterating item number " << i << std::endl;
            result = func(current->value, std::forward<Args>(args)...);
            current = current->next;
            i++;
        }
        std::cout << "Exiting loop" << std::endl;
        return result;
    }

    //template<typename T>
    int size() const {
        int count = 0;
        Node<T>* current = head;
        while (current != nullptr) {
            count++;
            current = current->next;
        }
        return count;
    }

    void swapNodes(Node<T>* node1, Node<T>* node2) {
    if (node1 == node2) {
        return;  // No need to swap if the nodes are the same
    }

    // Check if the nodes are adjacent to each other
    if (node1->next == node2) {
        // Node1 is immediately before Node2
        if (node1->prev) {
            node1->prev->next = node2;
        } else {
            head = node2;  // Update head if node1 is the current head
        }

        node2->prev = node1->prev;
        node1->next = node2->next;

        if (node2->next) {
            node2->next->prev = node1;
        } else {
            tail = node1;  // Update tail if node2 is the current tail
        }

        node2->next = node1;
        node1->prev = node2;
    } else if (node2->next == node1) {
        // Node2 is immediately before Node1
        if (node2->prev) {
            node2->prev->next = node1;
        } else {
            head = node1;  // Update head if node2 is the current head
        }

        node1->prev = node2->prev;
        node2->next = node1->next;

        if (node1->next) {
            node1->next->prev = node2;
        } else {
            tail = node2;  // Update tail if node1 is the current tail
        }

        node1->next = node2;
        node2->prev = node1;
    } else {
        // Nodes are not adjacent to each other

        // Update prev and next pointers of adjacent nodes
        if (node1->prev) {
            node1->prev->next = node2;
        } else {
            head = node2;  // Update head if node1 is the current head
        }

        if (node1->next) {
            node1->next->prev = node2;
        } else {
            tail = node2;  // Update tail if node1 is the current tail
        }

        if (node2->prev) {
            node2->prev->next = node1;
        } else {
            head = node1;  // Update head if node2 is the current head
        }

        if (node2->next) {
            node2->next->prev = node1;
        } else {
            tail = node1;  // Update tail if node2 is the current tail
        }

        // Swap prev pointers
        Node<T>* temp = node1->prev;
        node1->prev = node2->prev;
        node2->prev = temp;

        // Swap next pointers
        temp = node1->next;
        node1->next = node2->next;
        node2->next = temp;
    }
}




    void swapNodes(int index1, int index2) {
        if (index1 == index2 || index1 < 0 || index2 < 0) {
            return;  // No need to swap if the indices are the same or invalid
        }

        Node<T>* node1 = getNodeAtIndex(index1);
        Node<T>* node2 = getNodeAtIndex(index2);

        if (!node1 || !node2) {
            return;  // Invalid indices, nodes not found
        }

        swapNodes(node1, node2);
    }


    Node<T>* getNodeAtIndex(int index) {
        if (index < 0) {
            return nullptr;  // Invalid index
        }

        Node<T>* current = head;
        int currentIndex = 0;

        while (current && currentIndex < index) {
            current = current->next;
            currentIndex++;
        }

        return current;
    }

    T getValueAtIndex(int index)
    {
        Node<T>* nodeAtIndex = getNodeAtIndex(index);
        return nodeAtIndex->value;
    }

    void eraseAllNodes() {
    Node<T>* current = head;
    while (current != nullptr) {
        Node<T>* next = current->next;
        delete current;
        current = next;
    }
    head = nullptr;
    tail = nullptr;
    }


    ~DoublyLinkedList() {
    eraseAllNodes();
    }


    
private:
    Node<T>* head;
    Node<T>* tail;
};

#endif // DOUBLYLINKEDLIST_H
