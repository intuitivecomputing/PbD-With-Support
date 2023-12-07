#ifndef DOUBLYLINKEDLIST_H
#define DOUBLYLINKEDLIST_H

#include <iostream>

// Add the Loop structure to hold looping information
struct Loop 
{
    int startIndex;
    int endIndex;
    int loopCount;

    Loop(int start, int end, int count) : startIndex(start), endIndex(end), loopCount(count) {}
};

template<typename T>
struct Node {
    T value;
    Node* prev;
    Node* next;
    Loop* loop;    // Add this pointer to store loop information if the node is part of a loop.
    bool isInLoop; // Add this flag to indicate whether the node is part of a loop or not.
};


template<typename T>
class DoublyLinkedList {
    private:
        Node<T>* head = nullptr;
        Node<T>* tail = nullptr;

    public:
        DoublyLinkedList() : head(nullptr), tail(nullptr) {}
        
        void insert(T value) 
        {
            Node<T>* newNode = new Node<T> { value, tail, nullptr };

            if (tail) 
            {
                tail->next = newNode;
            } 
            else 
            {
                head = newNode;
            }

            tail = newNode;
        }
        
        bool remove(T value) 
        {
            Node<T>* current = head;
            while (current) 
            {
                if (current->value == value) 
                {
                    if (current == head) 
                    {
                        head = current->next;
                    } else 
                    {
                        current->prev->next = current->next;
                    }
                    if (current == tail) 
                    {
                        tail = current->prev;
                    } 
                    else 
                    {
                        current->next->prev = current->prev;
                    }
                    delete current;
                    return true;
                }
                current = current->next;
            }
            return false;
        }

        // Function to update loop indices after a waypoint is deleted.
        void updateLoopIndicesAfterDeletion(int deletedIndex) 
        {
            // Loop through all nodes and their loops to find those that need updating.
            Node<T>* currentNode = head;

            while (currentNode != nullptr) 
            {
                if (currentNode->isInLoop) 
                {
                    Loop* loop = currentNode->loop;

                    int loopSize = (currentNode->loop->endIndex - currentNode->loop->startIndex) + 1;
                    if (loop->startIndex > deletedIndex) 
                    {
                        // Update the start index if it's after the deleted waypoint.
                        loop->startIndex -= 1;
                    }
                    if (loop->endIndex > deletedIndex) 
                    {
                        // Update the end index if it's after the deleted waypoint.
                        loop->endIndex -= 1;
                    }

                    for (int i = 0; i < loopSize; i++)
                    {
                        currentNode = currentNode->next;
                    }
                }
                else
                {
                    currentNode = currentNode->next;
                }
            }
        }

        // Function to update loop indices after a waypoint is deleted.
        void updateLoopIndicesAfterInsertion(int insertIndex) 
        {
            // Loop through all nodes and their loops to find those that need updating.
            Node<T>* currentNode = head;
            while (currentNode != nullptr) 
            {
                if (currentNode->isInLoop) 
                {
                    Loop* loop = currentNode->loop;
                    int loopSize = (currentNode->loop->endIndex - currentNode->loop->startIndex) + 1;
                    if (loop->startIndex > insertIndex) 
                    {
                        // Update the start index if it's after the deleted waypoint.
                        loop->startIndex += 1;
                        loop->endIndex += 1;
                    }
                    for (int i = 0; i < loopSize; i++)
                    {
                        currentNode = currentNode->next;
                    }
                }
                else
                {
                    currentNode = currentNode->next;
                }
            }
        }
        
        bool removeAt(int index) 
        {
            if (index < 0) 
            {
                return false;
            }
            updateLoopIndicesAfterDeletion(index);

            Node<T>* current = head;
            for (int i = 0; i < index && current; i++) 
            {
                current = current->next;
            }
            if (current) 
            {
                if (current == head) 
                {
                    head = current->next;
                } 
                else 
                {
                    current->prev->next = current->next;
                }
                if (current == tail) 
                {
                    tail = current->prev;
                } 
                else 
                {
                    current->next->prev = current->prev;
                }

                delete current;
                return true;
            }

            return false;
        }

        DoublyLinkedList<T> copyUpToIndex(int index) const 
        {
            DoublyLinkedList<T> newList;
            
            // Iterate through the original list up to the specified index
            Node<T>* current = head;
            int count = 0;
            while (current && count <= index) 
            {
                newList.insert(current->value); // Add the current value to the new list
                current = current->next;
                count++;
            }
            
            return newList;
        }

        T getHeadValue() const 
        {
            if (head != nullptr) 
            {
                return head->value;
            }

            // Handle the case when the list is empty
            // For simplicity, assuming T is default-constructible
            return T{};
        }

        Node<T>* getHead() const
        {
            return head;
        }
        
        void move(T value, int newIndex) {
            remove(value);
            Node<T>* current = head;
            for (int i = 0; i < newIndex && current; i++) 
            {
                current = current->next;
            }
            if (current) 
            {
                Node<T>* newNode = new Node<T> { value, current->prev, current };
                if (current->prev) 
                {
                    current->prev->next = newNode;
                } else 
                {
                    head = newNode;
                }
                current->prev = newNode;
            } else 
            {
                insert(value);
            }
        }

        bool move(int oldIndex, int newIndex) 
        {
            if (oldIndex == newIndex || oldIndex < 0 || newIndex < 0) 
            {
                return false;
            }

            updateLoopIndicesAfterInsertion(newIndex); 

            Node<T>* current = head;
            Node<T>* target = nullptr;
            Node<T>* source = nullptr;

            for (int i = 0; i < oldIndex && current; i++) 
            {
                current = current->next;
            }

            if (!current) 
            {
                std::cout << "current is nullptr" << std::endl;
                return false;  // oldIndex out of range
            }
            target = current;

            current = head;
            for (int i = 0; i < newIndex && current; i++) 
            {
                current = current->next;
            }

            if (!current) 
            {
                std::cout << "current is nullptr" << std::endl;
                return false;  // newIndex out of range
            }
            source = current;

            // Update pointers for target node
            if (oldIndex < newIndex) 
            {
                // Shift nodes forward
                if (target->prev) 
                {
                    target->prev->next = target->next;
                } 
                else 
                {
                    head = target->next;
                }
                if (target->next) 
                {
                    target->next->prev = target->prev;
                } 
                else 
                {
                    tail = target->prev;
                }

                target->prev = source;
                target->next = source->next;
                source->next = target;

                if (target->next) 
                {
                    target->next->prev = target;
                } 
                else 
                {
                    tail = target;
                }
            } 
            else 
            {
                // Shift nodes backward
                if (target->prev) 
                {
                    target->prev->next = target->next;
                } 
                else 
                {
                    head = target->next;
                }

                if (target->next) 
                {
                    target->next->prev = target->prev;
                } 
                else 
                {
                    tail = target->prev;
                }

                target->next = source;
                target->prev = source->prev;
                source->prev = target;
                if (target->prev) 
                {
                    target->prev->next = target;
                } else 
                {
                    head = target;
                }
            }

            return true;
        }


        bool replace(int index, const T& value) 
        {
            Node<T>* current = head;
            for (int i = 0; i < index && current; i++) {
                current = current->next;
            }

            if (!current) 
            {
                return false;
            }

            current->value = value;
            return true;
        }

        template<typename Func, typename... Args>
        bool iterate(Func func, Args&&... args) 
        {
            Node<T>* current = head;
            bool result = true;

            int i = 1;
            while (current != nullptr) 
            {
                result = func(current->value, std::forward<Args>(args)...);
                current = current->next;
                i++;
            }

            return result;
        }

        template<typename Func, typename... Args>
        bool iterate_until(Func func, int endPt, Args&&... args) 
        {
            Node<T>* current = head;
            bool result = true;

            int i = 0;
            while (current != nullptr && i <= endPt) 
            {
                result = func(current->value, std::forward<Args>(args)...);
                current = current->next;
                i++;
            }

            return result;
        }

        int size() const 
        {
            int count = 0;
            Node<T>* current = head;
            while (current != nullptr) 
            {
                count++;
                current = current->next;
            }
            return count;
        }

        void swapNodes(Node<T>* node1, Node<T>* node2) 
            {
            if (node1 == node2) 
            {
                return;  // No need to swap if the nodes are the same
            }

            // Check if the nodes are adjacent to each other
            if (node1->next == node2) 
            {
                // Node1 is immediately before Node2
                if (node1->prev) 
                {
                    node1->prev->next = node2;
                } 
                else 
                {
                    head = node2;  // Update head if node1 is the current head
                }

                node2->prev = node1->prev;
                node1->next = node2->next;

                if (node2->next) 
                {
                    node2->next->prev = node1;
                } 
                else 
                {
                    tail = node1;  // Update tail if node2 is the current tail
                }

                node2->next = node1;
                node1->prev = node2;
            } 
            else if (node2->next == node1) 
            {
                // Node2 is immediately before Node1
                if (node2->prev) 
                {
                    node2->prev->next = node1;
                } 
                else 
                {
                    head = node1;  // Update head if node2 is the current head
                }

                node1->prev = node2->prev;
                node2->next = node1->next;

                if (node1->next) 
                {
                    node1->next->prev = node2;
                } 
                else 
                {
                    tail = node2;  // Update tail if node1 is the current tail
                }

                node1->next = node2;
                node2->prev = node1;
            } 
            else 
            {
                // Nodes are not adjacent to each other
                // Update prev and next pointers of adjacent nodes
                if (node1->prev) 
                {
                    node1->prev->next = node2;
                } 
                else 
                {
                    head = node2;  // Update head if node1 is the current head
                }

                if (node1->next) 
                {
                    node1->next->prev = node2;
                } else 
                {
                    tail = node2;  // Update tail if node1 is the current tail
                }

                if (node2->prev) 
                {
                    node2->prev->next = node1;
                } else 
                {
                    head = node1;  // Update head if node2 is the current head
                }

                if (node2->next) 
                {
                    node2->next->prev = node1;
                } 
                else 
                {
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

        void swapNodes(int index1, int index2) 
        {
            if (index1 == index2 || index1 < 0 || index2 < 0) 
            {
                return;  // No need to swap if the indices are the same or invalid
            }

            Node<T>* node1 = getNodeAtIndex(index1);
            Node<T>* node2 = getNodeAtIndex(index2);

            if (!node1 || !node2) 
            {
                return;  // Invalid indices, nodes not found
            }

            swapNodes(node1, node2);
        }


        Node<T>* getNodeAtIndex(int index) 
        {
            if (index < 0) 
            {
                return nullptr;  // Invalid index
            }

            Node<T>* current = head;
            int currentIndex = 0;

            while (current && currentIndex < index) 
            {
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

        void eraseAllNodes() 
        {
            Node<T>* current = head;
            while (current != nullptr) 
            {
                Node<T>* next = current->next;
                delete current;
                current = next;
            }

            head = nullptr;
            tail = nullptr;
        }

        void insertLoop(int startIndex, int endIndex, int loopCount) 
        {
            // Check if a loop already exists with the same start and end indices.
            Node<T>* node = getNodeAtIndex(startIndex);
            if (node != nullptr && node->isInLoop && node->loop->endIndex == endIndex) 
            {
                // A loop with the same start and end indices already exists.
                // Call changeLoopIterations to update the loop's iteration count.
                changeLoopIterations(startIndex, endIndex, loopCount);
            } 
            else 
            {
                // A new loop is being inserted.
                Loop* loop = new Loop(startIndex, endIndex, loopCount); // Create a new loop structure.

                // Loop through the nodes and set the isInLoop flag for nodes within the loop range.
                for (int i = startIndex; i <= endIndex; ++i) 
                {
                    Node<T>* node = getNodeAtIndex(i);
                    if (node != nullptr) 
                    {
                        node->isInLoop = true;
                        node->loop = loop;
                    }
                }
            }
        }

        bool deleteLoop(int startIndex, int endIndex) 
        {
            // Ensure that the start and end indices are valid.
            if (startIndex < 0 || endIndex >= size() || startIndex > endIndex) 
            {
                return false;
            }

            // Check if any node within the loop range is not part of a loop.
            // If so, it means there is a non-contiguous loop, and we cannot delete it.
            for (int i = startIndex; i <= endIndex; ++i) 
            {
                Node<T>* node = getNodeAtIndex(i);
                if (node != nullptr && !node->isInLoop) {
                    return false;
                }
            }

            bool isDeleted = false;

            // Now, remove the loop flag for all nodes within the loop range.
            for (int i = startIndex; i <= endIndex; ++i) 
            {
                Node<T>* node = getNodeAtIndex(i);
                if (node != nullptr) 
                {
                    if (!isDeleted)
                    {
                        delete node->loop;
                        isDeleted = true;
                    }
                    node->isInLoop = false;
                    node->loop = nullptr;
                }
            }

            return true;
        }

        bool changeLoopIterations(int startIndex, int endIndex, int newLoopCount) 
        {
            // Ensure that the start and end indices are valid.
            if (startIndex < 0 || endIndex >= size() || startIndex > endIndex) 
            {
                return false;
            }

            // Check if all nodes within the loop range are part of the same loop.
            // If not, it means there is a non-contiguous loop, and we cannot modify it.
            Loop* loop = nullptr;
            for (int i = startIndex; i <= endIndex; ++i) 
            {
                Node<T>* node = getNodeAtIndex(i);
                if (node != nullptr) 
                {
                    if (!node->isInLoop || node->loop == nullptr) 
                    {
                        return false;
                    }
                    if (loop == nullptr) 
                    {
                        loop = node->loop;
                    } 
                    else if (loop != node->loop) 
                    {
                        return false;
                    }
                }
            }

            // Update the loop count for the specified loop.
            loop->loopCount = newLoopCount;
            return true;
        }

        ~DoublyLinkedList() 
        {
            eraseAllNodes();
        }
};

#endif // DOUBLYLINKEDLIST_H
