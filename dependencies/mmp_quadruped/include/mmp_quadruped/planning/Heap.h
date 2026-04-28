#pragma once

#include <unordered_map>

namespace mmp {

template <class T> class Heap
{
    // TODO: add ROS2 node ptr to constructor for logging
    public:
        /**
        * @brief Constructs a heap with the starting root node
        *
        * @param gNode The node that should be placed at the root of the heap.
        */
        Heap(T& gNode) 
        { 
            push(gNode); 
        }

        /**
        * @brief Removes the element with highest priority according to the
        * higherPriority() function, then restores the heap property
        * through heapifyDown
        *
        * @return The element with highest priority in the heap.
        */
        T * pop()
        {
            if (empty())
                return NULL;

            if (elems_.size() != hash_table.size())
                throw std::runtime_error("                  heap pop: before pop, elems is size: " + std::to_string(elems_.size()) + 
                                                            " but hash table is size " + std::to_string(hash_table.size()));

            T * minValNode = elems_[root()];
            T * lastNode = elems_[elems_.size() - 1];
            elems_[root()] = lastNode; 

            std::string lastNodeKey = getHashKey(*lastNode);
            auto it = hash_table.find(lastNodeKey);
            if (it != hash_table.end())
            {
                // std::cout << "                      updating hash_table[lastNodeKey] to: " << 0 << std::endl;
                it->second = 0; // moving node up to root
            } else    
            {
                throw std::runtime_error("                      hash_table[lastNodeKey] does not exist");
            }
            // need to update hash_table[root()] to 0

            elems_.pop_back();

            // std::cout << "                  after pop, elems size: " << elems_.size() << std::endl;

            std::string minValKey = getHashKey(*minValNode);

            // std::cout << "                  before erase, hash_table size: " << hash_table.size() << std::endl;

            hash_table.erase(minValKey);

            // std::cout << "                  after erase, hash_table size: " << hash_table.size() << std::endl;

            if (elems_.size() != hash_table.size())
                throw std::runtime_error("                  heap pop: after pop, elems is size: " + std::to_string(elems_.size()) + 
                                                            " but hash table is size " + std::to_string(hash_table.size()));

            heapifyDown(root());

            // std::cout << "              <----------[pop]" << std::endl;

            return minValNode;
        }

        /**
        * @brief Inserts the given element into the heap, then restores the heap
        * property through heapifyUp
        *
        * @param gNode The node that is being added to the heap.
        */
        void push(T& gNode)
        {
            // std::cout << "              [push()]---------->" << std::endl;

            if (elems_.size() != hash_table.size())
                throw std::runtime_error("              heap push: before push, elems is size: " + std::to_string(elems_.size()) + 
                                                        " but hash table is size " + std::to_string(hash_table.size()));

            std::string key = getHashKey(gNode); 
            // if (key == theKey) std::cout << "theKey IN PUSH" << std::endl;
            // std::cout << "                  key: " << key << std::endl;
            // std::cout << "                  cost: " << getNodeValue(&gNode) << std::endl;
            // std::cout << "                  value: " << elems_.size() << std::endl;

            // std::cout << "                  before insert, hash_table size: " << hash_table.size() << std::endl;

            if (hash_table.find(key) != hash_table.end())
                throw std::runtime_error("                  heap push: key is already in heap, should not be re-added.");

            hash_table.insert( {key, elems_.size()});
            // hash_table[key] = elems_.size(); // supposed to happen

            // std::cout << "                  after insert, hash_table size: " << hash_table.size() << std::endl;
            // if (hash_table[key] == 0)
            //     throw std::runtime_error("heap push: hash_table[key] = 0");

            // std::cout << "                  before push_back, elems_ size: " << elems_.size() << std::endl;

            elems_.push_back(&gNode);

            // std::cout << "                  after push_back, elems_ size: " << elems_.size() << std::endl;

            heapifyUp(elems_.size() - 1);

            // std::cout << "              after heapifyUp, elems_ size: " << elems_.size() << std::endl;


            if (elems_.size() != hash_table.size())
                throw std::runtime_error("                  heap push: after push, elems is size: " + std::to_string(elems_.size()) + 
                                                            " but hash table is size " + std::to_string(hash_table.size()));

            // std::cout << "finished push" << std::endl;
            // std::cout << "              <----------[push]" << std::endl;
        }

        /**
        * @brief Searches through the heap to find the index of the given node.
        *
        * @param gNode The node to be found in the heap.
        * @return The index of the given node in the heap.
        */
        size_t find(T& gNode)
        {
            // std::cout << "              [find()]---------->" << std::endl;
            
            // std::cout << "                  elems_ size: " << elems_.size() << std::endl;

            // std::cout << "                  hash_table size: " << hash_table.size() << std::endl;

            // std::cout << "                  elems_: " << std::endl;
            // for (int i = 0; i < elems_.size(); i++)
            //     std::cout << "                      " << i << ": " << getHashKey(*elems_[i]) << std::endl;

            // std::cout << "                  hash_table: " << std::endl;
            int counter = 0;
            for (auto kv : hash_table) 
            {
                // std::cout << "                      " << counter << ": (" << kv.first << ", " << kv.second << ")" << std::endl;
                counter++;
            }
            if (elems_.size() != hash_table.size())
                throw std::runtime_error("                  heap find: elems and hash table are not same size");

            std::string key = getHashKey(gNode); 

            // std::cout << "                  key: " << key << std::endl;
            // if (key == theKey)
            //     std::cout << "theKey in find" << std::endl;

            // std::cout << "before find operation" << std::endl;
            // hash_table.find(key);
            // std::cout << "after find operation" << std::endl;

            if (hash_table.find(key) == hash_table.end())
                throw std::runtime_error("                  heap find: after push, could not find requested key");
            else
                return hash_table[key];
        }

        /**
        * @brief Returns the element at the provided index of the heap array.
        *
        * @param idx The index of the element to be returned. 
        * (Remember this is zero-indexed by default)
        * @return The element at the provided index.
        */
        T * getElem(const size_t & idx)
        {
            return elems_[idx];
        }

        /**
        * @brief Updates the element at the provided index of the heap array.
        * The update is done in such a way that the array will be 
        * corrected, so it will remain as a valid heap.
        *
        * @param idx The index of the node which we must update. 
        * @param value The value to update the node with.
        */
        void updateElem(const size_t & idx, const double & value)
        {
            // std::cout << "              [updateElem()]---------->" << std::endl;
            // std::cout << "                  idx: " << idx << std::endl;
            
            if (idx < 0)
                throw std::runtime_error("                  heap updateElem: index < 0");

            if (idx >= elems_.size())
                throw std::runtime_error("                  heap updateElem: index >= elems_ size");

            T * gNode = elems_[idx];

            // std::cout << "                  prior costToCome: " << gNode->getCostToCome() << std::endl;
            // std::cout << "                  updated costToCome: " << value << std::endl;

            if (getNodeValue(gNode) <= value)  
                throw std::runtime_error("                  updating element with higher cost");

            gNode->setCostToCome(value);

            heapifyUp(idx);
            heapifyDown(idx);
            // std::cout << "              <----------[updateElem]" << std::endl;
        }

        /**
        * @brief Determines if the given heap is empty.
        *
        * @return Whether or not there are elements in the heap.
        */
        bool empty()
        {
            return elems_.empty();
        }

    private:

        /**
        * @brief Helper function that returns the root index of this heap.
        *
        * @return The index of the root node of the heap.
        */
        size_t root()
        {
            return 0;
        }

        /**
        * @brief Helper function that returns the index of the left child of a
        * node in the heap.
        *
        * @param currentIdx The index of the current node.
        * @return The index of the left child of the current node.
        */
        size_t leftChild(const size_t & currentIdx)
        {
            return (2 * currentIdx) + 1;
        }

        /**
        * @brief Helper function that returns the index of the right child of a
        * node in the heap.
        *
        * @param currentIdx The index of the current node.
        * @return The index of the right child of the current node.
        */
        size_t rightChild(const size_t & currentIdx)
        {
            return (2*currentIdx) + 2;
        }

        /**
        * @brief Helper function that returns the index of the parent of a node
        * in the heap.
        *
        * @param currentIdx The index of the current node.
        * @return The index of the parent of the current node.
        */
        size_t parent(const size_t & currentIdx)
        {
            return ((currentIdx - 1) / 2);
        }

        /**
        * @brief Helper function that determines whether a given node has a
        * child.
        *
        * @param currentIdx The index of the current node.
        * @return A boolean indicating whether the current node has a
        *  child or not.
        */
        bool hasAChild(const size_t & currentIdx)
        {
            return (leftChild(currentIdx) < elems_.size());
        }

        /**
        * @brief Helper function that returns the index of the child with the
        * highest priority as defined by the higherPriority() functor.
        *
        * For example, if T == int and the left child of the current node
        * has data 5 and the right child of the current node has data 9,
        * this function should return the index of the left child (because
        * the default higherPriority() behaves like operator<).
        *
        * This function assumes that the current node has children.
        *
        * @param currentIdx The index of the current node.
        * @return The index of the highest priority child of this node.
        */
        size_t highestPriorityChild(const size_t & currentIdx)
        {
            // std::cout << "                      [highestPriorityChild()]---------->" << std::endl;

            size_t leftIdx = leftChild(currentIdx); 
            size_t rightIdx = rightChild(currentIdx);
            
            // std::cout << "                          leftIdx: " << leftIdx << std::endl;
            // std::cout << "                          rightIdx: " << rightIdx << std::endl;

            if (rightIdx >= elems_.size()) 
                return leftIdx;

            return higherPriority(elems_[leftIdx], elems_[rightIdx]) ? leftIdx : rightIdx;
        }

        /**
        * @brief Helper function that restores the heap property by sinking a
        * node down the tree as necessary.
        *
        * @param currentIdx The index of the current node that is being
        *  sunk down the tree.
        */
        void heapifyDown(const size_t & currentIdx)
        {
            // std::cout << "                  [heapifyDown()]---------->" << std::endl;
            // std::cout << "                      currentIdx: " << currentIdx << std::endl;

            // if (currentIdx == 0)
            //     throw std::runtime_error("heapifyDown: currentIdx == 0");


            if (hasAChild(currentIdx)) 
            {
                size_t minChildIndex = highestPriorityChild(currentIdx);
                // std::cout << "                      minChildIndex: " << minChildIndex << std::endl;
            
                // if (minChildIndex == 0)
                //     throw std::runtime_error("heapifyDown: minChildIndex == 0");

                if (higherPriority(elems_[minChildIndex], elems_[currentIdx])) 
                {
                    // std::cout << "          swapping" << std::endl;
                    std::string minChildKey = getHashKey(minChildIndex);                   
                    std::string currentKey = getHashKey(currentIdx);
                    // std::cout << "                      minChildKey: " << minChildKey << std::endl;
                    // std::cout << "                      currentKey: " << currentKey << std::endl;                
                    // size_t minChildHeapIdx = hash_table[minChildKey]; // = 0. HOW
                    // size_t currentHeapIdx = hash_table[currentKey];

                    // std::cout << "          minChildHeapIdx: " << minChildHeapIdx << std::endl;
                    // std::cout << "          currentHeapIdx: " << currentHeapIdx << std::endl;

                    // if (minChildIndex == 0)
                    //     throw std::runtime_error("heapifyDown: minChildHeapIdx == 0");
                    // if (currentIdx == 0)
                    //     throw std::runtime_error("heapifyDown: currentIdx == 0");

                    auto it = hash_table.find(minChildKey);
                    if (it != hash_table.end())
                    {
                        // std::cout << "                      updating hash_table[minChildKey] to: " << currentIdx << std::endl;
                        it->second = currentIdx;
                    } else    
                    {
                        throw std::runtime_error("                      hash_table[minChildKey] does not exist");
                    }
                    // hash_table[minChildKey] = currentHeapIdx;

                    auto it2 = hash_table.find(currentKey);
                    if (it2 != hash_table.end())
                    {
                        // std::cout << "                      updating hash_table[currentKey] to: " << minChildIndex << std::endl;                
                        it2->second = minChildIndex;
                    } else    
                    {
                        throw std::runtime_error("                      hash_table[currentKey] does not exist");                
                    }
                    // hash_table[currentKey] = minChildHeapIdx;

                    // if (hash_table[minChildKey] == 0)
                    //     throw std::runtime_error("hash_table[minChildKey] == 0");

                    // if (hash_table[currentKey] == 0)
                    //     throw std::runtime_error("hash_table[currentKey] == 0");

                    // std::cout << "                      swapping" << std::endl;                

                    std::swap(elems_[minChildIndex], elems_[currentIdx]);
                    heapifyDown(minChildIndex);
                }
            }
            // std::cout << "                      heap is correct" << std::endl;
            // std::cout << "                  <----------[heapifyDown]" << std::endl;
        }

        /**
        * @brief Helper function that restores the heap property by bubbling a
        * node up the tree as necessary.
        *
        * @param currentIdx The index of the current node that is being
        *  bubbled up the top
        */
        void heapifyUp(const size_t & currentIdx)
        {
            // std::cout << "                  [heapifyUp()]---------->" << std::endl;
            // std::cout << "                      currentIdx: " << currentIdx << std::endl;

            if (currentIdx != root()) 
            {
                size_t parentIdx = parent(currentIdx);

                // if (parentIdx == 0)
                //     throw std::runtime_error("heapifyUp: parentIdx == 0");

                if (higherPriority(elems_[currentIdx], elems_[parentIdx])) 
                {
                    // std::cout << "          swapping" << std::endl;            
                    std::string parentKey = getHashKey(parentIdx);                          
                    std::string currentKey = getHashKey(currentIdx);
                    // std::cout << "                      parentKey: " << parentKey << std::endl;
                    // std::cout << "                      currentKey: " << currentKey << std::endl;                     
                    size_t parentHeapIdx = hash_table[parentKey];
                    size_t currentHeapIdx = hash_table[currentKey];

                    // std::cout << "                      parentHeapIdx: " << parentHeapIdx << std::endl;
                    // std::cout << "                      currentHeapIdx: " << currentHeapIdx << std::endl;

                    // if (parentHeapIdx == 0)
                    //     throw std::runtime_error("heapifyUp: parentHeapIdx == 0");
                    // if (currentHeapIdx == 0)
                    //     throw std::runtime_error("heapifyUp: currentHeapIdx == 0");

                    auto it = hash_table.find(parentKey);
                    if (it != hash_table.end())
                    {
                        // std::cout << "                      updating hash_table[parentKey] to: " << currentHeapIdx << std::endl;
                        it->second = currentHeapIdx;
                    } else    
                    {
                        throw std::runtime_error("                      hash_table[parentKey] does not exist");
                    }
                    // hash_table[parentKey] = currentHeapIdx;

                    auto it2 = hash_table.find(currentKey);
                    if (it2 != hash_table.end())
                    {
                        // std::cout << "                      updating hash_table[currentKey] to: " << parentHeapIdx << std::endl;
                        it2->second = parentHeapIdx;
                    } else    
                    {
                        throw std::runtime_error("                      hash_table[currentKey] does not exist");
                    }
                    // hash_table[currentKey] = parentHeapIdx; 

                    // std::cout << "                      swapping" << std::endl;                

                    std::swap(elems_[currentIdx], elems_[parentIdx]);
                    heapifyUp(parentIdx);
                }
            }
            // std::cout << "                      heap is correct" << std::endl;
            // std::cout << "                  <----------[heapifyUp]" << std::endl;
        }

        /**
        * @brief Returns the value of the given node in the heap.
        *
        * @param gNode The node whose value we want to retrieve.
        * @return The value of the given node.
        */
        double getNodeValue(T * gNode)
        {
            return gNode->getCostToCome() + gNode->getSearchAlgorithm() * gNode->getCostToGo();
        }

        /**
        * @brief Helper function that determines if the left node has higher
        * priority than the right node.
        *
        * @param left The left node to compare.
        * @param right The right node to compare.
        * @return Whether or not the left node has higher priority.
        */
        bool higherPriority(T * left, T * right)
        {
            // std::cout << "                      [higherPriority()]---------->" << std::endl;

            // std::cout << "                          left value: " << getNodeValue(left) << std::endl;
            // std::cout << "                          right value: " << getNodeValue(right) << std::endl;
            bool priority = getNodeValue(left) < getNodeValue(right);
            // std::cout << "                          left < right: " << priority << std::endl;

            // std::cout << "      [higherPriority]: " << std::endl;
            // std::cout << "      " << (left->getCostToCome() + 10.0 * left->getSearchAlgorithm() * left->getCostToGo()) << std::endl;
            // std::cout << "      " << (right->getCostToCome() + 10.0 * right->getSearchAlgorithm() * right->getCostToGo()) << std::endl;

            // std::cout << "                      <----------[higherPriority]" << std::endl;

            return priority;
        }

        /**
        * @brief Returns the hash key for the given index.
        *
        * @param idx The index of the node whose key we want to retrieve.
        * @return The hash key of the given index.
        */
        std::string getHashKey(const size_t & idx)
        {
            return getHashKey(*elems_[idx]);
        }

        /**
        * @brief Returns the hash key for the given node.This function uses the node's 
        * getCoparamInds function here to uniqely identify from coparameters
        *
        * @param gNode The node whose key we want to retrieve.
        * @return The hash key of the given node.
        */
        std::string getHashKey(T& gNode)
        {
            std::string hashKey = "";
            for (int l = 0; l < 4; l++)
            {
                hashKey += gNode.getModeFamily()->getHashID(l) + "_";
                
                // hashKey += (std::to_string(gNode.getModeFamily()->X(l)) + "_" + std::to_string(gNode.getModeFamily()->Y(l)) + "_");
                // hashKey += vec2string(gNode.getCoparamInds(l)) + (l < 3 ? "_" : "");
            }
            return hashKey;
        }

        std::vector<T *> elems_; /**< The internal storage for this heap. This heap is 0-based, meaning that the root is stored at index 0 of this vector. */

        std::unordered_map<std::string, size_t> hash_table; /**< Hash table that takes in a unique node key and returns the value of the node's index in elems_ */
};

}  // namespace mmp


// #endif