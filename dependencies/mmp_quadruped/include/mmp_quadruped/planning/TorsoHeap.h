#pragma once

#include <unordered_map>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ocs2_switched_model_interface/core/SwitchedModel.h>

#include <mmp_quadruped/planning/utils.h>

namespace mmp {

template <class T> class TorsoHeap
{
    public:
        /**
        * @brief Constructs a heap with the starting root node
        *
        * @param tNode The node that should be placed at the root of the heap.
        */
        TorsoHeap(T& tNode)
        {
            push(tNode);
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

            // std::cout << "      [pop()]" << std::endl;

            if (elems_.size() != hash_table.size())
                throw std::runtime_error("heap pop: before pop, elems is size: " + std::to_string(elems_.size()) + 
                                        " but hash table is size " + std::to_string(hash_table.size()));

            T * minValNode = elems_[root()];
            T * lastNode = elems_[elems_.size() - 1];
            elems_[root()] = lastNode;

            std::string lastNodeKey = getHashKey(*lastNode);
            auto it = hash_table.find(lastNodeKey);
            if (it != hash_table.end())
                it->second = 0;
            else    
                throw std::runtime_error("hash_table[lastNodeKey] does not exist");

            elems_.pop_back();

            std::string key = getHashKey(*minValNode);
            // std::cout << "key: " << key << std::endl;

            // std::cout << "before erase" << std::endl;
            hash_table.erase(key);

            // T * minValNode = minVal->tNode; 
            // delete minVal;

            if (elems_.size() != hash_table.size())
                throw std::runtime_error("heap pop: after pop, elems is size: " + std::to_string(elems_.size()) + 
                                        " but hash table is size " + std::to_string(hash_table.size()));

            heapifyDown(root());

            return minValNode;
        }

        /**
        * @brief Inserts the given element into the heap, then restores the heap
        * property through heapifyUp
        *
        * @param tNode The node that is being added to the heap.
        */
        void push(T& tNode)
        {
            // std::cout << "push" << std::endl;
            // bin_node<T>* n = new bin_node<T>;
            // n->tNode = &tNode; 
            // n->value = tNode.getCostToCome() + 10.0 * tNode.getSearchAlgorithm() * tNode.getCostToGo(); // value; // should be costToCome of ModeNode

            if (elems_.size() != hash_table.size())
                throw std::runtime_error("heap push: before push, elems is size: " + std::to_string(elems_.size()) + 
                                        " but hash table is size " + std::to_string(hash_table.size()));

            std::string key = getHashKey(tNode); 
            

            if (hash_table.find(key) != hash_table.end())
                throw std::runtime_error("torso heap push: key is already in heap, should not be re-added.");

            // std::cout << "key: " << key << std::endl;

            hash_table.insert( {key, elems_.size()});

            elems_.push_back(&tNode);
            heapifyUp(elems_.size() - 1);

            if (elems_.size() != hash_table.size())
                throw std::runtime_error("heap push: after push, elems is size: " + std::to_string(elems_.size()) + 
                                        " but hash table is size " + std::to_string(hash_table.size()));

            // std::cout << "finished push" << std::endl;
        }

        /**
        * @brief Searches through the heap to find the index of the given node.
        *
        * @param tNode The node to be found in the heap.
        * @return The index of the given node in the heap.
        */
        int find(T& tNode)
        {
            // std::cout << "[find:]" << std::endl;
            
            std::string key = getHashKey(tNode); 

            // std::cout << "before find operation" << std::endl;
            hash_table.find(key);
            // std::cout << "after find operation" << std::endl;

            if (hash_table.find(key) == hash_table.end())
                throw std::runtime_error("heap find: after push, could not find requested key");
            else
                return hash_table[key];

            // std::cout << "finished find" << std::endl;        
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
        void updateElem(const int & idx, const double & value)
        {
            if (idx < 0 || idx >= elems_.size())
                throw std::runtime_error("heap updateElem: index out of bounds");

            T * tNode = elems_[idx];
            tNode->setCostToCome(value);
            // n->value = n->tNode->getCostToCome() + 10.0 * n->tNode->getSearchAlgorithm() * n->tNode->getCostToGo();
            
            // how do we do indices?
            // idx = find(n);
            //

            heapifyUp(idx);
            heapifyDown(idx);
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
            size_t left = leftChild(currentIdx); 
            size_t right = rightChild(currentIdx);
            
            if (right >= elems_.size()) 
                return left;

            return higherPriority(elems_[left], elems_[right]) ? left : right;
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
            // std::cout << "      [heapifyDown()]:" << std::endl;
            if (hasAChild(currentIdx)) 
            {
                size_t minChildIndex = highestPriorityChild(currentIdx);
                if (higherPriority(elems_[minChildIndex], elems_[currentIdx])) 
                {
                    // std::cout << "          swapping" << std::endl;
                    std::string minChildKey = getHashKey(minChildIndex);                   
                    std::string currentKey = getHashKey(currentIdx);
                    // std::cout << "          minChild: " << minChildKey << std::endl;
                    // std::cout << "          current: " << currentKey << std::endl;                
                    auto it = hash_table.find(minChildKey);
                    if (it != hash_table.end())
                        it->second = currentIdx;
                    else    
                        throw std::runtime_error("hash_table[minChildKey] does not exist");
                    
                    auto it2 = hash_table.find(currentKey);
                    if (it2 != hash_table.end())
                        it2->second = minChildIndex;
                    else    
                        throw std::runtime_error("hash_table[currentKey] does not exist");                
                    
                    std::swap(elems_[minChildIndex], elems_[currentIdx]);
                    heapifyDown(minChildIndex);
                }
            }
        }

        /**
        * @brief Helper function that restores the heap property by bubbling a
        * node up the tree as necessary.
        *
        * @param currentIdx The index of the current node that is being
        *  bubbled up th
        */
        void heapifyUp(const size_t & currentIdx)
        {
            // std::cout << "      [heapifyUp()]:" << std::endl;
            if (currentIdx != root()) 
            {
                size_t parentIdx = parent(currentIdx);
                if (higherPriority(elems_[currentIdx], elems_[parentIdx])) 
                {
                    // std::cout << "          swapping" << std::endl;            
                    std::string parentKey = getHashKey(parentIdx);                          
                    std::string currentKey = getHashKey(currentIdx);                
                    
                    size_t parentHeapIdx = hash_table[parentKey];
                    size_t currentHeapIdx = hash_table[currentKey];

                    // std::cout << "          parent: " << parentKey << std::endl;
                    // std::cout << "          current: " << currentKey << std::endl;                     
                    auto it = hash_table.find(parentKey);
                    if (it != hash_table.end())
                        it->second = currentHeapIdx;
                    else    
                        throw std::runtime_error("hash_table[parentKey] does not exist");
                    
                    auto it2 = hash_table.find(currentKey);
                    if (it2 != hash_table.end())
                        it2->second = parentHeapIdx;
                    else    
                        throw std::runtime_error("hash_table[currentKey] does not exist");

                    std::swap(elems_[currentIdx], elems_[parentIdx]);
                    heapifyUp(parentIdx);
                }
            }
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
            // std::cout << "      [higherPriority]: " << std::endl;
            // std::cout << "      " << (left->getCostToCome() + 10.0 * left->getSearchAlgorithm() * left->getCostToGo()) << std::endl;
            // std::cout << "      " << (right->getCostToCome() + 10.0 * right->getSearchAlgorithm() * right->getCostToGo()) << std::endl;
            return (left->getCostToCome() + left->getCostToGo()) <
                    (right->getCostToCome() + right->getCostToGo());
        }

        /**
        * @brief Returns the hash key for the given index.
        *
        * @param idx The index of the node whose key we want to retrieve.
        * @return The hash key of the given index.
        */
        std::string getHashKey(const int & idx)
        {
            return getHashKey(*elems_[idx]);
        }

        /**
        * @brief Returns the hash key for the given node.This function uses the node's 
        * getCoparamInds function here to uniqely identify from coparameters
        *
        * @param tNode The node whose key we want to retrieve.
        * @return The hash key of the given node.
        */
        std::string getHashKey(T& tNode)
        {
            switched_model::base_coordinate_t tNodePose = tNode.getPoseWorldFrame();
            switched_model::vector3_t tNodePosition = extractTorsoPosition(tNodePose);
            switched_model::vector3_t tNodeOrientation = extractTorsoOrientation(tNodePose);
            return std::to_string(tNodePosition[0]) + "_" + std::to_string(tNodePosition[1]) + "_" + std::to_string(tNodePosition[2]) + 
                   "_" + std::to_string(tNodeOrientation[0]) + "_" + std::to_string(tNodeOrientation[1]) + "_" + std::to_string(tNodeOrientation[2]);                         
        }

        std::vector<T *> elems_; /**< The internal storage for this heap. This heap is 0-based, meaning that the root is stored at index 0 of this vector. */

        std::unordered_map<std::string, int> hash_table; /**< Hash table that takes in a unique node key and returns the value of the node's index in elems_ */

};

}  // namespace mmp

