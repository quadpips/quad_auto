#pragma once

#include <mmp_quadruped/planning/Graph.h>
#include <mmp_quadruped/planning/Heap.h>
#include <mmp_quadruped/planning/TorsoPathPlanner.h>


namespace mmp {

/****************************GraphSearcher****************************/
class GraphSearcher
{
    public:
        GraphSearcher(const rclcpp::Node::SharedPtr& node);

        /**
        * @brief This is the master search function which parses the user-specified search specifications 
        * (Dijkstras or A*, mode family search or mode search) and then runs the corresponding search help function. 
        * @param graph : pointer to the graph object we are searching over
        * @param costFormula : short which specifies search algorithm (0 - Dijkstra's, 1 - A*) 
        *
        * @return bool : true if search was successful, false otherwise
        */    
        bool search(Graph * graph, 
                    const short & costFormula,
                    const std::vector<TorsoStateNode::TorsoTransition *> & torsoPath);

        /**
        * @brief Getter for the graph search time
        *
        * @return int64_t time it takes to run the graph search
        */
        int64_t getGraphSearchTime() { return graphSearchTime; }

    private:
        /**
        * This is the search function in which we only search over mode families.
        * This search will only return a sequence of mode families (i.e., a sequence of end effector-object contact combinations)
        * @param graph : pointer to the graph object we are searching over
        * @param costFormula : short which specifies search algorithm (0 - Dijkstra's, 1 - A* (default) )
        * @return ModeFamilyNode * : pointer to the mode family node to trace backwards from
        */
        // Camera * camera,
        ModeFamilyNode * ModeFamilySearch(Graph * graph, const short & costFormula, 
                                            const std::vector<TorsoStateNode::TorsoTransition *> & torsoPath);

        rclcpp::Node::SharedPtr node_;

        int64_t graphSearchTime = 0; /**< time it takes to run the graph search */
        double maxSearchTime_ = 1.0; /**< maximum time allowed for the graph search */
};

}  // namespace mmp
 