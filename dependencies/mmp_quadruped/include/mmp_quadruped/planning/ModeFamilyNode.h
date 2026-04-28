#pragma once

#include <mmp_quadruped/planning/ModeFamily.h>
#include <mmp_quadruped/planning/TorsoPathPlanner.h>

#include <ocs2_oc/oc_data/PrimalSolution.h>


namespace mmp {

/****************************ModeFamilyNode****************************/
class ModeFamilyNode
{
    public:
        /****************************Transition****************************/
        class Transition
        {
            public:
                /**
                * @brief Construct a new Transition object
                * 
                * @param nFrom The node from which the transition is made
                * @param nTo The node to which the transition is made
                * @param torsoPath The torso path for the transition
                * @param w_exp The weight for the experience term
                * @param w_euc The weight for the euclidean term
                * @param w_stance The weight for the stance deviation term
                * @param w_step The weight for the steppability term
                * @param w_torso The weight for the torso path deviation term
                */
                Transition(ModeFamilyNode * nFrom, 
                            ModeFamilyNode * nTo, 
                            const std::vector<TorsoStateNode::TorsoTransition *> & torsoPath,
                            const double & w_exp, 
                            const double & w_euc, 
                            const double & w_stance, 
                            const double & w_step, 
                            const double & w_torso);
                
                Transition(ModeFamilyNode * nFrom, ModeFamilyNode * nTo);

                /**
                * @brief Destroy the Transition object
                */
                ~Transition();

                // Experience
                static const short expSize = TRANSITION_DISCR_FACTOR + 1; /**< The size of one dimension of the experience distributions */
                static constexpr double expStep = 1.0f / TRANSITION_DISCR_FACTOR; /**< The step size for the experience distributions */

                /*************************Mode Family Weight*************************/

                /**
                * @brief Get the original mode family weight
                *
                * @return The original mode family weight
                */
                double getModeFamilyWeight();
                
                /**
                * @brief Get the weighted mode family weight
                *
                * @return The weighted mode family weight
                */
                double getModeFamilyWeightTerm();

                /****************************Euclidean Weight****************************/
                /**
                * @brief Calculate the Euclidean weight (distance) between the transition's two nodes
                *
                * @return The Euclidean weight (distance) between the transition's two nodes
                */
                double calculateEuclideanWeight();

                /**
                * @brief Get the weighted term of the Euclidean distance between the transition's two nodes
                *
                * @return The weighted term of the Euclidean distance between the transition's two nodes
                */
                double getEuclideanWeightTerm();

                /****************************Stance Deviation Weight****************************/

                /**
                * @brief Calculate the stance deviation weight
                *
                * @return The stance deviation weight
                */
                double calculateStanceDeviationWeight();
                
                /**
                * @brief calculate consistency deviation weight
                *
                * @return The consistency deviation weight
                */
                double calculateConsistencyDeviationWeight();

                /**
                * @brief Get the original stance deviation weight
                *
                * @return The original stance deviation weight
                */
                double getStanceDeviationWeight();
                
                /**
                * @brief Get the weighted stance deviation weight
                *
                * @return The weighted stance deviation weight
                */
                double getStanceDeviationWeightTerm();

                /**
                * @brief Get the consistency deviation weight
                *
                * @return The consistency deviation weight
                */
                double getConsistencyDeviationWeight();

                /**
                * @brief Get the consistency deviation weight term
                *
                * @return The consistency deviation weight term
                */
                double getConsistencyDeviationWeightTerm();

                /****************************Torso Path Deviation Weight****************************/

                // /**
                // * @brief Calculate the torso path deviation
                // *
                // * @param torsoPath The torso path for the transition
                // * @return The torso path deviation
                // */
                // double calculateTorsoPathDeviation(const std::vector<TorsoStateNode::TorsoTransition *> & torsoPath);

                /**
                * @brief Calculate the nominal torso position for the transition footholds in the world frame
                *
                * @return The nominal torso position for the transition footholds in the world frame
                */
                vector3_t calculateNominalTorsoPositionWorldFrame() const;

                // /**
                // * @brief Calculate the nominal torso position for the transition footholds in the base frame
                // *
                // * @return The nominal torso position for the transition footholds in the base frame
                // */
                // vector3_t calculateNominalTorsoPositionBaseFrame() const;                

                /**
                * @brief Get the original torso path deviation weight
                *
                * @return The original torso path deviation weight
                */
                double getTorsoPathDeviationWeight();
                
                /**
                * @brief Get the weighted torso path deviation weight
                *
                * @return The weighted torso path deviation weight
                */
                double getTorsoPathDeviationWeightTerm();

                /**
                *@brief update the scalar transition weight with the cost from traj opt
                *
                *@param trajopt_cost The cost from traj opt
                */
                void updateWeightWithTrajOptCost(const double & trajopt_cost);

                /**
                * @brief Get the node from which the transition is made
                *
                * @return The node from which the transition is made
                */
                ModeFamilyNode *getNodeFrom() { return nodeFrom; }

                /**
                * @brief Get the node to which the transition is made
                *
                * @return The node to which the transition is made
                */
                ModeFamilyNode *getNodeTo() { return nodeTo; }

                /**
                * @brief Setter for transition traversal count
                *
                * @param newTraversalCount The new traversal count
                */
                void setTraversalCount(const short & newTraversalCount) { traversalCount = newTraversalCount; }

                /**
                * @brief Increment the transition traversal count
                */
                void incrementTraversalCount() { traversalCount++; }
                
                /**
                * @brief Getter for transition traversal count
                *
                * @return The transition traversal count
                */
                short getTraversalCount() { return traversalCount; }

                /**
                * @brief pretty-print the transition's from node and to node information
                *
                * @param includeTransitions Whether to include the other transitions of the nodes
                * @return The pretty-printed transition information
                */
                std::string description(const bool & includeTransitions=false);

                /**
                * @brief Getter for the number of dimensions of the transition
                *
                * @return Number of dimensions of the transition
                */
                short getNumDims() { return nDims_; }

                /**
                * @brief Setter for best existing solution to the transition swing trajectory
                *
                * @param bestSolution The new best solution to the transition swing trajectory
                */
                void setBestSolution(const ocs2::PrimalSolution & bestSolution) { bestSolution_ = bestSolution; }

                /**
                * @brief Setter for best existing cost to the transition swing trajectory
                *
                * @param bestCost The new best cost to the transition swing trajectory
                */
                void setBestCost(const double & bestCost) { bestCost_ = bestCost; }

                /**
                * @brief Getter for best existing solution to the transition swing trajectory
                *
                * @return Best existing solution to the transition swing trajectory
                */
                ocs2::PrimalSolution getBestSolution() { return bestSolution_; }
                
                /**
                * @brief Getter for best existing cost to the transition swing trajectory
                *
                * @return Best existing cost to the transition swing trajectory
                */
                double getBestCost() { return bestCost_; }             

            private:
                double weight = 0.0; /** Weight used for scoring entire mode family transition */
                double torsoPathDeviation = 0.0; /** Torso path deviation, euclidean distance from closest pose */
                double stanceDeviation_ = 0.0; /** Ideal stance deviation, cumulative distance/angle error */
                double consistencyDeviation_ = 0.0; /** Consistent transition displacement deviation  */
                ModeFamilyNode *nodeFrom = NULL; /** The node from which the transition is made */
                ModeFamilyNode *nodeTo = NULL; /** The node to which the transition is made */
                short traversalCount = 0.0; /** Number of times this transition has been traversed in prior experience trials */
                short nDims_ = 0; /** Number of dimensions of the transition, 4 for rebar grid, 8 for stepping stones */
                double w_exp = 0.0; /** Weight for the experience term */
                double w_euc = 0.0; /** Weight for the euclidean term */
                double w_stance = 0.0; /** Weight for the stance deviation term */
                double w_step = 0.0; /** Weight for the steppability term */
                double w_torso = 0.0;  /** Weight for the torso path deviation term */
                double w_consistency = 25.0; /** Weight for the consistency deviation term */

                ocs2::PrimalSolution bestSolution_; /** Best existing solution to the transition swing trajectory */
                double bestCost_ = 1e30; /** Best existing cost to the transition swing trajectory */
        };    

        /**
        * @brief Construct a new ModeFamilyNode object
        *
        * @param mf The mode family for the node
        * @param search_algorithm The search algorithm to use (0 - Dijkstra's, 1 - A*)
        */
        ModeFamilyNode(ModeFamily &mf, const base_coordinate_t & refTorsoPose, const short & search_algorithm);

        /**
        * @brief Destroy the ModeFamilyNode object
        */
        ~ModeFamilyNode();

        /**
        * @brief Reset the node parameters for a new search
        */
        void resetNode();

        /**
        * @brief Mark the node as explored during search
        */
        void markAsExplored() { explored = true; }

        /**
        * @brief Mark the node as visited during search
        */
        void markAsVisited() { visited = true; }

        /**
        * @brief Check if the node has been explored during search
        *
        * @return True if the node has been explored during search
        */
        bool isExplored() { return explored; }
        
        /**
        * @brief Check if the node has been visited during search
        *
        * @return True if the node has been visited during search
        */
        bool isVisited() { return visited; }

        /**
        * @brief set the cost to come to the node
        * @param costToCome The new cost to come to the node
        */
        void setCostToCome(const double & costToCome) { costToCome_ = costToCome; }
        
        /**
        * @brief Get the cost to come to the node
        *
        * @return The cost to come to the node
        */
        double getCostToCome() { return costToCome_; }

        /**
        * @brief Get the cost to go from the node
        *
        * @return The cost to go from the node
        */
        double getCostToGo() { return costToGo; }

        /**
        * @brief Calculate the Euclidean distance between the given node and another node
        *
        * @param otherNode The other node to calculate the Euclidean distance to
        * @return The Euclidean distance between the given node and another node
        */
        double calculateEuclideanDistance(const ModeFamilyNode & otherNode);
        
        double calculateEuclideanDistance(const base_coordinate_t & goalPose);

        /**
        * @brief Set the heuristic distance to the goal node, used in A* search as heuristic
        *
        * @param w_euc The weight for the Euclidean distance
        * @param goalNode The goal node
        */
        void setHeuristicDistance(const double & w_euc, const base_coordinate_t & goalPose) { costToGo = w_euc * calculateEuclideanDistance(goalPose); }

        /**
        * @brief Get the search algorithm used on the node during the search
        *
        * @return The search algorithm used on the node during the search
        */
        short getSearchAlgorithm() { return search_algorithm; }

        /**
        * @brief Get the mode family of the node
        *
        * @return The mode family of the node
        */
        ModeFamily* getModeFamily()  { return modeFamily; }

        /**
        * @brief Pretty-print the mode family node's information
        *
        * @param includeTransitions Whether to include the transitions of the node
        * @return The pretty-printed mode family node information
        */
        std::string description(const bool & includeTransitions=false);

        /**
        * @brief Add a transition from this node to another node
        *
        * @param nt The node to transition to
        * @param torsoPath The guiding torso path (used in weighting edge)
        * @param w_exp The weight for the experience term
        * @param w_euc The weight for the Euclidean term
        * @param w_stance The weight for the stance deviation term
        * @param w_step The weight for the steppability term
        * @param w_torso The weight for the torso path deviation term
        */
        void addTransitionTo(ModeFamilyNode * nt, 
                                const std::vector<TorsoStateNode::TorsoTransition *> & torsoPath,
                                const double & w_exp, 
                                const double & w_euc, 
                                const double & w_stance, 
                                const double & w_step, 
                                const double & w_torso);

        void addTransitionToGoal(ModeFamilyNode * nt);

        /**
        * @brief Get the number of dimensions of the whole mode family node
        *
        * @return The number of dimensions of the whole mode family node
        */
        short getNumDims() { return modeFamily->getNumDims(); }
        
        /**
        * @brief Get the number of dimensions of the mode family node for the given leg index
        *
        * @param legIdx The index of the leg
        * @return The number of dimensions of the mode family node for the given leg index
        */
        short getNumDims(const int & legIdx) { return modeFamily->getNumDims(legIdx); }

        /**
        * @brief == operator overlad. Used in std::find_if evaluation criterion 
        * within heap in order to determine if two nodes are equal. For our case, 
        * we define nodes as equal if their mode families are equal. 

        * @param n1 The first node to compare
        * @param n2 The second node to compare
        */
        friend bool operator==(const ModeFamilyNode &n1, const ModeFamilyNode &n2)
        {
            // std::cout << "ModeFamilyNode operator==" << std::endl;
            // std::cout<< "Comparing nodes: \n" << st1 << '\n' << st2 << std::endl;
            bool res = (*n1.modeFamily) == (*n2.modeFamily);
            // std::cout << "ModeFamilyNode operator== - " << res << std::endl;
            return res;
        };

        base_coordinate_t getRefTorsoPose() const { return refTorsoPose; }

        std::vector<Transition *> trans; /** Vector of transitions from this node to other nodes */

        ModeFamily *modeFamily = NULL; /** The mode family of the node */
        ModeFamilyNode *previousNode = NULL; /** The previous node in the search */
        Transition *previousTransition = NULL; /** The previous transition in the search */

    private:
        bool visited = false; /** Whether the mode family node has been visited during search */
        bool explored = false; /** Whether the mode family node has been explored during search */
        double costToCome_ = 1e30; /** The cost to come to the mode family node from the start node */
        double costToGo = 0.0; /** The cost to go from the mode family node to the goal node */
        short search_algorithm = 0; /** The search algorithm used on the node during the search */
        base_coordinate_t refTorsoPose; /** The reference torso pose for the node */
};

}  // namespace mmp
