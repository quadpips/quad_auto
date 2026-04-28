#pragma once

#include <mmp_quadruped/planning/utils.h>


namespace mmp {

struct TorsoStateNode
{
    public:
        struct TorsoTransition
        {
            public:
                /**
                * @brief Construct a new Torso Transition object
                * 
                * @param nodeFrom The node from which the transition originates
                * @param nodeTo The node to which the transition goes
                // * @param weight The weight of the transition
                */
                TorsoTransition(TorsoStateNode * nodeFrom, 
                                TorsoStateNode * nodeTo);

                /**
                * @brief Getter for nodeFrom_
                *
                * @return TorsoStateNode* The node from which the transition originates
                */                
                TorsoStateNode * getNodeFrom() { return nodeFrom_; }
                
                /**
                * @brief Getter for nodeTo_
                *
                * @return TorsoStateNode* The node to which the transition goes
                */
                TorsoStateNode * getNodeTo() { return nodeTo_; }

                // /**
                // * @brief Transform the torso transition positions to the base frame
                // *
                // * @param worldFrameToBaseFrameTransform The transform from the world frame to the base frame
                // */
                // void transformToBaseFrame(const geometry_msgs::msg::TransformStamped & worldFrameToBaseFrameTransform);

                /**
                * @brief Getter for torso transition weight
                *
                * @return double The weight of the torso transition
                */
                double getWeight() { return weight_; }

            private:
                TorsoStateNode * nodeFrom_ = NULL; /** The node from which the transition originates */
                TorsoStateNode * nodeTo_ = NULL; /** The node to which the transition goes */
                double weight_; /** The weight of the transition */
        };

        /**
        * @brief Construct a new Torso State Node object
        *
        * @param torsoPosition The torso position in the world frame
        * @param theta The torso yaw
        * @param goalPosition The goal position in the world frame
        * @param goal Whether the node is a goal node
        */
        TorsoStateNode(const vector3_t & torsoPosition, const vector3_t & torsoOrientation, 
                        const vector3_t & goalPosition, const vector3_t & goalOrientation); // , const bool & goal

        /**
        * @brief Destroy the Torso State Node object
        */
        ~TorsoStateNode();

        double calculatePositionalDistance(TorsoStateNode * other);
        
        double calculateOrientationDistance(TorsoStateNode * other);

        double calculateDistance(TorsoStateNode * other);
     
        /**
        * @brief Add a transition from this torso state node to another
        *
        * @param dst_node The destination torso state node
        */
        void addTransition(TorsoStateNode * dst_node);

        /**
        * @brief Getter for the torso position in the world frame
        *
        * @return vector3_t The torso position in the world frame
        */
        base_coordinate_t getPoseWorldFrame() { return poseWorldFrame; } 

        // /**
        // * @brief Getter for the torso position in the base frame
        // *
        // * @return vector3_t The torso position in the base frame
        // */
        // vector3_t getPositionBaseFrame() { return positionBaseFrame; } 

        // /**
        // * @brief Transform the torso position from the world frame to the base frame
        // *
        // * @param worldFrameToBaseFrameTransform The transform from the world frame to the base frame
        // */
        // void transformToBaseFrame(const geometry_msgs::msg::TransformStamped & worldFrameToBaseFrameTransform);

        // /**
        // * @brief Getter for the torso yaw
        // *
        // * @return double : The torso yaw
        // */
        // double Theta() { return theta; }

        /**
        * @brief Getter for torso state node goal flag
        *
        * @return bool : Whether the torso state node is the goal node
        */
        bool isGoal() { return goal; }

        /**
        * @brief Pretty-print the torso state node description
        *
        * @return std::string : The torso state node description
        */
        std::string description();

        /**
        * @brief Setter for the cost to come
        *
        * @param costToCome The cost to come
        */
        void setCostToCome(double costToCome) { costToCome_ = costToCome; }
        
        /** 
        * @brief Getter for the cost to come
        *
        * @return double : The cost to come
        */
        double getCostToCome() { return costToCome_; }
        
        /**
        * @brief Getter for the cost to go
        *
        * @return double : The cost to go
        */
        double getCostToGo() { return costToGo_; }

        /** 
        * @brief Mark the torso node as visited during search
        */
        void markAsVisited() { visited = true; }
        
        /**
        * @brief Mark the torso node as explored during search
        */
        void markAsExplored() { explored = true; }

        /**
        * @brief Getter for the visited flag
        *
        * @return bool : Whether the torso node has been visited
        */
        bool isVisited() { return visited; }
        
        /**
        * @brief Getter for the explored flag
        *
        * @return bool : Whether the torso node has been explored
        */
        bool isExplored() { return explored; }

        void setGoal(const bool & goal) { this->goal = goal; }

        std::vector<TorsoTransition *> trans; /**< Vector of transitions from this torso state node */

        TorsoStateNode * previousNode = NULL; /**< The previous torso state node, used for tracing back after search */
        TorsoTransition * previousTransition = NULL; /**< The previous torso transition, used for tracing back after search */

    private:
        /**
        * @brief Calculate the cost to go (Euclidean distance) from the torso state node to the goal position
        *
        * @param goalPosition The goal position in the world frame
        * @return double : The cost to go
        */
        double calculateCostToGo(const vector3_t & goalPositionWorldFrame,
                                    const vector3_t & goalOrientationWorldFrame); 

        // vector3_t positionWorldFrame; /**< The torso position in the world frame */
        // vector3_t orientationWorldFrame; /**< The torso orientation in the world frame */
        base_coordinate_t poseWorldFrame; /**< The torso pose in the world frame */
        // vector3_t positionBaseFrame; /**< The torso position in the base frame */
        // double theta = 0.0; /**< The torso yaw */

        double costToCome_ = 0.0; /**< The cost to come from the start node to this node */
        double costToGo_ = 0.0; /**< The cost to go from this node to the goal node */

        bool goal = false; /**< Whether the torso state node is the goal node */

        bool visited = false; /**< Whether the torso state node has been visited during search */
        bool explored = false; /**< Whether the torso state node has been explored during search */
};

}  // namespace mmp
