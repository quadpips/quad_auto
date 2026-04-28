#pragma once
#include <mmp_quadruped/planning/Graph.h>

// #include <mmp_quadruped/MmpInterface.h>
#include <ocs2_custom_quadruped_interface/CustomQuadrupedInterface.h>

// #include <convex_plane_decomposition_msgs/PlanarTerrain.h>

#include <mmp_quadruped/MmpDimensions.h>

#include <random>

#include <Eigen/Core>
#include <Eigen/Geometry>


namespace mmp {

class ConstraintManager
{
    public:
        /**
        * @brief Construct a new Constraint Manager object
        *
        * @param interface legged robot interface object used to access the robot model and data
        */
        ConstraintManager(const rclcpp::Node::SharedPtr& node,
                            std::shared_ptr<switched_model::CustomQuadrupedInterface> & interface);
        
        /**
        * @brief Set the Graph object
        *
        * @param graph graph object
        */
        void setGraph(Graph * graph) { graph_ = graph; }

        /**
        * @brief Reset the configurations to the start state and input
        */
        void resetConfigurations();

        /**
        * @brief Find a transition configuration
        *
        * @return true if a valid transition configuration is found
        */
        bool findTransitionConfiguration();

        /**
        * @brief perform IK projection on configuration to satisfy contact constraints
        *
        * @param new_q new configuration after IK projection
        * @param contactCenter center of contact points
        * @param localContactNormal normal vector of contact plane
        * @return true if IK projection is successful
        */
        bool IKProjection(Eigen::VectorXd & new_q, 
                            const vector3_t & contactCenter,
                            const vector3_t & localContactNormal);

        /**
        * @brief Check if the goal mode family constraint is met
        *
        * @param latest_q latest configuration
        * @return true if the goal mode family constraint is met
        */
        bool checkGoalModeFamilyConstraint(const comkino_state_t & latest_x);

        bool checkGlobalGoalConstraint(const comkino_state_t & latest_x,
                                        const base_coordinate_t & torsoGlobalGoalPosition);

        /**
        * @brief Check if the destination mode family constraint is met
        *
        * @return true if the destination mode family constraint is met
        */
        bool checkDestinationModeFamilyConstraint();
        
        /**
        * @brief build the start mode family node ID from the latest configuration and steppable regions
        *
        * @param latest_q latest configuration
        * @param latest_u latest input
        * @param allRegions all steppable regions
        * @return start mode family node ID
        */
        ModeFamilyNodeID getStartModeFamilyNodeID(const comkino_state_t & latest_q,
                                                    const comkino_input_t & latest_u,
                                                    const std::vector<switched_model::ConvexTerrain> & allRegions);

        // /**
        // * @brief build the goal mode family node ID from the latest configuration and steppable regions
        // *
        // * @param torsoGoalPosition goal position of the torso
        // * @param allRegions all steppable regions
        // * @return goal mode family node ID
        // */                
        // ModeFamilyNodeID getGoalModeFamilyNodeID(const vector3_t & torsoGoalPosition,
        //                                             const std::vector<switched_model::ConvexTerrain> & allRegions);


        /**
        * @brief Pretty print for contact positions
        *
        * @return string of contact positions
        */
        std::string descriptionPositions();

        /**
        * @brief update positions of legs to current node's contact constraints
        */
        void updateLegsPositionConstraints();

        /**
        * @brief set current state
        *
        * @param currentState current state
        */
        void setCurrentState(const comkino_state_t & x) { current_x = x; }
        
        /**
        * @brief get current state
        *
        * @return current state
        */
        comkino_state_t getCurrentState() { return current_x; }

        /**
        * @brief set current input
        *
        * @param currentInput current input
        */
        void setCurrentInput(const comkino_input_t & u) { current_u = u; }
        
        /**
        * @brief get current input
        *
        * @return current input
        */
        comkino_input_t getCurrentInput() { return current_u; }

        /**
        * @brief set target state
        *
        * @param targetState target state
        */
        void setTargetState(const comkino_state_t & x) { target_x = x; }        
        
        /**
        * @brief get target state
        *
        * @return target state
        */
        comkino_state_t getTargetState() { return target_x; }

    private:

        void enforceJointLimits(Eigen::VectorXd & q,
                                const vector3_t & contactCenter,
                                const vector3_t & localContactNormal);

        int attachRegionToLeg(const std::vector<switched_model::ConvexTerrain> & allRegions,
                                const int & legIdx,
                                const comkino_state_t & x);

        /**
        * @brief find a configuration that satisfies the set constraints
        *
        * @param q configuration to evaluate
        * @return true if a valid configuration is found
        */
        bool findConfiguration(comkino_state_t & x);

        /**
        * @brief Check if the mode family constraint is met
        *
        * @param latest_q configuration to evaluate
        * @param destModeFamily destination mode family
        * @return true if the mode family constraint is met
        */
        bool checkModeFamilyConstraint(const comkino_state_t & latest_x,
                                        ModeFamily * destModeFamily);

        /**
        * @brief update positions to the start mode family contact constraints
        */
        void updateLegsPositionConstraintsToStart();
        
        /**
        * @brief Query leg position constraint from mode family
        *
        * @param mf mode family
        * @param legIdx leg index
        * @return 3D position of the leg
        */
        vector3_t getLegPositionConstraint(ModeFamily * mf, const int & legIdx);

        rclcpp::Node::SharedPtr node_; /**< ROS2 node pointer */

        std::shared_ptr<switched_model::CustomQuadrupedInterface> interface_; /**< legged robot interface object */
        Graph * graph_; /**< graph object */

        std::vector<vector3_t> positions; /**< 3D positions of the leg constraints */

        comkino_state_t start_x = comkino_state_t::Zero(); /**< start state */
        // Eigen::VectorXd goal_x; /**< goal state */
        comkino_state_t current_x  = comkino_state_t::Zero(); /**< current state */
        comkino_state_t target_x  = comkino_state_t::Zero(); /**< target state */

        comkino_input_t start_u  = comkino_input_t::Zero(); /**< start input */
        // comkino_input_t goal_u; /**< goal input */
        comkino_input_t current_u  = comkino_input_t::Zero(); /**< current input */
        comkino_input_t target_u  = comkino_input_t::Zero(); /**< target input */

        Eigen::MatrixXd J_FL = Eigen::MatrixXd::Zero(6, 12); /**< Jacobian for front left leg */
        Eigen::MatrixXd J_FR = Eigen::MatrixXd::Zero(6, 12); /**< Jacobian for front right leg */
        Eigen::MatrixXd J_BL = Eigen::MatrixXd::Zero(6, 12); /**< Jacobian for back left leg */
        Eigen::MatrixXd J_BR = Eigen::MatrixXd::Zero(6, 12); /**< Jacobian for back right leg */
        Eigen::MatrixXd J_total = Eigen::MatrixXd::Zero(12, 12); /**< Combined Jacobian for all legs */
};

}  // namespace mmp