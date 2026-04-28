#include <mmp_quadruped/planning/ModeFamilyNode.h>


namespace mmp {

struct ExperienceStep
{
    public:
        /**
        * @brief Construct a new Experience Step object
        * 
        * @param transition mode family transition for which the experience step is created
        * @param lead full lead of transitions
        */
        ExperienceStep(ModeFamilyNode::Transition * transition, const std::vector<ModeFamilyNode::Transition *> & lead)
        {
            this->transition = transition;
            this->modeFamilyFrom = transition->getNodeFrom()->modeFamily;
            this->modeFamilyTo = transition->getNodeTo()->modeFamily;
            this->leadSize = lead.size();
            this->result = "";
            this->trajopt_cost = 0.0;
            this->singleModePlanningTime = 0.0;
            this->numIterations = 0;
        }

        /**
        * @brief Setter for traj opt cost
        *
        * @param trajopt_cost cost of the trajectory optimization
        */
        void setTrajOptCost(const double & trajopt_cost) { this->trajopt_cost = trajopt_cost; }
        
        /**
        * @brief Getter for traj opt cost
        *
        * @return double cost of the trajectory optimization
        */
        double getTrajOptCost() { return this->trajopt_cost; }

        /**
        * @brief Getter for mode family transition
        *
        * @return ModeFamilyNode::Transition* mode family transition
        */
        ModeFamilyNode::Transition * getTransition() { return this->transition; }

        /**
        * @brief Getter for mode family from
        *
        * @return ModeFamily* mode family from
        */
        ModeFamily* getModeFamilyFrom() { return this->modeFamilyFrom; }
        
        /**
        * @brief Getter for mode family to
        *
        * @return ModeFamily* mode family to
        */
        ModeFamily* getModeFamilyTo() { return this->modeFamilyTo; }

        /**
        * @brief Getter for lead size
        *
        * @return int size of the lead
        */
        int getLeadSize() { return this->leadSize; }

        /**
        * @brief Setter for transition result
        *
        * @param result result of the transition
        */
        void setResult(const std::string & result) { this->result = result; }
        
        /**
        * @brief Getter for transition result
        *
        * @return std::string result of the transition
        */
        std::string getResult() { return this->result; }

        /**
        * @brief Setter for single mode traj opt planning time
        *
        * @param singleModePlanningTime time taken for single mode planning
        */
        void setSingleModePlanningTime(const double & singleModePlanningTime) { this->singleModePlanningTime = singleModePlanningTime; }
        
        /**
        * @brief Getter for single mode traj opt planning time
        *
        * @return double time taken for single mode planning
        */
        double getSingleModePlanningTime() { return this->singleModePlanningTime; }

        /**
        * @brief Setter for number of SQP iterations
        *
        * @param numIterations number of SQP iterations
        */
        void setNumIterations(const size_t & numIterations) { this->numIterations = numIterations; }
        
        /**
        * @brief Getter for number of SQP iterations
        *
        * @return size_t number of SQP iterations
        */
        size_t getNumIterations() { return this->numIterations; }

        /**
        * @brief Setter for primal solution
        *
        * @param solution primal solution
        */
        void setSolution(const ocs2::PrimalSolution & solution) { this->solution_ = solution; }
        
        /**
        * @brief Getter for primal solution
        *
        * @return PrimalSolution primal solution
        */
        ocs2::PrimalSolution getSolution() { return this->solution_; }

    private:
        ModeFamilyNode::Transition * transition = NULL; /**< mode family transition */
        ModeFamily* modeFamilyFrom = NULL; /**< from mode family */
        ModeFamily* modeFamilyTo = NULL; /**< to mode family */
        int leadSize = 0; /**< size of the lead */
        std::string result = ""; /**< result of the transition */
        double trajopt_cost = 0.0; /**< cost of the trajectory optimization */
        double singleModePlanningTime = 0.0; /**< time taken for single mode traj opt planning */
        size_t numIterations = 0; /**< number of SQP iterations */
        ocs2::PrimalSolution solution_; /**< primal solution */
};

}  // namespace mmp
