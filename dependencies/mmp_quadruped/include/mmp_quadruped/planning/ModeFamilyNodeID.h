#pragma once

namespace mmp {

/****************************ModeFamilyNodeID****************************/
struct ModeFamilyNodeID
{
    public:
        int BL_id_ = 0; //**< Region ID of the back left leg */
        int BR_id_ = 0; //**< Region ID of the back right leg */
        int FL_id_ = 0; //**< Region ID of the front left leg */
        int FR_id_ = 0; //**< Region ID of the front right leg */

        /**
        * @brief Construct a new ModeFamilyNodeID object,
        *        more or less used as a placeholder
        */
        ModeFamilyNodeID() :
                    BL_id_(-1), BR_id_(-1), FL_id_(-1), FR_id_(-1) {}

        /**
        * @brief Construct a new ModeFamilyNodeID object
        *
        * @param FL_id Region ID of the front left leg
        * @param FR_id Region ID of the front right leg
        * @param BL_id Region ID of the back left leg
        * @param BR_id Region ID of the back right leg
        */
        ModeFamilyNodeID(const int & FL_id, const int & FR_id, const int & BL_id, const int & BR_id) :
                    FL_id_(FL_id), FR_id_(FR_id) , BL_id_(BL_id), BR_id_(BR_id) {}

        /**
        * @brief Set the region IDs for all legs
        *
        * @param FL_id Region ID of the front left leg
        * @param FR_id Region ID of the front right leg
        * @param BL_id Region ID of the back left leg
        * @param BR_id Region ID of the back right leg
        */
        void setRegionID(const int & FL_id, const int & FR_id, const int & BL_id, const int & BR_id)
        {
            BL_id_ = BL_id;
            BR_id_ = BR_id;
            FL_id_ = FL_id;
            FR_id_ = FR_id;
        }

        /**
        * @brief Set the region ID for the given leg
        *
        * @param legId The index of the leg
        * @param regionID The region ID to set
        */
        void setRegionID(const int & legId, const int & regionID)
        {
            if (legId == FL)
                FL_id_ = regionID;
            else if (legId == FR)
                FR_id_ = regionID;
            else if (legId == BL)
                BL_id_ = regionID;
            else if (legId == BR)
                BR_id_ = regionID;
        }

        int getRegionID(const int & legId) const
        {
            if (legId == FL)
                return FL_id_;
            else if (legId == FR)
                return FR_id_;
            else if (legId == BL)
                return BL_id_;
            else if (legId == BR)
                return BR_id_;
            else
                throw std::runtime_error("Invalid leg index");
        }

        // 
        /**
        * @brief < operator overload, we use these IDs as keys in a node map,
        * and for std::map, the key equality (x == y) is checked by seeing 
        * if x < y is false and y < x is false
        *
        * @param rhs The right hand side ModeFamilyNodeID
        * @return true if this < rhs, false otherwise
        */
        bool operator<(const ModeFamilyNodeID & rhs) const
        { 
            bool FL_less = less(FL_id_, rhs.FL_id_);
            bool FL_equal = equal(FL_id_, rhs.FL_id_);
            bool FL_greater = greater(FL_id_, rhs.FL_id_);

            if (FL_less) //             std::cout << "FL_x_less --> true" << std::endl;
                return true;

            if (FL_greater) //             std::cout << "FL_x_less --> true" << std::endl;
                return false;            

            // Therefore, FL's are equal

            bool FR_less = less(FR_id_, rhs.FR_id_);
            bool FR_equal = equal(FR_id_, rhs.FR_id_);
            bool FR_greater = greater(FR_id_, rhs.FR_id_);

            if (FR_less) //             std::cout << "FR_x_less --> true" << std::endl;
                return true;

            if (FR_greater) //             std::cout << "FR_x_less --> true" << std::endl;
                return false;            

            // Therefore, FR's are equal

            bool BL_less = less(BL_id_, rhs.BL_id_);
            bool BL_equal = equal(BL_id_, rhs.BL_id_);
            bool BL_greater = greater(BL_id_, rhs.BL_id_);

            if (BL_less) //             std::cout << "BL_x_less --> true" << std::endl;
                return true;

            if (BL_greater) //             std::cout << "BL_x_less --> true" << std::endl;
                return false;            

            // Therefore, BL's are equal

            bool BR_less = less(BR_id_, rhs.BR_id_);
            bool BR_equal = equal(BR_id_, rhs.BR_id_);
            bool BR_greater = greater(BR_id_, rhs.BR_id_);

            if (BR_less) //             std::cout << "BR_x_less --> true" << std::endl;
                return true;

            if (BR_greater) //             std::cout << "BR_x_less --> true" << std::endl;
                return false;            

            // Therefore, BR's are equal
            return false;
        }

    private:

        /**
        * @brief Helper function for < operator, checks if val is less than rhs_val with epsilon robustness
        *
        * @param val The value to compare
        * @param rhs_val The value to compare against
        * @return true if val < rhs_val, false otherwise
        */
        bool less(double val, const double & rhs_val) const
        {
            return (rhs_val - val) > std::numeric_limits<double>::epsilon();
        }

        /**
        * @brief Helper function for < operator, checks if val is greater than rhs_val with epsilon robustness
        *
        * @param val The value to compare
        * @param rhs_val The value to compare against
        * @return true if val > rhs_val, false otherwise
        */
        bool greater(double val, const double & rhs_val) const
        {
            return (rhs_val - val) < -std::numeric_limits<double>::epsilon();
        }

        /**
        * @brief Helper function for < operator, checks if val is equal to rhs_val with epsilon robustness
        *
        * @param val The value to compare
        * @param rhs_val The value to compare against
        * @return true if val == rhs_val, false otherwise
        */
        bool equal(double val, const double & rhs_val) const
        {
            return std::fabs(rhs_val - val) < std::numeric_limits<double>::epsilon();
        }

};

}  // namespace mmp
