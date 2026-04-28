#include <egocylindrical/range_image_inflator_generator.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

//Redundant
#include <rclcpp/rclcpp.hpp>


namespace egocylindrical
{

    /**
    * @brief Nodelet-wrapper of the RangeImageInflatorGenerator class
    */
    class RangeImageInflatorNodelet : public nodelet::Nodelet
    {
    public:
      RangeImageInflatorNodelet(){};
      ~RangeImageInflatorNodelet(){}
      
      /**
      * @brief Initialise the nodelet
      *
      * This function is called, when the nodelet manager loads the nodelet.
      */
      virtual void onInit()
      {
        ros::NodeHandle nh = this->getMTNodeHandle();
        ros::NodeHandle pnh = this->getMTPrivateNodeHandle();
        
        // resolve node(let) name
        std::string name = pnh.getUnresolvedNamespace();
        int pos = name.find_last_of('/');
        name = name.substr(pos + 1);
        
        NODELET_INFO_STREAM("Initialising nodelet... [" << name << "]");
        inflator_ = std::make_shared<RangeImageInflatorGenerator>(nh, pnh);
        
        // Initialises the controller
        if (inflator_->init())
        {
          NODELET_INFO_STREAM("Nodelet initialised. [" << name << "]");
        }
        else
        {
          NODELET_ERROR_STREAM("Couldn't initialise nodelet! Please restart. [" << name << "]");
        }
      }
    private:
      std::shared_ptr<RangeImageInflatorGenerator> inflator_;
    };
    

PLUGINLIB_EXPORT_CLASS(egocylindrical::RangeImageInflatorNodelet,
                       nodelet::Nodelet);
// %EndTag(FULLTEXT)%


}

