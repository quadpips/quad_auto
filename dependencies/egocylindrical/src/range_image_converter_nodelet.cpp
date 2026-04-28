#include <egocylindrical/range_image_converter.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

//Redundant
#include <rclcpp/rclcpp.hpp>


namespace egocylindrical
{

    /**
    * @brief Nodelet-wrapper of the RangeImageConverter class
    */
    class RangeImageConverterNodelet : public nodelet::Nodelet
    {
    public:
      RangeImageConverterNodelet(){};
      ~RangeImageConverterNodelet(){}
      
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
        converter_ = std::make_shared<RangeImageConverter>(nh, pnh);
        
        // Initialises the controller
        if (converter_->init())
        {
          NODELET_INFO_STREAM("Nodelet initialised. [" << name << "]");
        }
        else
        {
          NODELET_ERROR_STREAM("Couldn't initialise nodelet! Please restart. [" << name << "]");
        }
      }
    private:
      std::shared_ptr<RangeImageConverter> converter_;
    };
    

PLUGINLIB_EXPORT_CLASS(egocylindrical::RangeImageConverterNodelet,
                       nodelet::Nodelet);
// %EndTag(FULLTEXT)%


}

