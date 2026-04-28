#include <egocylindrical/egocylindrical.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>




namespace egocylindrical
{
  
  /**
   * @brief Nodelet-wrapper of the PipsTrajectoryController class
   */
  class EgocylindricalPropagatorNodelet : public nodelet::Nodelet
  {
  public:
    EgocylindricalPropagatorNodelet(){};
    ~EgocylindricalPropagatorNodelet(){}
    
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
      propagator_ = std::make_shared<EgoCylindricalPropagator>(nh, pnh);
      
      // Initialises the controller
      if (propagator_->init())
      {
        NODELET_INFO_STREAM("Nodelet initialised. [" << name << "]");
      }
      else
      {
        NODELET_ERROR_STREAM("Couldn't initialise nodelet! Please restart. [" << name << "]");
      }
    }
  private:
    std::shared_ptr<EgoCylindricalPropagator> propagator_;
  };
  
} // namespace egocylindrical

PLUGINLIB_EXPORT_CLASS(egocylindrical::EgocylindricalPropagatorNodelet,
                       nodelet::Nodelet);
// %EndTag(FULLTEXT)%

