#include <egocylindrical/ecwrapper_buffer.h>
// #include <egocylindrical/PropagatorConfig.h>
#include <egocylindrical/ecwrapper.h>

#include <stdexcept>
#include <queue>

namespace egocylindrical
{

    // utils::ECParams getParams(const egocylindrical::PropagatorConfig &config)
    // {
    //   utils::ECParams params;
    //   params.height = config.height;
    //   params.width = config.width;
    //   params.vfov = config.vfov;
    //   params.can_width = config.can_width;
    //   params.v_offset = config.v_offset;
    //   params.cyl_radius = config.cyl_radius;
    //   return params;
    // }

    utils::ECParams getParams() // const egocylindrical::PropagatorConfig &config
    {
      utils::ECParams params;
      params.height = 120; // 80; // 120
      params.width = 256; // 128; // 256
      params.vfov = M_PI / 3.0; // M_PI / 3.0;
      params.can_width = 256; // 512
      params.v_offset = 0.0;
      params.cyl_radius = 1.0;
      return params;
    }

    namespace utils
    {
        // const egocylindrical::PropagatorConfig &config, 
        utils::ECWrapperPtr getECWrapper(bool allocate_arrays=false)
        {
            return utils::getECWrapper(getParams(), allocate_arrays); // config
        }
    }

    ECWrapperBuffer::ECWrapperBuffer(): // egocylindrical::PropagatorConfig& config
        // config_(config),
        old_pts_(nullptr)
    {}

    ECWrapperBuffer::~ECWrapperBuffer()
    {

    }

    bool ECWrapperBuffer::init()
    {
        processing_thread_ = std::make_unique<Thread>(&egocylindrical::ECWrapperBuffer::bufferProcessingThread, this);

        // ROS_DEBUG_STREAM_NAMED("ecwrapper_buffer.init", "Initialize ECWrapperBuffer");
        addNew();
        return true;
    }

    utils::ECWrapper::Ptr ECWrapperBuffer::getOld()
    {
        {
            Lock lk(reset_mutex_);
            if(reset_requested_)
            {
                old_pts_ = nullptr;
                reset_requested_ = false;
                reset_cv_.notify_all();
            }
        }

        return old_pts_;
    }

    void ECWrapperBuffer::reset(float block)
    {
        {
            Lock lk(reset_mutex_);
            reset_requested_ = true;
        }

        if(block < 0)   //Wait indefinitely
        {
            Lock lk(reset_mutex_);
            reset_cv_.wait(lk, [this]{return reset_requested_;});
        }
        else if(block > 0)
        {
            Lock lk(reset_mutex_);
            std::chrono::duration<float> fblock;
            reset_cv_.wait_for(lk, std::chrono::duration_cast<std::chrono::milliseconds>(fblock), [this]{return reset_requested_;});
        }
    }

    void ECWrapperBuffer::addNew()
    {
        addToBuffer(nullptr);
    }

    void ECWrapperBuffer::addToBuffer(utils::ECWrapper::Ptr v)
    {
        next_pts_buffer_.push(v);
        next_pts_cv_.notify_one();
    }

    utils::ECWrapper::Ptr ECWrapperBuffer::getNew()
    {
        while (new_pts_buffer_.empty())
        {
            // ROS_DEBUG_STREAM_NAMED("ecwrapper_buffer.getNew", "Waiting for wrapper to become available...");
            addNew();
            // ros::WallDuration(0.01).sleep();
            rclcpp::Duration duration = rclcpp::Duration::from_seconds(0.01);
            rclcpp::sleep_for((std::chrono::nanoseconds(duration.nanoseconds()))); // sleep();

        }
        if (!new_pts_buffer_.empty())
        {
            // ROS_DEBUG_STREAM_NAMED("ecwrapper_buffer.getNew", "Retrieving clean, preallocated ECWrapper");
            new_pts_ = new_pts_buffer_.front();
            new_pts_buffer_.pop();
            return new_pts_;
        }
        else
        {
            // // ROS_ERROR(_STREAM("There must be a valid ECWrapper in [new_pts_buffer_]");

            throw std::out_of_range("There must be a valid ECWrapper in [new_pts_buffer_]");
        }
    }

    utils::ECWrapper::Ptr ECWrapperBuffer::reuseOld()
    {
        auto old = old_pts_;
        if(old->isLocked())
        {
            new_pts_ = copyECWrapper(*old);
            // old_pts_buffer_.pop();
            //releaseOld();
            // ROS_DEBUG_STREAM_NAMED("ecwrapper_buffer.reuseOld", "Unable to reuse old locked ECWrapper, copying it");
        }
        else
        {
            new_pts_ = old_pts_;
            old_pts_ = nullptr;
            // ROS_DEBUG_STREAM_NAMED("ecwrapper_buffer.reuseOld", "Reusing old unlocked ECWrapper");
        }
        return new_pts_;
    }

    void ECWrapperBuffer::update()
    {
        makeNewOld();

        // ROS_DEBUG_STREAM_NAMED("ecwrapper_buffer.update", "new_pts_buffer: " << new_pts_buffer_.size() << "; next_pts_buffer: " << next_pts_buffer_.size());
    }

    void ECWrapperBuffer::releaseOld()
    {
        auto v = old_pts_;
        {
            if(!v)
            {
                // ROS_DEBUG_STREAM_NAMED("ecwrapper_buffer.releaseOld", "Releasing nullptr ECWrapper");
            }
            else if(!v->isLocked())
            {
                // ROS_DEBUG_STREAM_NAMED("ecwrapper_buffer.releaseOld", "Releasing old unlocked ECWrapper");
            }
            else
            {
                // ROS_DEBUG_STREAM_NAMED("ecwrapper_buffer.releaseOld", "Releasing old locked ECWrapper");
            }

            addToBuffer(v);
        }
    }

    void ECWrapperBuffer::makeNewOld()
    {
        old_pts_ = new_pts_;
        new_pts_ = nullptr;
        // ROS_DEBUG_STREAM_NAMED("ecwrapper_buffer.makeNewOld", "'old'='new', 'new'=nullptr");
    }

    // TODO: Move config reconfigure and mutex into separate class used by this and main class
    utils::ECWrapper::Ptr ECWrapperBuffer::createNew()
    {
        // ROS_DEBUG_STREAM_NAMED("ecwrapper_buffer.createNew", "Create new ECWrapper with current config");
        return utils::getECWrapper(); // config_
    }

    void ECWrapperBuffer::bufferProcessingThread()
    {
        while(true || rclcpp::ok())
        {
            // Wait until object added to buffer
            Lock lk(next_pts_mutex_);
            std::cv_status res = next_pts_cv_.wait_for(lk, std::chrono::seconds(1));
            if (res == std::cv_status::timeout)
            {
                // addNew();
            }

        
            while(!next_pts_buffer_.empty())
            {
                // ros::WallTimet1 = ros::WallTime::now();
                auto v = next_pts_buffer_.front();
                next_pts_buffer_.pop();

                if(v)
                {
                    if(!v->isLocked())
                    {
                        v->init(getParams(), true); // config_
                        // ROS_DEBUG_STREAM_NAMED("ecwrapper_buffer.prepareNext", "Reuse old ECWrapper for next time");
                    }
                    else
                    {
                        v = createNew();
                        // ROS_DEBUG_STREAM_NAMED("ecwrapper_buffer.prepareNext", "Cannot reuse old locked ECWrapper, create new one");
                    }
                }
                else
                {
                    v = createNew();
                    // ROS_DEBUG_STREAM_NAMED("ecwrapper_buffer.makprepareNexteNewOld", "No old ECWrapper to reuse, create new one");
                }

                // ros::WallTimet2 = ros::WallTime::now();
                // ROS_DEBUG_STREAM_NAMED("timing", "Allocating wrapper took " << (t2-t1).toSec()*1000 << "ms");

                new_pts_buffer_.push(v);
            }
        }

    }

}   //end namespace egocylindrical
