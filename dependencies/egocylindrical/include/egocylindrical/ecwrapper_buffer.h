#ifndef EGOCYLINDRICAL_ECWRAPPER_BUFFER_H
#define EGOCYLINDRICAL_ECWRAPPER_BUFFER_H

// #include <egocylindrical/PropagatorConfig.h>
#include <egocylindrical/ecwrapper.h>

#include <queue>
#include <atomic>
#include <chrono>
#include <mutex>
#include <condition_variable>
#include <thread>

#include <rclcpp/rclcpp.hpp>


namespace egocylindrical
{

    class ECWrapperBuffer
    {

    private:

        using Mutex = std::mutex;
        using Lock = std::unique_lock<Mutex>;
        using ConditionVar = std::condition_variable;
        
        Mutex next_pts_mutex_;
        ConditionVar next_pts_cv_;

        Mutex reset_mutex_;
        ConditionVar reset_cv_;
        bool reset_requested_;

        using Thread = std::thread;
        using ThreadPtr = std::unique_ptr<Thread>;
        ThreadPtr processing_thread_;


    public:
        ECWrapperBuffer(); // egocylindrical::PropagatorConfig& config

        ~ECWrapperBuffer();

        bool init();

        void reset(float block_time=0);

        utils::ECWrapper::Ptr getOld();

        void addNew();

        void addToBuffer(utils::ECWrapper::Ptr v);

        utils::ECWrapper::Ptr getNew();

        utils::ECWrapper::Ptr reuseOld();

        void update();

    public:

        void releaseOld();

    protected:

        void makeNewOld();

        utils::ECWrapper::Ptr createNew();

        void bufferProcessingThread();

    public:
        using Ptr = std::shared_ptr<ECWrapperBuffer>;


    protected:
        // egocylindrical::PropagatorConfig& config_;

        std::queue<utils::ECWrapper::Ptr> new_pts_buffer_, next_pts_buffer_;
        
        utils::ECWrapper::Ptr old_pts_, new_pts_;


    };

}   //end namespace egocylindrical

#endif //EGOCYLINDRICAL_ECWRAPPER_BUFFER_H
