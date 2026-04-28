/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#ifndef EGOCYLINDRICAL_TIME_SEQUENCER_H
#define EGOCYLINDRICAL_TIME_SEQUENCER_H

#include <rclcpp/rclcpp.hpp>

#include <message_filters/connection.h>
#include <message_filters/simple_filter.h>

namespace egocylindrical
{

/**
 * \class TimeSequencer
 *
 * \brief Sequences messages based on the timestamp of their header.
 *
 * The TimeSequencer object is templated on the type of message being sequenced.
 *
 * \section behavior BEHAVIOR
 * At construction, the TimeSequencer takes a ros::Duration
 * "delay" which specifies how long to queue up messages to
 * provide a time sequencing over them.  As messages arrive they are
 * sorted according to their time stamps.  A callback for a message is
 * never invoked until the messages' time stamp is out of date by at
 * least delay.  However, for all messages which are out of date
 * by at least delay, their callback are invoked and guaranteed
 * to be in temporal order.  If a message arrives from a time \b prior
 * to a message which has already had its callback invoked, it is
 * thrown away.
 *
 * \section connections CONNECTIONS
 *
 * TimeSequencer's input and output connections are both of the same signature as roscpp subscription callbacks, ie.
\verbatim
void callback(const std::shared_ptr<M const>&);
\endverbatim
 *
 */
template<class M>
class TimeSequencer  //: public message_filters::SimpleFilter<M>
{
public:
//   typedef std::shared_ptr<M> MPtr;

  using MPtr = typename M::Ptr;
  using Callback = boost::function<void(MPtr) > ;
      
//   typedef ros::MessageEvent<M const> EventType;
  
  

  /**
   * \brief Constructor
   * \param f A filter to connect this sequencer's input to
   * \param delay The minimum time to hold a message before passing it through.
   * \param update_rate The rate at which to check for messages which have passed "delay"
   * \param queue_size The number of messages to store
   * \param nh (optional) The NodeHandle to use to create the ros::SteadyTimer that runs at update_rate
   */
//   template<class F>
//   TimeSequencer(F& f, ros::Duration delay, ros::Duration update_rate, uint32_t queue_size, ros::NodeHandle nh = ros::NodeHandle())
//   : delay_(delay)
//   , update_rate_(update_rate)
//   , queue_size_(queue_size)
//   , nh_(nh)
//   {
//     init();
//     connectInput(f);
//   }

  /**
   * \brief Constructor
   *
   * This version of the constructor does not take a filter immediately.  You can connect to a filter later with the connectInput() function
   *
   * \param delay The minimum time to hold a message before passing it through.
   * \param update_rate The rate at which to check for messages which have passed "delay"
   * \param queue_size The number of messages to store
   * \param nh (optional) The NodeHandle to use to create the ros::SteadyTimer that runs at update_rate
   */
  TimeSequencer(ros::Duration delay, ros::Duration update_rate, uint32_t queue_size, ros::NodeHandle nh = ros::NodeHandle())
  : delay_(delay)
  , update_rate_(update_rate)
  , queue_size_(queue_size)
  , nh_(nh)
  {
    init();
  }

  /**
   * \brief Connect this filter's input to another filter's output.
   */
//   template<class F>
//   void connectInput(F& f)
//   {
//     incoming_connection_.disconnect();
//     incoming_connection_ = f.registerCallback(typename SimpleFilter<M>::EventCallback(std::bind(&TimeSequencer::cb, this, boost::placeholders::_1)));
//   }

  ~TimeSequencer()
  {
    update_timer_.stop();
//     incoming_connection_.disconnect();
  }
  
  void setCallback(Callback cb) {cb_ = cb;}

  void add(const MPtr& evt)
  {
    namespace mt = ros::message_traits;

    boost::mutex::scoped_lock lock(messages_mutex_);
    if (mt::TimeStamp<M>::value(*evt) < last_time_)
    {
      return;
    }

    messages_.insert(evt);

    if (queue_size_ != 0 && messages_.size() > queue_size_)
    {
      messages_.erase(*messages_.begin());
    }
  }

//   /**
//    * \brief Manually add a message to the cache.
//    */
//   void add(const MPtr& msg)
//   {
//     EventType evt(msg);
//     add(evt);
//   }

private:
  class MessageSort
  {
  public:
    bool operator()(const MPtr& lhs, const MPtr& rhs) const
    {
      namespace mt = ros::message_traits;
      return mt::TimeStamp<M>::value(*lhs) < mt::TimeStamp<M>::value(*rhs);
    }
  };
  typedef std::multiset<MPtr, MessageSort> S_Message;
  typedef std::vector<MPtr> V_Message;

//   void cb(const EventType& evt)
//   {
//     add(evt);
//   }

  void dispatch()
  {
    namespace mt = ros::message_traits;

    V_Message to_call;

    {
      boost::mutex::scoped_lock lock(messages_mutex_);

      while (!messages_.empty())
      {
        const MPtr& e = *messages_.begin();
        ros::Time stamp = mt::TimeStamp<M>::value(*e);
        if (stamp + delay_ <= ros::Time::now())
        {
          last_time_ = stamp;
          to_call.push_back(e);
          messages_.erase(messages_.begin());
        }
        else
        {
          break;
        }
      }
    }

    {
      typename V_Message::iterator it = to_call.begin();
      typename V_Message::iterator end = to_call.end();
      for (; it != end; ++it)
      {
        cb_(*it);
        //this->signalMessage(*it);
      }
    }
  }

  void update(const ros::SteadyTimerEvent&)
  {
    dispatch();
  }

  void init()
  {
    update_timer_ = nh_.createSteadyTimer(ros::WallDuration(update_rate_.toSec()), &TimeSequencer::update, this);
  }

  ros::Duration delay_;
  ros::Duration update_rate_;
  uint32_t queue_size_;
  ros::NodeHandle nh_;

  ros::SteadyTimer update_timer_;

  //std::vector<Connection> incoming_connections_;


  S_Message messages_;
  boost::mutex messages_mutex_;
  ros::Time last_time_;
  
      
  protected:
  Callback cb_=0;
  
};

}

#endif
