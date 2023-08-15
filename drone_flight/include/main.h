#ifndef DRONE_FLIGHT_MAIN_H
#define DRONE_FLIGHT_MAIN_H

///// common headers
#include <time.h>
#include <math.h>
#include <cmath>
#include <chrono>
#include <vector>

///// ROS
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
///// MAVROS
#include <mavros_msgs/State.h> //state
#include <mavros_msgs/SetMode.h> //offboarding
#include <mavros_msgs/CommandBool.h> //arming
#include <mavros_msgs/CommandLong.h> //disarming (kill service)


using namespace std;
using namespace std::chrono;


//////////////////////////////////////////////////////////////////////////////
inline double euclidean_dist(const double &x, const double &y, const double &z, const double &x2, const double &y2, const double &z2)
{
  return sqrt( (x-x2)*(x-x2) + (y-y2)*(y-y2) + (z-z2)*(z-z2) );
}
//////////////////////////////////////////////////////////////////////////////
class drone_flight_class
{
  private:
    ///// basic params
    bool m_ctrl_init = false;
    bool m_pose_check = false;

    ///// drone state
    mavros_msgs::State m_current_state;
    double m_current_pos_x = 0.0, m_current_pos_y = 0.0, m_current_pos_z = 0.0;
    double m_fixed_x_pos_temporal, m_fixed_y_pos_temporal;

    ///// control pararm
    double m_taking_off_altitude = 2.0;
    vector<double> m_waypoints;
    bool m_loop_enable = true;
    double m_dist_tol = 0.3;
    double m_control_loop_hz = 5.0;
    int m_waypoint_index = 0;

    ///// ROS
    ros::NodeHandle m_nh;
    ros::Subscriber m_pose_sub, m_state_sub;
    ros::Publisher m_position_controller_pub;
    ros::ServiceClient m_arming_client, m_set_mode_client, m_kill_client;
    ros::Timer m_control_timer;
    
    ///// functions
    void state_cb(const mavros_msgs::State::ConstPtr& msg);
    void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void control_timer_func(const ros::TimerEvent& event);

  public:
    drone_flight_class(const ros::NodeHandle& n_private);
};


//////////////////////////////////////////////////////////////////////////////
drone_flight_class::drone_flight_class(const ros::NodeHandle& n_private) : m_nh(n_private)
{
  ///// params
  m_nh.param<double>("/taking_off_altitude", m_taking_off_altitude, 2.0);
  m_nh.param<double>("/dist_tol", m_dist_tol, 0.3);
  m_nh.param<double>("/control_loop_hz", m_control_loop_hz, 5.0);
  m_nh.param<bool>("/loop_enable", m_loop_enable, true);
  m_nh.param<vector<double>>("/waypoints", m_waypoints, vector<double>());
  if (m_waypoints.empty() || m_waypoints.size()%3 != 0)
  {
    ROS_ERROR("Waypoints are not multiple of 3!!! it should be x,y,z, x2,y2,z2, ...");
    return;
  }

  ///// ROS      
  // publishers
  m_position_controller_pub = m_nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 3);
  // subscribers
  m_pose_sub = m_nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 3, &drone_flight_class::pose_cb, this);
  m_state_sub = m_nh.subscribe<mavros_msgs::State>("/mavros/state", 10, &drone_flight_class::state_cb, this);
  // for mavros
  m_arming_client = m_nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
  m_kill_client = m_nh.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");
  m_set_mode_client = m_nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
  // Timer at the end
  m_control_timer = m_nh.createTimer(ros::Duration(1.0/m_control_loop_hz), &drone_flight_class::control_timer_func, this);
  
  cout << "\033[32;1mFlight class started...\033[0m" << endl;
}


//////////////////////// callbacks
void drone_flight_class::state_cb(const mavros_msgs::State::ConstPtr& msg)
{
  m_current_state = *msg;
  return;
}

void drone_flight_class::pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  m_current_pos_x = msg->pose.position.x;
  m_current_pos_y = msg->pose.position.y;
  m_current_pos_z = msg->pose.position.z;

  if (!m_pose_check)
  {
    cout << "\033[36;1mPose came in!!!\033[0m" << endl;
    m_pose_check = true;
  }
  return;
}

void drone_flight_class::control_timer_func(const ros::TimerEvent& event)
{
  if (!m_pose_check)
  {
    cout << "\033[35;1mPose did not come in yet waiting...\033[0m" << endl;
    return;
  }
  else if (m_pose_check)
  {
    ///// need to arm, offboarding, taking off
    if (!m_ctrl_init)
    {
      if(!m_current_state.armed) //waiting arming
      {
        cout << "\033[31;1mArming waiting...\033[0m" << endl;
        return;
      }
      else if(m_current_state.mode != "OFFBOARD") //trying to change mode
      {
        // sending any command to change mode
        geometry_msgs::PoseStamped goal_pose_;
        goal_pose_.pose.position.x = m_current_pos_x;
        goal_pose_.pose.position.y = m_current_pos_y;
        goal_pose_.pose.position.z = m_current_pos_z;
        goal_pose_.pose.orientation.w = 1.0;
        m_position_controller_pub.publish(goal_pose_);

        mavros_msgs::SetMode offboarding_command_;
        offboarding_command_.request.custom_mode = "OFFBOARD";
        m_set_mode_client.call(offboarding_command_);
        m_fixed_x_pos_temporal = m_current_pos_x;
        m_fixed_y_pos_temporal = m_current_pos_y;
        return;
      }
      else // armed & offboard, taking off
      {
        geometry_msgs::PoseStamped goal_pose_;
        goal_pose_.pose.position.x = m_fixed_x_pos_temporal;
        goal_pose_.pose.position.y = m_fixed_y_pos_temporal;
        goal_pose_.pose.position.z = m_taking_off_altitude;
        goal_pose_.pose.orientation.w = 1.0;
        m_position_controller_pub.publish(goal_pose_);
        if ( euclidean_dist(m_current_pos_x, m_current_pos_y, m_current_pos_z, m_fixed_x_pos_temporal, m_fixed_y_pos_temporal, m_taking_off_altitude) < m_dist_tol)
        {
          cout << "\033[34;1mTaking off done!\033[0m" << endl;
          m_ctrl_init=true;
        }
        return;
      }
    }

    ///// taking off done, now following waypoints
    else if (m_ctrl_init)
    {
      geometry_msgs::PoseStamped goal_pose_;
      goal_pose_.pose.position.x = m_waypoints[m_waypoint_index * 3];
      goal_pose_.pose.position.y = m_waypoints[m_waypoint_index * 3 + 1];
      goal_pose_.pose.position.z = m_waypoints[m_waypoint_index * 3 + 2];
      goal_pose_.pose.orientation.w = 1.0;
      m_position_controller_pub.publish(goal_pose_);
      if ( m_dist_tol > euclidean_dist(m_current_pos_x, m_current_pos_y, m_current_pos_z, 
        m_waypoints[m_waypoint_index * 3], m_waypoints[m_waypoint_index * 3 + 1], m_waypoints[m_waypoint_index * 3 + 2]))
      {
        if (m_waypoint_index+1 >= m_waypoints.size()/3)
        {
          if (m_loop_enable)
          {
            m_waypoint_index = 0;
            cout << "\033[36;1mReached the final waypoint, loop enabled! going to the first point\033[0m" << endl;
          }
          else
          {
            cout << "\033[31;1mReached the final waypoint...\033[0m" << endl;
          }
        }
        else
        {
          m_waypoint_index++;
          cout << "\033[32;1mReached a waypoint, moving to next!\033[0m" << endl;
        }
      }
      return;
    }
  }
}


#endif