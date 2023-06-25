#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Twist.h>
#include <urdf/model.h>
#include <sensor_msgs/JointState.h>

#include <string.h>
#include <stdio.h>
#include <math.h>
#include <assert.h>
#include <sstream>
#include <iostream>

#include <HL/hl.h>
#include <HD/hd.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>
#include <HDU/hduMatrix.h>
#include <HDU/hduQuaternion.h>
#define BT_EULER_DEFAULT_ZYX
#include <bullet/LinearMath/btMatrix3x3.h>

#include "omni_msgs/OmniButtonEvent.h"
#include "omni_msgs/OmniFeedback.h"
#include "omni_msgs/OmniState.h"
#include <pthread.h>

// #include "tf2_msgs/msg/tf_message.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/tf.h>

#include "robotiq_ft_sensor/rq_sensor_state.h"
#include "robotiq_ft_sensor/ft_sensor.h"
#include "robotiq_ft_sensor/sensor_accessor.h"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
float prev_time;
int calibrationStyle;


struct OmniState {
  hduVector3Dd position;  //3x1 vector of position
  hduVector3Dd velocity;  //3x1 vector of velocity
  hduVector3Dd inp_vel1;  //3x1 history of velocity used for filtering velocity estimate
  hduVector3Dd inp_vel2;
  hduVector3Dd inp_vel3;
  hduVector3Dd out_vel1;
  hduVector3Dd out_vel2;
  hduVector3Dd out_vel3;
  hduVector3Dd pos_hist1; //3x1 history of position used for 2nd order backward difference estimate of velocity
  hduVector3Dd pos_hist2;
  hduQuaternion rot;
  hduVector3Dd joints;
  hduVector3Dd force;   //3 element double vector force[0], force[1], force[2]
  float thetas[7];
  int buttons[2];
  int buttons_prev[2];
  bool lock;
  bool close_gripper;
  hduVector3Dd lock_pos;
  double units_ratio;
  hduVector3Dd key_pose;
  hduVector3Dd orient;
};

class PhantomROS {

public:
  ros::NodeHandle n;
  ros::Publisher state_publisher;
  ros::Publisher pose_publisher;
  ros::Publisher button_publisher;
  ros::Publisher joint_publisher;
  ros::Publisher teleop_publisher;
  ros::Subscriber force_key;
  ros::Subscriber haptic_sub;
  ros::Publisher rotation_publisher;
  ros::Subscriber force_sub;
  ros::Subscriber ur_forcesub;

  ros::Subscriber ur_sub;
  ros::Subscriber master_sub;
  ros::Subscriber qik_sub;
  ros::Subscriber qika_sub;
  ros::Subscriber qqpa_sub;
  ros::Subscriber qqp_sub;

  std::string omni_name, ref_frame, units;
  // ros::Subscriber button_subscriber;
  OmniState *state;



  void init(OmniState *s) {
    ros::param::param(std::string("~omni_name"), omni_name, std::string("phantom"));
    ros::param::param(std::string("~reference_frame"), ref_frame, std::string("/map"));
    ros::param::param(std::string("~units"), units, std::string("mm"));

    //Publish button state on NAME/button
    std::ostringstream stream1;
    stream1 << omni_name << "/button";
    std::string button_topic = std::string(stream1.str());
    button_publisher = n.advertise<omni_msgs::OmniButtonEvent>(button_topic.c_str(), 100);

    //Publish on NAME/state
    std::ostringstream stream2;
    stream2 << omni_name << "/state";
    std::string state_topic_name = std::string(stream2.str());
    state_publisher = n.advertise<omni_msgs::OmniState>(state_topic_name.c_str(), 1);

    //创建键盘接收对象
    force_key = n.subscribe<geometry_msgs::Twist>("/cmd_vel",1,&PhantomROS::dokey,this);

    //创建主手发布者对象
    teleop_publisher = n.advertise<geometry_msgs::Twist>("/phantom/Twist_msgs",100);
    //创建旋转向量发布对象
    rotation_publisher = n.advertise<geometry_msgs::Twist>("/phantom/rotation_msg",100);

    //创建力接受对象
    force_sub = n.subscribe<robotiq_ft_sensor::ft_sensor>("robotiq_ft_pub_sensor",100,&PhantomROS::doforce,this);

    ur_sub = n.subscribe<geometry_msgs::Twist>("ur_position",100,&PhantomROS::doursub,this);
    master_sub = n.subscribe<geometry_msgs::Twist>("master_position",100,&PhantomROS::domastersub,this);
    qik_sub = n.subscribe<geometry_msgs::Twist>("qik_v",100,&PhantomROS::doqiksub,this);
    qika_sub = n.subscribe<geometry_msgs::Twist>("qik_a",100,&PhantomROS::doqikasub,this);
    qqpa_sub = n.subscribe<geometry_msgs::Twist>("qqp_a",100,&PhantomROS::doqqpasub,this);
    qqp_sub = n.subscribe<geometry_msgs::Twist>("qqp_v",100,&PhantomROS::doqqpsub,this);
    ur_forcesub = n.subscribe<geometry_msgs::Twist>("ur_force",100,&PhantomROS::doqqpsub,this);

   
    //Subscribe to NAME/force_feedback
    std::ostringstream stream3;
    stream3 << omni_name << "/force_feedback";
    std::string force_feedback_topic = std::string(stream3.str());
    haptic_sub = n.subscribe(force_feedback_topic.c_str(), 1, &PhantomROS::force_callback, this);

    //Publish on NAME/pose
    std::ostringstream stream4;
    stream4 << omni_name << "/pose";
    std::string pose_topic_name = std::string(stream4.str());
    pose_publisher = n.advertise<geometry_msgs::PoseStamped>(pose_topic_name.c_str(), 1);

    //Publish on NAME/joint_states
    std::ostringstream stream5;
    stream5 << omni_name << "/joint_states";
    std::string joint_topic_name = std::string(stream5.str());
    joint_publisher = n.advertise<sensor_msgs::JointState>(joint_topic_name.c_str(), 1);

    state = s;
    state->buttons[0] = 0;
    state->buttons[1] = 0;
    state->buttons_prev[0] = 0;
    state->buttons_prev[1] = 0;
    hduVector3Dd zeros(0, 0, 0);
    state->velocity = zeros;
    state->inp_vel1 = zeros;  //3x1 history of velocity
    state->inp_vel2 = zeros;  //3x1 history of velocity
    state->inp_vel3 = zeros;  //3x1 history of velocity
    state->out_vel1 = zeros;  //3x1 history of velocity
    state->out_vel2 = zeros;  //3x1 history of velocity
    state->out_vel3 = zeros;  //3x1 history of velocity
    state->pos_hist1 = zeros; //3x1 history of position
    state->pos_hist2 = zeros; //3x1 history of position
    state->key_pose = zeros;
    state->lock = false;
    state->close_gripper = false;
    state->lock_pos = zeros;
    state->orient = zeros;
    if (!units.compare("mm"))
      state->units_ratio = 1.0;
    else if (!units.compare("cm"))
      state->units_ratio = 10.0;
    else if (!units.compare("dm"))
      state->units_ratio = 100.0;
    else if (!units.compare("m"))
      state->units_ratio = 1000.0;
    else
    {
      state->units_ratio = 1.0;
      ROS_WARN("Unknown units [%s] unsing [mm]", units.c_str());
      units = "mm";
    }
    ROS_INFO("PHaNTOM position given in [%s], ratio [%.1f]", units.c_str(), state->units_ratio);
  }



  void publish_omni_state() {
    // Build the state msg
    omni_msgs::OmniState state_msg;
    // Locked
    state_msg.locked = state->lock;
    state_msg.close_gripper = state->close_gripper;
    // Position
    state_msg.pose.position.x = state->position[0];
    state_msg.pose.position.y = state->position[1];
    state_msg.pose.position.z = state->position[2];
    // Orientation
    state_msg.pose.orientation.x = state->rot.v()[0];
    state_msg.pose.orientation.y = state->rot.v()[1];
    state_msg.pose.orientation.z = state->rot.v()[2];
    state_msg.pose.orientation.w = state->rot.s();

//发布旋转信息
    geometry_msgs::Twist rotation_msg;

    // Eigen::Quaterniond quaternion(state_msg.pose.orientation.w,state_msg.pose.orientation.x,state_msg.pose.orientation.y,state_msg.pose.orientation.z);
    // Eigen::AngleAxisd rotation_vector(quaternion);
    // Eigen::Vector3d  rotation_v = rotation_vector.axis();//单位向量
    // float theta = rotation_vector.angle();//旋转角度

    rotation_msg.angular.x=state_msg.pose.orientation.x;
    rotation_msg.angular.y=state_msg.pose.orientation.y;
    rotation_msg.angular.z=state_msg.pose.orientation.z;
    rotation_msg.linear.x = state_msg.pose.orientation.w;
    rotation_publisher.publish(rotation_msg);

    // float k_x = rotation_v(0) * theta;
    // float k_y = rotation_v(1) * theta;
    // float k_z = rotation_v(2) * theta;




    // ROS_INFO("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",rotation_v(0),rotation_v(1),rotation_v(2),rotation_v(3),rotation_v(4),rotation_v(5),rotation_v(6),rotation_v(7),rotation_v(8));
// rotation_vector(0);

    // float k_x = state_msg.pose.orientation.x * asin(theta/2);
    // float k_y = state_msg.pose.orientation.y * asin(theta/2);
    // float k_z = state_msg.pose.orientation.z * asin(theta/2);
    // ROS_INFO("%.2f,%.2f,%.2f",k_x,k_y,k_z);


    // Velocity
    state_msg.velocity.x = state->velocity[0];
    state_msg.velocity.y = state->velocity[1];
    state_msg.velocity.z = state->velocity[2];


//通过给定位置实现主手位置始终保持在给定点上
    // if(state->position[0] < state->key_pose[0])
    // {
    //   state->force[0] = (state->key_pose[0] - state->position[0])*0.1;
    // }
    // else if(state->position[0] > state->key_pose[0])   
    // {
    //   state->force[0] = -(state->position[0] - state->key_pose[0])*0.1;
    // }
    // else if(state->position[0] = state->key_pose[0])
    // {
    //   state->force[0] = 0;
    // }

    // if(state->position[1] < state->key_pose[1])
    // {
    //   state->force[1] = (state->key_pose[1] - state->position[1])*0.1;
    // }
    // else if(state->position[1] > state->key_pose[1])   
    // {
    //   state->force[1] = -(state->position[1] - state->key_pose[1])*0.1;
    // }
    // else if(state->position[1] = state->key_pose[1])
    // {
    //   state->force[1] = 0;
    // }

    // if(state->position[2] < state->key_pose[2])
    // {
    //   state->force[2] = (state->key_pose[2] - state->position[2])*0.1;
    // }
    // else if(state->position[2] > state->key_pose[2])   
    // {
    //   state->force[2] = -(state->position[2] - state->key_pose[2])*0.1;
    // }
    // else if(state->position[2] = state->key_pose[2])
    // {
    //   state->force[2] = 0;
    // }

    
    // TODO: Append Current to the state msg
    state_msg.header.stamp = ros::Time::now();
    state_publisher.publish(state_msg);



    // Publish the JointState msg
    sensor_msgs::JointState joint_state;
    joint_state.header.stamp = ros::Time::now();
    joint_state.name.resize(6);
    joint_state.position.resize(6);
    joint_state.name[0] = "waist";
    joint_state.position[0] = -state->thetas[1];
    joint_state.name[1] = "shoulder";
    joint_state.position[1] = state->thetas[2];
    joint_state.name[2] = "elbow";
    joint_state.position[2] = state->thetas[3];
    joint_state.name[3] = "yaw";
    joint_state.position[3] = -state->thetas[4] + M_PI;
    joint_state.name[4] = "pitch";
    joint_state.position[4] = -state->thetas[5] - 3*M_PI/4;
    joint_state.name[5] = "roll";
    joint_state.position[5] = -state->thetas[6] - M_PI;
    joint_publisher.publish(joint_state);

    // Build the pose msg
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header = state_msg.header;
    pose_msg.header.frame_id = ref_frame;
    pose_msg.pose = state_msg.pose;
    pose_msg.pose.position.x /= 1000.0;
    pose_msg.pose.position.y /= 1000.0;
    pose_msg.pose.position.z /= 1000.0;
    pose_publisher.publish(pose_msg);

    // if ((state->buttons[0] != state->buttons_prev[0])
    // //state_button 默认是0，按下去是1，抬起来是0
    // //buttons_prev 是button的没有按时候的值，初始化为0，这里是if button 按了一次
    //     or (state->buttons[1] != state->buttons_prev[1]))
    // {
    //     state->close_gripper = !(state->close_gripper);//button==1  ->   close_gripper 取反
    //     state->buttons_prev[0]=state->buttons[0];
    //     state->buttons_prev[1]=state->buttons[1];
    //     // close_gripper 为false
    //     // 按下去第一次，即希望不传递数值，此时close_gripper 为true
    //     // 按下去第二次，即希望继续传递数值，此时close_gripper 为false
      


      omni_msgs::OmniButtonEvent button_event;
      button_event.grey_button = state->buttons[0];//grey->0
      button_event.white_button = state->buttons[1];//white->1
      // state->buttons_prev[0] = state->buttons[0];
      // state->buttons_prev[1] = state->buttons[1];
      button_publisher.publish(button_event);
    // }

    geometry_msgs::Twist teleop_msg;
    teleop_msg.linear.x = state->position[0];
    teleop_msg.linear.y = state->position[1];
    teleop_msg.linear.z = state->position[2];
    // teleop_msg.angular.x = k_x;
    // teleop_msg.angular.y = k_y;
    // teleop_msg.angular.z = k_z;


    // ROS_INFO("X=%.2f,Y=%.2f,Z=%.2f",teleop_msg.angular.x,teleop_msg.angular.y,teleop_msg.angular.z);

    teleop_publisher.publish(teleop_msg);



  }

    /*******************************************************************************
   ROS node callback.
   *******************************************************************************/

  // void dobutton(const omni_msgs::OmniButtonEvent::ConstPtr& button_state){
  //     state->buttons[0] = button_state->grey_button;//grey->0
  //     state->buttons[1] = button_state->white_button;//white->1

  // }
  void doforce(const robotiq_ft_sensor::ft_sensor::ConstPtr& force_msg){
    
    state->force[0] = force_msg->Fx;
    state->force[1] = force_msg->Fy;
    state->force[2] = force_msg->Fz;
    for(int k=0;k<3;k++){
      if(state->force[k]>1)
      {
        state->force[k]=1;
      }
    }
  }
  void domastersub (const geometry_msgs::Twist::ConstPtr& key_msg){}
  void doursub(const geometry_msgs::Twist::ConstPtr& key_msg){}
  void doqiksub(const geometry_msgs::Twist::ConstPtr& key_msg){}
  void doqikasub(const geometry_msgs::Twist::ConstPtr& key_msg){}
  void doqqpsub(const geometry_msgs::Twist::ConstPtr& key_msg){}
  void doqqpasub(const geometry_msgs::Twist::ConstPtr& key_msg){}
  void dourforce(const geometry_msgs::Twist::ConstPtr& key_msg){}
  
  void dokey(const geometry_msgs::Twist::ConstPtr& key_msg){


// 键盘控制 话题改为/cmd_vel
    // state->key_pose[0]=state->key_pose[0]+key_msg->linear.x;//linear保存目标xyz位置
    // state->key_pose[1]=state->key_pose[1]+key_msg->linear.y;
    // state->key_pose[2]=state->key_pose[2]+key_msg->linear.z;

// 命令行控制
    // state->force[0] = key_msg->linear.x;//linear发送力信息
    // state->force[1] = key_msg->linear.y;
    // state->force[2] = key_msg->linear.z;

    // state->key_pose[0]=key_msg->angular.x;//angular保存目标xyz位置
    // state->key_pose[1]=key_msg->angular.y;
    // state->key_pose[2]=key_msg->angular.z;


    // if(state->position[0] < key_msg->angular.x)
    // {
    //   state->force[0] = state->force[0]+(key_msg->angular.x - state->position[0])*0.1;
    // }
    // else if(state->position[0] > key_msg->angular.x)   
    // {
    //   state->force[0] = state->force[0]+(state->position[0] - key_msg->angular.x)*0.1;
    // }


  }

  void force_callback(const omni_msgs::OmniFeedbackConstPtr& omnifeed) {
    ////////////////////Some people might not like this extra damping, but it
    ////////////////////helps to stabilize the overall force feedback. It isn't
    ////////////////////like we are getting direct impedance matching from the
    ////////////////////omni anyway
    // state->force[0] = omnifeed->force.x - 0.001 * state->velocity[0];
    // state->force[1] = omnifeed->force.y - 0.001 * state->velocity[1];
    // state->force[2] = omnifeed->force.z - 0.001 * state->velocity[2];

    // state->lock_pos[0] = omnifeed->position.x;
    // state->lock_pos[1] = omnifeed->position.y;
    // state->lock_pos[2] = omnifeed->position.z;
    // if(state->position[0] < state->key_pose[0])
    // {
    //   state->force[0] = state->force[0]+(state->key_pose[0] - state->position[0])*0.1;
    // }
    // else if(state->position[0] > state->key_pose[0])   
    // {
    //   state->force[0] = state->force[0]+(state->position[0] - state->key_pose[0])*0.1;
    // }

  }



};

HDCallbackCode HDCALLBACK omni_state_callback(void *pUserData)
{
  OmniState *omni_state = static_cast<OmniState *>(pUserData);
  if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_UPDATE) {
    ROS_DEBUG("Updating calibration...");
      hdUpdateCalibration(calibrationStyle);
    }
  hdBeginFrame(hdGetCurrentDevice());
  // Get transform and angles
  hduMatrix transform;
  hdGetDoublev(HD_CURRENT_TRANSFORM, transform);
  hdGetDoublev(HD_CURRENT_JOINT_ANGLES, omni_state->joints);
  hduVector3Dd gimbal_angles;
  hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES, gimbal_angles);
  // Notice that we are inverting the Z-position value and changing Y <---> Z
  // Position
  omni_state->position = hduVector3Dd(transform[3][0], -transform[3][2], transform[3][1]);
  omni_state->position /= omni_state->units_ratio;
  // Orientation (quaternion)
  hduMatrix rotation(transform);
  rotation.getRotationMatrix(rotation);
  hduMatrix rotation_offset( 0.0,  -1.0, 0.0, 0.0,
                             1.0,  0.0, 0.0, 0.0,
                             0.0,  0.0, 1.0, 0.0,
                             0.0,  0.0, 0.0, 1.0);
  rotation_offset.getRotationMatrix(rotation_offset);
  omni_state->rot = hduQuaternion(rotation_offset * rotation);
  // Velocity estimation
  hduVector3Dd vel_buff(0, 0, 0);
  vel_buff = (omni_state->position * 3 - 4 * omni_state->pos_hist1
      + omni_state->pos_hist2) / 0.002;  //(units)/s, 2nd order backward dif
  omni_state->velocity = (.2196 * (vel_buff + omni_state->inp_vel3)
      + .6588 * (omni_state->inp_vel1 + omni_state->inp_vel2)) / 1000.0
      - (-2.7488 * omni_state->out_vel1 + 2.5282 * omni_state->out_vel2
          - 0.7776 * omni_state->out_vel3);  //cutoff freq of 20 Hz
  omni_state->pos_hist2 = omni_state->pos_hist1;
  omni_state->pos_hist1 = omni_state->position;
  omni_state->inp_vel3 = omni_state->inp_vel2;
  omni_state->inp_vel2 = omni_state->inp_vel1;
  omni_state->inp_vel1 = vel_buff;
  omni_state->out_vel3 = omni_state->out_vel2;
  omni_state->out_vel2 = omni_state->out_vel1;
  omni_state->out_vel1 = omni_state->velocity;

  //~ // Set forces if locked
  //~ if (omni_state->lock == true) {
    //~ omni_state->force = 0.04 * omni_state->units_ratio * (omni_state->lock_pos - omni_state->position)
        //~ - 0.001 * omni_state->velocity;
  //~ }
  hduVector3Dd feedback;
  // Notice that we are changing Y <---> Z and inverting the Z-force_feedback
  feedback[0] = omni_state->force[0];
  feedback[1] = omni_state->force[2];
  feedback[2] = -omni_state->force[1];
  hdSetDoublev(HD_CURRENT_FORCE, feedback);

  //Get buttons
  int nButtons = 0;
  hdGetIntegerv(HD_CURRENT_BUTTONS, &nButtons);
  omni_state->buttons[0] = (nButtons & HD_DEVICE_BUTTON_1) ? 1 : 0;
  omni_state->buttons[1] = (nButtons & HD_DEVICE_BUTTON_2) ? 1 : 0;

  hdEndFrame(hdGetCurrentDevice());

  HDErrorInfo error;
  if (HD_DEVICE_ERROR(error = hdGetError())) {
    hduPrintError(stderr, &error, "Error during main scheduler callback");
    if (hduIsSchedulerError(&error))
      return HD_CALLBACK_DONE;
  }

  float t[7] = { 0., omni_state->joints[0], omni_state->joints[1],
      omni_state->joints[2] - omni_state->joints[1], gimbal_angles[0],
      gimbal_angles[1], gimbal_angles[2] };
  for (int i = 0; i < 7; i++)
    omni_state->thetas[i] = t[i];
  return HD_CALLBACK_CONTINUE;
}

/*******************************************************************************
 Automatic Calibration of Phantom Device - No character inputs
 *******************************************************************************/
void HHD_Auto_Calibration() {
  int supportedCalibrationStyles;
  HDErrorInfo error;

  hdGetIntegerv(HD_CALIBRATION_STYLE, &supportedCalibrationStyles);
  if (supportedCalibrationStyles & HD_CALIBRATION_ENCODER_RESET) {
    calibrationStyle = HD_CALIBRATION_ENCODER_RESET;
    ROS_INFO("HD_CALIBRATION_ENCODER_RESET..");
  }
  if (supportedCalibrationStyles & HD_CALIBRATION_INKWELL) {
    calibrationStyle = HD_CALIBRATION_INKWELL;
    ROS_INFO("HD_CALIBRATION_INKWELL..");
  }
  if (supportedCalibrationStyles & HD_CALIBRATION_AUTO) {
    calibrationStyle = HD_CALIBRATION_AUTO;
    ROS_INFO("HD_CALIBRATION_AUTO..");
  }
  if (calibrationStyle == HD_CALIBRATION_ENCODER_RESET) {
    do {
      hdUpdateCalibration(calibrationStyle);
      ROS_INFO("Calibrating.. (put stylus in well)");
      if (HD_DEVICE_ERROR(error = hdGetError())) {
        hduPrintError(stderr, &error, "Reset encoders reset failed.");
        break;
      }
    } while (hdCheckCalibration() != HD_CALIBRATION_OK);
    ROS_INFO("Calibration complete.");
  }
  while(hdCheckCalibration() != HD_CALIBRATION_OK) {
    usleep(1e6);
    if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_MANUAL_INPUT)
      ROS_INFO("Please place the device into the inkwell for calibration");
    else if (hdCheckCalibration() == HD_CALIBRATION_NEEDS_UPDATE) {
      ROS_INFO("Calibration updated successfully");
      hdUpdateCalibration(calibrationStyle);
    }
    else
      ROS_FATAL("Unknown calibration status");
  }
}

void *ros_publish(void *ptr) {
  PhantomROS *omni_ros = (PhantomROS *) ptr;
  int publish_rate;
  ros::param::param(std::string("~publish_rate"), publish_rate, 1000);
  ROS_INFO("Publishing PHaNTOM state at [%d] Hz", publish_rate);
  ros::Rate loop_rate(publish_rate);
  ros::AsyncSpinner spinner(2);
  spinner.start();

  while (ros::ok()) {
    omni_ros->publish_omni_state();

    ros::spinOnce();

    loop_rate.sleep();
  }
  return NULL;
}

int main(int argc, char** argv) {
  ////////////////////////////////////////////////////////////////
  // Init Phantom
  ////////////////////////////////////////////////////////////////
  HDErrorInfo error;
  HHD hHD;
  hHD = hdInitDevice(HD_DEFAULT_DEVICE);
  if (HD_DEVICE_ERROR(error = hdGetError())) {
    //hduPrintError(stderr, &error, "Failed to initialize haptic device");
    ROS_ERROR("Failed to initialize haptic device"); //: %s", &error);
    return -1;
  }

  ROS_INFO("Found %s.", hdGetString(HD_DEVICE_MODEL_TYPE));
  hdEnable(HD_FORCE_OUTPUT);
  hdStartScheduler();
  if (HD_DEVICE_ERROR(error = hdGetError())) {
    ROS_ERROR("Failed to start the scheduler"); //, &error);
    return -1;
  }
  HHD_Auto_Calibration();

  ////////////////////////////////////////////////////////////////
  // Init ROS
  ////////////////////////////////////////////////////////////////
  ros::init(argc, argv, "omni_haptic_node");
  OmniState state;
  PhantomROS omni_ros;

  omni_ros.init(&state);
  hdScheduleAsynchronous(omni_state_callback, &state,
      HD_MAX_SCHEDULER_PRIORITY);

  ////////////////////////////////////////////////////////////////
  // Loop and publish
  ////////////////////////////////////////////////////////////////
  pthread_t publish_thread;
  pthread_create(&publish_thread, NULL, ros_publish, (void*) &omni_ros);
  pthread_join(publish_thread, NULL);

  ROS_INFO("Ending Session....");
  hdStopScheduler();
  hdDisableDevice(hHD);

  return 0;
}
