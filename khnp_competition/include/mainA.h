#ifndef KHNP_MAIN_H
#define KHNP_MAIN_H

///// Qt GUI elements
#include <QtCore>
#include <QApplication>
#include <QtWidgets>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QImage>
#include <QPixmap>
#include <QPushButton>
#include <QIcon>

//QT utils
#include <QTimer> // important, or GUI freezes
#include <QFrame>
#include <QString>
#include <QPalette>
#include <QFont>

///// common headers
#include <ros/ros.h>
#include <ros/package.h> // get path
#include <iostream> //cout
#include <fstream>
#include <string>
#include <math.h> // pow
#include <vector>
#include <random> //random

///// Utils
#include <tf/LinearMath/Quaternion.h> // to Quaternion_to_euler
#include <tf/LinearMath/Matrix3x3.h> // to Quaternion_to_euler

///// time for Gazebo
#include <rosgraph_msgs/Clock.h>

///// headers
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/SpawnModel.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>

///// image processing
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include "courseA.h"

using namespace std;

///// utils
#include <signal.h>
void signal_handler(sig_atomic_t s) {
  std::cout << "You pressed Ctrl + C, exiting" << std::endl;
  exit(1);
}
template <typename T1, typename T2, typename T3>
bool within_range(T1 a, T2 b, T3 c){
  if (abs(a.x-b.x)<c.x && abs(a.y-b.y)<c.y && abs(a.z-b.z)<c.z)
    return true;
  else
    return false;
}


////////////////////////////////////////////////////////////////////////////////////////////////////
class khnp_comp: public QWidget{
  private:
    // no meaning for private, just separated QT variables
    QTimer *mainTimer;
    QHBoxLayout *main_hbox;
    QVBoxLayout *left_vbox, *right_vbox;
    QHBoxLayout *right_hbox_btns, *right_hbox1, *right_hbox2, *right_hbox3, *right_hbox4, *right_hbox5, *right_hbox6, *right_hbox_result;
    QLabel *left_text1, *left_text2, *left_3rd_img, *left_1st_img;
    QLabel *right_text1, *right_text2, *right_text3, *right_text4, *right_text9;
    QLabel *right_text5, *right_text6, *right_text7, *right_text8, *right_text10;
    QLabel *right_creator, *right_logo, *right_result;
    QPushButton *falldown_button, *pause_button, *reset_button, *skip_button, *refresh_button;

    QPalette palette;
    QFont font;
    QColor cyan=QColor(121,215,252);
    QColor palegreen=QColor(172,252,186);
    QColor palepurple=QColor(228,191,255);
    QColor lightred=QColor(255,77,115);
    int iconsize=100;

    cv::Mat third_cam_cv_img, first_cam_cv_img, logo_img, pause_img, paused_img, falldown_img, fell_img, reset_img, skip_img;
    string path;
    bool paused_check=false, skip_check=false;

    void QT_initialize();
    void qt_img_update(QLabel *label, cv::Mat img);
    void qt_icon_update(QPushButton *btn, cv::Mat img);
    void falldown_button_callback();
    void pause_button_callback();
    void reset_button_callback();
    void skip_button_callback();
    void finish_result();
    void nothing();

    void qt_timer_func();

  public:
    // no meaning for public, just separate ROS and main variables
    gazebo_msgs::ModelStates states;
    gazebo_msgs::ModelState cam_pose, robot_pose, other_pose;
    gazebo_msgs::SetModelState model_move_srv;
    std_srvs::Empty empty_srv;
    std_msgs::Empty empty_msg;
    rosgraph_msgs::Clock real_current_time, fixed_current_time, fixed_course_time, fell_down_time;

    bool initialized=false, qt_initialized=false, state_check=false, clock_check=false, third_cam_check=false, first_cam_check=false;
    bool first_clock_in=false, if_felldown_flag=false, tmp_felldown_counter=false, if_finished=false;
    std::string robot_name, cube_name, third_cam_name, third_cam_topic, first_cam_topic;
    int robot_idx=0, cube_idx=0, img_width, img_height, current_score=0, falldown_score=0;
    position3d tolerance={0.15, 0.6, 0.5};
    position3d sphere_tolerance={0.15, 1000.0, 0.5};
    position3d cube_tolerance={0.25, 0.6, 1.5};


    ///// ros and tf
    ros::NodeHandle nh;
    ros::Subscriber states_sub, third_cam_sub, first_cam_sub, clock_sub;
    ros::Publisher spawning_msg_pub;
    ros::ServiceClient model_mover, model_spawner, pauser, unpauser, resetter;
    ros::Timer main_timer, qt_timer, sphere_timer, felldown_timer;

    void main_timer_func(const ros::TimerEvent& event);
    void main_initialize();
    void sphere_time_func(const ros::TimerEvent& event);
    void if_felldown_time_func(const ros::TimerEvent& event);
    bool if_felldown(geometry_msgs::Pose pose);
    void cam_move(geometry_msgs::Pose pose);
    void if_time_over(double time_left);
    void if_started_course(geometry_msgs::Pose pose);
    void if_passed_course(geometry_msgs::Pose pose);
    void move_to_current_course();
    void move_to_next_course();
    bool move_to_next_map();
    void states_callback(const gazebo_msgs::ModelStates::ConstPtr& msg);
    void third_cam_callback(const sensor_msgs::CompressedImage::ConstPtr& msg);
    void first_cam_callback(const sensor_msgs::CompressedImage::ConstPtr& msg);
    void clock_callback(const rosgraph_msgs::Clock::ConstPtr& msg);

    khnp_comp(ros::NodeHandle& n, QWidget *parent=0) : nh(n), QWidget(parent){
      ///// params
      nh.param("/img_width", img_width, 480);
      nh.param("/img_height", img_height, 320);
      nh.param<std::string>("/robot_name", robot_name, "/");
      nh.param<std::string>("/cube_name", cube_name, "b2");
      nh.param<std::string>("/third_cam_name", third_cam_name, "third_camera");
      nh.param<std::string>("/third_cam_topic", third_cam_topic, "/third_camera/rgb/image_raw/compressed");
      nh.param<std::string>("/first_cam_topic", first_cam_topic, "/d455/depth/rgb_image_raw/compressed");

      ///// Init
      course_initilization();
      path = ros::package::getPath("khnp_competition");
      main_initialize();
      QT_initialize();

      ///// sub pub
      states_sub = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states", 3, &khnp_comp::states_callback, this);
      third_cam_sub = nh.subscribe<sensor_msgs::CompressedImage>(third_cam_topic, 10, &khnp_comp::third_cam_callback, this);
      first_cam_sub = nh.subscribe<sensor_msgs::CompressedImage>(first_cam_topic, 10, &khnp_comp::first_cam_callback, this);
      clock_sub = nh.subscribe<rosgraph_msgs::Clock>("/clock", 3, &khnp_comp::clock_callback, this);

      model_mover = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
      pauser = nh.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
      unpauser = nh.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");
      resetter = nh.serviceClient<std_srvs::Empty>("/gazebo/reset_world");

      spawning_msg_pub = nh.advertise<std_msgs::Empty>("/spawning_model", 2);

      main_timer = nh.createTimer(ros::Duration(1/12.0), &khnp_comp::main_timer_func, this); 
      sphere_timer = nh.createTimer(ros::Duration(1/2.0), &khnp_comp::sphere_time_func, this); // every 1/2 second.
      felldown_timer = nh.createTimer(ros::Duration(1/3.0), &khnp_comp::if_felldown_time_func, this); // every 1/3 second.

      mainTimer = new QTimer(this);
      connect(mainTimer, &QTimer::timeout, [=](){qt_timer_func();} );
      mainTimer->start(80);

      ROS_WARN("class heritated, starting node...");
    }
    ~khnp_comp(){}
};











/////////// definitions, can be separated to .cpp
void khnp_comp::main_timer_func(const ros::TimerEvent& event){
  if (initialized && qt_initialized && state_check && third_cam_check && first_cam_check){
    cam_move(states.pose[robot_idx]);
    if(!if_finished){
      ros::Duration temp2 = courseAB[current_map].time_limit - (real_current_time.clock-fixed_course_time.clock);
      if_time_over(temp2.sec + temp2.nsec*1e-9);
      if_started_course(states.pose[robot_idx]);
      if_passed_course(states.pose[robot_idx]);
    }
  }
  else if (!third_cam_check){
    spawning_msg_pub.publish(empty_msg);
  }
  else{
    cout << initialized << qt_initialized << state_check << third_cam_check << first_cam_check << endl;
  }
}

void khnp_comp::qt_timer_func(){
  if (initialized && qt_initialized && state_check && third_cam_check && first_cam_check){
    if (!third_cam_cv_img.empty() and !first_cam_cv_img.empty()){
      qt_img_update(left_3rd_img, third_cam_cv_img);
      qt_img_update(left_1st_img, first_cam_cv_img);
    }
    if(!if_finished){
      ros::Duration temp2 = courseAB[current_map].time_limit - (real_current_time.clock-fixed_course_time.clock);
      right_text7->setText(QString::number(temp2.sec + temp2.nsec*1e-9,'g',7));

      ros::Duration temp = real_current_time.clock-fixed_current_time.clock;
      right_text8->setText(QString::number(temp.sec + temp.nsec*1e-9,'g',7));
    }
  }
}

void khnp_comp::main_initialize(){
  cam_pose.model_name=third_cam_name;
  robot_pose.model_name=robot_name;
  initialized=true;
}

void khnp_comp::cam_move(geometry_msgs::Pose pose){
  cam_pose.pose = pose;
  cam_pose.pose.position.z += 5.0;
  cam_pose.pose.orientation.x = 0.0; cam_pose.pose.orientation.y = 0.0; cam_pose.pose.orientation.z = 0.0; cam_pose.pose.orientation.w = 1.0;
  model_move_srv.request.model_state = cam_pose;
  model_mover.call(model_move_srv);
}

void khnp_comp::qt_img_update(QLabel *label, cv::Mat img){
  cv::Mat vis_img;
  if (img.cols != img_width or img.rows != img_height){
    cv::resize(img, vis_img, cv::Size(img_width, img_height));
    cv::cvtColor(vis_img, vis_img, CV_BGR2RGB);
  }
  else{
    cv::cvtColor(img, vis_img, CV_BGR2RGB);
  }
  try{
    QImage imgIn= QImage((uchar*) vis_img.data, vis_img.cols, vis_img.rows, vis_img.step, QImage::Format_RGB888);
    QPixmap pixmap = QPixmap::fromImage(imgIn);
    label->setPixmap(pixmap);
  }
  catch(...){
    return;
  }
}

void khnp_comp::qt_icon_update(QPushButton *btn, cv::Mat img){
  try{
    QImage imgIn= QImage((uchar*) img.data, img.cols, img.rows, img.step, QImage::Format_RGB888);
    QPixmap pixmap = QPixmap::fromImage(imgIn);
    btn->setIcon(pixmap);
  }
  catch(...){
    return;
  }
}

bool khnp_comp::if_felldown(geometry_msgs::Pose pose){
  double curr_roll, curr_pitch, curr_yaw;
  tf::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  tf::Matrix3x3 m(q);
  m.getRPY(curr_roll, curr_pitch, curr_yaw);
  if (abs(curr_roll)>1.309 or abs(curr_pitch)>1.309)
    return true;
  else return false;
  // else if(){

  // }
  // else if(){

  // }
}

void khnp_comp::if_felldown_time_func(const ros::TimerEvent& event){
  if (initialized && qt_initialized && state_check && third_cam_check && first_cam_check){
    if(!if_felldown_flag){
      if(!tmp_felldown_counter){
        if (if_felldown(states.pose[robot_idx])){
          ROS_WARN("Fell down check start");
          tmp_felldown_counter=true;
          fell_down_time=real_current_time;
        }
      }
      else if (tmp_felldown_counter){
        if (if_felldown(states.pose[robot_idx])){ // still fell down
          if ( (real_current_time.clock - fell_down_time.clock).toSec() >= 5.0 ){
            ROS_WARN("Fell down! penalty!");
            if_felldown_flag=true;
            qt_icon_update(falldown_button, fell_img);
            current_score-=3;
            falldown_score-=3;
            right_text5->setText(QString::number(current_score,'g',7));
            right_text10->setText(QString::number(falldown_score,'g',7));
            tmp_felldown_counter=false;
          }
        }
        else{ // stand again
          ROS_WARN("Stood up again");
          tmp_felldown_counter=false;
        }
      }
    }
    else if(if_felldown_flag){ // still fell down, longer than 5 seconds
      if(!tmp_felldown_counter){
        if (if_felldown(states.pose[robot_idx])){
          ROS_WARN("Still fell down check start");
          tmp_felldown_counter=true;
        }
      }
      else if (tmp_felldown_counter){
        if (if_felldown(states.pose[robot_idx])){
          if ( (real_current_time.clock - fell_down_time.clock).toSec() >= 125.0 ){
            ROS_WARN("Still fell down longer than 2 mins, penalty!");
            qt_icon_update(falldown_button, falldown_img);
            current_score-=2;
            falldown_score-=2;
            right_text5->setText(QString::number(current_score,'g',7));
            right_text10->setText(QString::number(falldown_score,'g',7));
            if_felldown_flag=false;
            tmp_felldown_counter=false;
            move_to_current_course();
            fixed_current_time.clock -= ros::Duration(120.0);
          }
        }
        else{ // stand again
          ROS_WARN("Stood up again");
          qt_icon_update(falldown_button, falldown_img);
          tmp_felldown_counter=false;
          if_felldown_flag=false;
        }
      }
    }
  }
}

void khnp_comp::sphere_time_func(const ros::TimerEvent& event){
  if (initialized && qt_initialized && state_check && third_cam_check && first_cam_check){
    if(courseAB[current_map].name==disturbance_map.name){
      if (!spheres_throw_vec[current_course].if_throw){
        if(within_range(states.pose[robot_idx].position, spheres_throw_vec[current_course].reference_position, sphere_tolerance)){

          if (spheres_throw_vec[current_course].direction=="back"){
            other_pose.pose.position.x = states.pose[robot_idx].position.x-1.0;
            other_pose.pose.position.y = states.pose[robot_idx].position.y; 
            other_pose.twist.linear.x = 5.0;
            other_pose.twist.linear.y = 0.0;
          }
          else if (spheres_throw_vec[current_course].direction=="front"){
            other_pose.pose.position.x = states.pose[robot_idx].position.x+1.0;
            other_pose.pose.position.y = states.pose[robot_idx].position.y; 
            other_pose.twist.linear.x = -5.0;
            other_pose.twist.linear.y = 0.0;
          }
          else if (spheres_throw_vec[current_course].direction=="left"){
            other_pose.pose.position.x = states.pose[robot_idx].position.x;
            other_pose.pose.position.y = states.pose[robot_idx].position.y+1.0; 
            other_pose.twist.linear.x = 0.0;
            other_pose.twist.linear.y = -5.0;
          }
          else if (spheres_throw_vec[current_course].direction=="right"){
            other_pose.pose.position.x = states.pose[robot_idx].position.x;
            other_pose.pose.position.y = states.pose[robot_idx].position.y-1.0; 
            other_pose.twist.linear.x = 0.0;
            other_pose.twist.linear.y = 5.0;
          }
          else if (spheres_throw_vec[current_course].direction=="random"){
            random_device rd;  mt19937 gen(rd());  std::uniform_int_distribution<int> dis(0, 1000); int rand_tmp = dis(gen);
            double rand_tmp2 = sqrt( 1 - pow(double(rand_tmp)/1000.0, 2) );
            other_pose.pose.position.x = states.pose[robot_idx].position.x+(double)rand_tmp/1000.0;
            other_pose.pose.position.y = states.pose[robot_idx].position.y+rand_tmp2; 
            other_pose.twist.linear.x = -(double)rand_tmp/200.0;
            other_pose.twist.linear.y = -rand_tmp2*5.0;
          }
          other_pose.pose.position.z = states.pose[robot_idx].position.z+0.4;
          other_pose.twist.linear.z = 0.0;
          other_pose.model_name = spheres_names[current_course];
          other_pose.pose.orientation.x = 0.0; other_pose.pose.orientation.y = 0.0; other_pose.pose.orientation.z = 0.0; other_pose.pose.orientation.w = 1.0;
          model_move_srv.request.model_state = other_pose;
          model_mover.call(model_move_srv);

          spheres_throw_vec[current_course].if_throw=true;
          spheres_throw_vec[current_course].time_counter=real_current_time;
        }
      }
      else if(spheres_throw_vec[current_course].if_throw){
        if ( (real_current_time.clock - spheres_throw_vec[current_course].time_counter.clock).toSec() >= 3.0){
          other_pose.pose.position.x = spheres_poses[current_course].x;
          other_pose.pose.position.y = spheres_poses[current_course].y; 
          other_pose.pose.position.z = spheres_poses[current_course].z;
          other_pose.twist.linear.x = 0.0; other_pose.twist.linear.y = 0.0; other_pose.twist.linear.z = 0.0;
          other_pose.pose.orientation.x = 0.0; other_pose.pose.orientation.y = 0.0; other_pose.pose.orientation.z = 0.0; other_pose.pose.orientation.w = 1.0;
          model_move_srv.request.model_state = other_pose;
          model_mover.call(model_move_srv);          
        }
      }
    }
  }
}

void khnp_comp::move_to_current_course(){
  robot_pose.pose.position.x = courseAB[current_map].courses[current_course].spawn_position.x;
  robot_pose.pose.position.y = courseAB[current_map].courses[current_course].spawn_position.y;
  robot_pose.pose.position.z = courseAB[current_map].courses[current_course].spawn_position.z+0.6;
  robot_pose.pose.orientation.x = 0.0; robot_pose.pose.orientation.y = 0.0; 
  robot_pose.pose.orientation.z = 0.0; robot_pose.pose.orientation.w = 1.0;
  if (courseAB[current_map].courses[current_course].heading_opposite){
    robot_pose.pose.orientation.z = 1.0; robot_pose.pose.orientation.w = 0.0;
  }
  else if (courseAB[current_map].courses[current_course].heading_clock){
    robot_pose.pose.orientation.z = 0.7071068; robot_pose.pose.orientation.w = 0.7071068;
  }
  model_move_srv.request.model_state = robot_pose;
  model_mover.call(model_move_srv);
}

void khnp_comp::move_to_next_course(){
  if (current_course+1 < courseAB[current_map].courses.size()){
    current_course+=1;
    courseAB[current_map].if_passed_map=false;
  }
  else if(current_map+1 < courseAB.size()){
    current_map+=1;
    current_course=0;
    courseAB[current_map-1].if_passed_map=false;
    fixed_course_time=real_current_time;
  }
  else{
    ROS_WARN("No more course to proceed!");
    return;
  }
  robot_pose.pose.position.x = courseAB[current_map].courses[current_course].spawn_position.x;
  robot_pose.pose.position.y = courseAB[current_map].courses[current_course].spawn_position.y;
  robot_pose.pose.position.z = courseAB[current_map].courses[current_course].spawn_position.z+0.6;
  robot_pose.pose.orientation.x = 0.0; robot_pose.pose.orientation.y = 0.0; 
  robot_pose.pose.orientation.z = 0.0; robot_pose.pose.orientation.w = 1.0;
  if (courseAB[current_map].courses[current_course].heading_opposite){
    robot_pose.pose.orientation.z = 1.0; robot_pose.pose.orientation.w = 0.0;
  }
  else if (courseAB[current_map].courses[current_course].heading_clock){
    robot_pose.pose.orientation.z = 0.7071068; robot_pose.pose.orientation.w = 0.7071068;
  }
  model_move_srv.request.model_state = robot_pose;
  model_mover.call(model_move_srv);
}

bool khnp_comp::move_to_next_map(){
  bool update;
  if(current_map+1 < courseAB.size()){
    current_map+=1;
    current_course=0;
    fixed_course_time=real_current_time;
    update=true;
  }
  else{
    ROS_WARN("No more map to proceed!");
    fixed_course_time=real_current_time;
    update=false;
  }
  robot_pose.pose.position.x = courseAB[current_map].courses[current_course].spawn_position.x;
  robot_pose.pose.position.y = courseAB[current_map].courses[current_course].spawn_position.y;
  robot_pose.pose.position.z = courseAB[current_map].courses[current_course].spawn_position.z+0.6;
  robot_pose.pose.orientation.x = 0.0; robot_pose.pose.orientation.y = 0.0; 
  robot_pose.pose.orientation.z = 0.0; robot_pose.pose.orientation.w = 1.0;
  if (courseAB[current_map].courses[current_course].heading_opposite){
    robot_pose.pose.orientation.z = 1.0; robot_pose.pose.orientation.w = 0.0;
  }
  else if (courseAB[current_map].courses[current_course].heading_clock){
    robot_pose.pose.orientation.z = 0.7071068; robot_pose.pose.orientation.w = 0.7071068;
  }
  model_move_srv.request.model_state = robot_pose;
  model_mover.call(model_move_srv);
  return update;
}

void khnp_comp::if_time_over(double time_left){
  if (time_left<=0){
    if (move_to_next_map()){
      courseAB[current_map-1].if_passed_map=false;
      char stg_string[20];
      sprintf(stg_string, "%s- %d", courseAB[current_map].name.c_str(), current_course+1);
      right_text6->setText(QString::fromStdString(stg_string));
    }
    if_felldown_flag=false;
    qt_icon_update(falldown_button, falldown_img);
  }
}
void khnp_comp::if_started_course(geometry_msgs::Pose pose){
  if(within_range(pose.position, courseAB[current_map].courses[current_course].start_position, tolerance)){
    char stg_string[20];
    sprintf(stg_string, "%s- %d", courseAB[current_map].name.c_str(), current_course+1);
    right_text6->setText(QString::fromStdString(stg_string));
  }
}
void khnp_comp::if_passed_course(geometry_msgs::Pose pose){
  if(within_range(pose.position, courseAB[current_map].courses[current_course].finish_position, tolerance)){
    if(courseAB[current_map].name == manipulator_map.name){
      if (within_range(states.pose[cube_idx].position, courseAB[current_map].courses[current_course].finish_position, cube_tolerance)){
        current_score+=courseAB[current_map].courses[current_course].score;
        right_text5->setText(QString::number(current_score,'g',7));
      }
    }
    else{
      current_score+=courseAB[current_map].courses[current_course].score;
      right_text5->setText(QString::number(current_score,'g',7));
    }
    if (current_course+1 < courseAB[current_map].courses.size()){
      current_course+=1;
    }
    else if(current_map+1 < courseAB.size()){
      current_map+=1;
      current_course=0;
      fixed_course_time=real_current_time;
    }
    if (current_course+1 >= courseAB[current_map].courses.size() && current_map+1 >= courseAB.size()){
      finish_result();
    }
  }
}

void khnp_comp::clock_callback(const rosgraph_msgs::Clock::ConstPtr& msg){
  real_current_time = *msg;
  if(!first_clock_in){
    fixed_current_time=real_current_time;
    fixed_course_time=real_current_time;
    first_clock_in=true;
  }
}

void khnp_comp::states_callback(const gazebo_msgs::ModelStates::ConstPtr& msg){
  states = *msg;
  for (int i = 0; i < states.name.size(); ++i)
  {
    if (states.name[i]==robot_name){
      robot_idx=i;
      state_check=true;
    }
    else if (states.name[i]==cube_name){
      cube_idx=i;
    }
  }
}

void khnp_comp::third_cam_callback(const sensor_msgs::CompressedImage::ConstPtr& msg){
  cv_bridge::CvImagePtr third_cam_cv_img_ptr = cv_bridge::toCvCopy(*msg);
  third_cam_cv_img_ptr->image.copyTo(third_cam_cv_img);
  if(!third_cam_check){
    ROS_WARN("%s initialized!", third_cam_name.c_str());
    third_cam_check=true;
  }
}
void khnp_comp::first_cam_callback(const sensor_msgs::CompressedImage::ConstPtr& msg){
  cv_bridge::CvImagePtr first_cam_cv_img_ptr = cv_bridge::toCvCopy(*msg);
  first_cam_cv_img_ptr->image.copyTo(first_cam_cv_img);
  first_cam_check=true;
}

void khnp_comp::falldown_button_callback(){
  if(!if_felldown_flag){
    ROS_WARN("Fell down from user");
    if_felldown_flag=true;
    tmp_felldown_counter=true;
    current_score-=3;
    falldown_score-=3;
    fell_down_time=real_current_time;
    qt_icon_update(falldown_button, fell_img);
    right_text5->setText(QString::number(current_score,'g',7));
    right_text10->setText(QString::number(falldown_score,'g',7));
  }
}
void khnp_comp::pause_button_callback(){
  QImage pause_imgIn;
  if (paused_check){
    unpauser.call(empty_srv);
    qt_icon_update(pause_button, pause_img);
    ROS_WARN("Resumed!!");
    paused_check=false;
  }
  else{
    pauser.call(empty_srv);
    qt_icon_update(pause_button, paused_img);
    ROS_WARN("Paused!!");
    paused_check=true;
  }
}
void khnp_comp::reset_button_callback(){
  resetter.call(empty_srv);
  fixed_current_time=real_current_time; 
  fixed_course_time=real_current_time;
  if_felldown_flag=false;
  qt_icon_update(falldown_button, falldown_img);
  current_map=0;
  current_course=0;
  current_score=0;
  falldown_score=0;
  palette = right_text1->palette();
  palette.setColor(QPalette::Window, palegreen);
  right_text1->setPalette(palette);
  palette = right_text4->palette();
  palette.setColor(QPalette::Window, palegreen);
  right_text4->setPalette(palette);
  right_text1->setText(tr("Current score"));
  right_text4->setText(tr("Current total time"));
  right_text5->setText(QString::number(current_score,'g',7));
  right_text6->setText(QString::fromStdString("Starting again"));
  right_text10->setText(QString::number(falldown_score,'g',7));
  for (int i = 0; i < spheres_throw_vec.size(); ++i){
    spheres_throw_vec[i].if_throw=false;
  }
  if_finished=false;
  ROS_WARN("Resetted!!!");
}
void khnp_comp::skip_button_callback(){
  move_to_next_course();
  if_felldown_flag=false;
  qt_icon_update(falldown_button, falldown_img);
  char stg_string[20];
  sprintf(stg_string, "%s- %d", courseAB[current_map].name.c_str(), current_course+1);
  right_text6->setText(QString::fromStdString(stg_string));

  skip_check=true;
  ROS_WARN("skip");
}

void khnp_comp::finish_result(){
  if(!if_finished){
    right_text1->setText(tr("Total score"));
    palette = right_text1->palette();
    palette.setColor(QPalette::Window, lightred);
    right_text1->setPalette(palette);

    right_text4->setText(tr("Finished time"));
    palette = right_text4->palette();
    palette.setColor(QPalette::Window, lightred);
    right_text4->setPalette(palette);
    
    char result_string[300];
    string _one_passed=(courseAB[0].if_passed_map?"Passed":"Failed (or partially)");
    string _two_passed=(courseAB[1].if_passed_map?"Passed":"Failed (or partially)");
    string _three_passed=(courseAB[2].if_passed_map?"Passed":"Failed (or partially)");
    string _four_passed=(courseAB[3].if_passed_map?"Passed":"Failed (or partially)");
    string _five_passed=(courseAB[4].if_passed_map?"Passed":"Failed (or partially)");
    if (!skip_check){
      sprintf(result_string, "Results:\n\n%s: %s \n%s: %s \n%s: %s \n%s: %s \n%s: %s \nPenalty (falldown): %d", //
        courseAB[0].name.c_str(), _one_passed.c_str(), courseAB[1].name.c_str(), _two_passed.c_str(), //
        courseAB[2].name.c_str(), _three_passed.c_str(), courseAB[3].name.c_str(), _four_passed.c_str(), // 
        courseAB[4].name.c_str(), _five_passed.c_str(), falldown_score);
    }
    else{
      sprintf(result_string, "Results:\nYou skipped at least once, score invalid!\n%s: %s \n%s: %s \n%s: %s \n%s: %s \n%s: %s \nPenalty (falldown): %d", //
        courseAB[0].name.c_str(), _one_passed.c_str(), courseAB[1].name.c_str(), _two_passed.c_str(), //
        courseAB[2].name.c_str(), _three_passed.c_str(), courseAB[3].name.c_str(), _four_passed.c_str(), // 
        courseAB[4].name.c_str(), _five_passed.c_str(), falldown_score);
    }
    QString result = QString::fromStdString(result_string);
    right_result->setText(result);
    right_result->setAlignment(Qt::AlignCenter);
    right_result->setAutoFillBackground(true);
    right_result->setFixedSize(QSize(400,180));
    palette = right_result->palette();
    palette.setColor(QPalette::Window, lightred);
    right_result->setPalette(palette);
    font = right_result->font();
    font.setPointSize(11);
    right_result->setFont(font);
    right_result->setFrameStyle(QFrame::Panel | QFrame::Raised);
    right_result->setLineWidth(3);

    ros::Duration temp = real_current_time.clock-fixed_current_time.clock;
    right_text8->setText(QString::number(temp.sec + temp.nsec*1e-9,'g',7));

    for (int i = 0; i < cubes_names.size(); ++i){
      other_pose.model_name = cubes_names[i];
      other_pose.pose.position.x = cubes_poses[i].x; other_pose.pose.position.y = cubes_poses[i].y; other_pose.pose.position.z = cubes_poses[i].z;
      other_pose.pose.orientation.x = 0.0; other_pose.pose.orientation.y = 0.0; other_pose.pose.orientation.z = 0.0; other_pose.pose.orientation.w = 1.0;
      other_pose.twist.linear.x = 0.0; other_pose.twist.linear.y = 0.0; other_pose.twist.linear.z = 0.0;
      model_move_srv.request.model_state = other_pose;
      model_mover.call(model_move_srv);
    }
    for (int i = 0; i < spheres_names.size(); ++i){
      other_pose.model_name = spheres_names[i];
      other_pose.pose.position.x = spheres_poses[i].x; other_pose.pose.position.y = spheres_poses[i].y; other_pose.pose.position.z = spheres_poses[i].z;
      other_pose.pose.orientation.x = 0.0; other_pose.pose.orientation.y = 0.0; other_pose.pose.orientation.z = 0.0; other_pose.pose.orientation.w = 1.0;
      other_pose.twist.linear.x = 0.0; other_pose.twist.linear.y = 0.0; other_pose.twist.linear.z = 0.0;
      model_move_srv.request.model_state = other_pose;
      model_mover.call(model_move_srv);
      spheres_throw_vec[i].if_throw=false;
    }

    if_finished=true;

    ROS_WARN("Finished!!!");
  }
}


void khnp_comp::QT_initialize(){
  logo_img = cv::imread(path + "/resources/khnp.png");
  pause_img = cv::imread(path + "/resources/pause.png");
  paused_img = cv::imread(path + "/resources/paused.png");
  falldown_img = cv::imread(path + "/resources/falldown.png");
  fell_img = cv::imread(path + "/resources/fell.png");
  reset_img = cv::imread(path + "/resources/reset.png");
  skip_img = cv::imread(path + "/resources/skip.png");
  cv::cvtColor(logo_img, logo_img, CV_BGR2RGB);
  cv::cvtColor(pause_img, pause_img, CV_BGR2RGB);
  cv::cvtColor(paused_img, paused_img, CV_BGR2RGB);
  cv::cvtColor(falldown_img, falldown_img, CV_BGR2RGB);
  cv::cvtColor(fell_img, fell_img, CV_BGR2RGB);
  cv::cvtColor(reset_img, reset_img, CV_BGR2RGB);
  cv::cvtColor(skip_img, skip_img, CV_BGR2RGB);

  left_text1 = new QLabel();
  left_text2 = new QLabel();
  left_3rd_img = new QLabel();
  left_1st_img = new QLabel();
  falldown_button = new QPushButton();
  pause_button = new QPushButton();
  reset_button = new QPushButton();
  skip_button = new QPushButton();
  refresh_button = new QPushButton();
  right_text1 = new QLabel();
  right_text2 = new QLabel();
  right_text3 = new QLabel();
  right_text4 = new QLabel();
  right_text5 = new QLabel();
  right_text6 = new QLabel();
  right_text7 = new QLabel();
  right_text8 = new QLabel();
  right_text9 = new QLabel();
  right_text10 = new QLabel();
  right_creator = new QLabel();
  right_logo = new QLabel();
  right_result = new QLabel();
  right_hbox_btns = new QHBoxLayout();
  right_hbox1 = new QHBoxLayout();
  right_hbox2 = new QHBoxLayout();
  right_hbox3 = new QHBoxLayout();
  right_hbox4 = new QHBoxLayout();
  right_hbox5 = new QHBoxLayout();
  right_hbox6 = new QHBoxLayout();
  right_hbox_result = new QHBoxLayout();
  left_vbox = new QVBoxLayout();
  right_vbox = new QVBoxLayout();
  main_hbox = new QHBoxLayout();



  QImage falldown_imgIn= QImage((uchar*) falldown_img.data, falldown_img.cols, falldown_img.rows, falldown_img.step, QImage::Format_RGB888);
  QPixmap falldown_pixmap = QPixmap::fromImage(falldown_imgIn);
  falldown_button->setIcon(falldown_pixmap);
  falldown_button->setIconSize(QSize(iconsize, iconsize));

  QImage pause_imgIn= QImage((uchar*) pause_img.data, pause_img.cols, pause_img.rows, pause_img.step, QImage::Format_RGB888);
  QPixmap pause_pixmap = QPixmap::fromImage(pause_imgIn);
  pause_button->setIcon(pause_pixmap);
  pause_button->setIconSize(QSize(iconsize, iconsize));

  QImage reset_imgIn= QImage((uchar*) reset_img.data, reset_img.cols, reset_img.rows, reset_img.step, QImage::Format_RGB888);
  QPixmap reset_pixmap = QPixmap::fromImage(reset_imgIn);
  reset_button->setIcon(reset_pixmap);
  reset_button->setIconSize(QSize(iconsize, iconsize));

  QImage skip_imgIn= QImage((uchar*) skip_img.data, skip_img.cols, skip_img.rows, skip_img.step, QImage::Format_RGB888);
  QPixmap skip_pixmap = QPixmap::fromImage(skip_imgIn);
  skip_button->setIcon(skip_pixmap);
  skip_button->setIconSize(QSize(iconsize, iconsize));

  refresh_button->setText("click here only when GUI freezed");
  font = refresh_button->font();
  font.setPointSize(10);
  refresh_button->setFont(font);

  left_text1->setText(tr("3rd person view image"));
  left_text1->setAlignment(Qt::AlignCenter);
  left_text1->setAutoFillBackground(true);
  left_text1->setFixedSize(QSize(img_width,50));
  palette = left_text1->palette();
  palette.setColor(QPalette::Window, cyan);
  left_text1->setPalette(palette);
  font = left_text1->font();
  font.setPointSize(14);
  left_text1->setFont(font);
  left_text1->setFrameStyle(QFrame::Panel | QFrame::Raised);
  left_text1->setLineWidth(3);

  left_text2->setText(tr("1st person view image"));
  left_text2->setAlignment(Qt::AlignCenter);
  left_text2->setAutoFillBackground(true);
  left_text2->setFixedSize(QSize(img_width,50));
  palette = left_text2->palette();
  palette.setColor(QPalette::Window, cyan);
  left_text2->setPalette(palette);
  font = left_text2->font();
  font.setPointSize(14);
  left_text2->setFont(font);
  left_text2->setFrameStyle(QFrame::Panel | QFrame::Raised);
  left_text2->setLineWidth(3);



  right_text1->setText(tr("Current score"));
  right_text1->setAlignment(Qt::AlignCenter);
  right_text1->setAutoFillBackground(true);
  right_text1->setFixedSize(QSize(260,50));
  palette = right_text1->palette();
  palette.setColor(QPalette::Window, palegreen);
  right_text1->setPalette(palette);
  font = right_text1->font();
  font.setPointSize(14);
  right_text1->setFont(font);
  right_text1->setFrameStyle(QFrame::Panel | QFrame::Raised);
  right_text1->setLineWidth(3);

  right_text2->setText(tr("Current stage"));
  right_text2->setAlignment(Qt::AlignCenter);
  right_text2->setAutoFillBackground(true);
  right_text2->setFixedSize(QSize(260,50));
  palette = right_text2->palette();
  palette.setColor(QPalette::Window, palegreen);
  right_text2->setPalette(palette);
  font = right_text2->font();
  font.setPointSize(14);
  right_text2->setFont(font);
  right_text2->setFrameStyle(QFrame::Panel | QFrame::Raised);
  right_text2->setLineWidth(3);

  right_text3->setText(tr("Course left time"));
  right_text3->setAlignment(Qt::AlignCenter);
  right_text3->setAutoFillBackground(true);
  right_text3->setFixedSize(QSize(260,50));
  palette = right_text3->palette();
  palette.setColor(QPalette::Window, palegreen);
  right_text3->setPalette(palette);
  font = right_text3->font();
  font.setPointSize(14);
  right_text3->setFont(font);
  right_text3->setFrameStyle(QFrame::Panel | QFrame::Raised);
  right_text3->setLineWidth(3);

  right_text4->setText(tr("Current total time"));
  right_text4->setAlignment(Qt::AlignCenter);
  right_text4->setAutoFillBackground(true);
  right_text4->setFixedSize(QSize(260,50));
  palette = right_text4->palette();
  palette.setColor(QPalette::Window, palegreen);
  right_text4->setPalette(palette);
  font = right_text4->font();
  font.setPointSize(14);
  right_text4->setFont(font);
  right_text4->setFrameStyle(QFrame::Panel | QFrame::Raised);
  right_text4->setLineWidth(3);

  right_text9->setText(tr("Fall down Penalty"));
  right_text9->setAlignment(Qt::AlignCenter);
  right_text9->setAutoFillBackground(true);
  right_text9->setFixedSize(QSize(260,50));
  palette = right_text9->palette();
  palette.setColor(QPalette::Window, palegreen);
  right_text9->setPalette(palette);
  font = right_text9->font();
  font.setPointSize(14);
  right_text9->setFont(font);
  right_text9->setFrameStyle(QFrame::Panel | QFrame::Raised);
  right_text9->setLineWidth(3);

  right_text5->setText(QString::number(0.0,'g',7));
  right_text5->setAlignment(Qt::AlignCenter);
  right_text5->setAutoFillBackground(true);
  right_text5->setFixedSize(QSize(260,50));
  font = right_text5->font();
  font.setPointSize(13);
  right_text5->setFont(font);

  right_text6->setText(tr("Starting"));
  right_text6->setAlignment(Qt::AlignCenter);
  right_text6->setAutoFillBackground(true);
  right_text6->setFixedSize(QSize(260,50));
  font = right_text6->font();
  font.setPointSize(13);
  right_text6->setFont(font);

  right_text7->setText(QString::number(0.0,'g',7));
  right_text7->setAlignment(Qt::AlignCenter);
  right_text7->setAutoFillBackground(true);
  right_text7->setFixedSize(QSize(260,50));
  font = right_text7->font();
  font.setPointSize(13);
  right_text7->setFont(font);

  right_text8->setText(QString::number(0.0,'g',7));
  right_text8->setAlignment(Qt::AlignCenter);
  right_text8->setAutoFillBackground(true);
  right_text8->setFixedSize(QSize(260,50));
  font = right_text8->font();
  font.setPointSize(13);
  right_text8->setFont(font);

  right_text10->setText(QString::number(0.0,'g',7));
  right_text10->setAlignment(Qt::AlignCenter);
  right_text10->setAutoFillBackground(true);
  right_text10->setFixedSize(QSize(260,50));
  font = right_text10->font();
  font.setPointSize(13);
  right_text10->setFont(font);

  QString creator = "Maintainers (Report any bugs please)\n\nEungchang Mason Lee (email: eungchang_mason@kaist.ac.kr)\nJunho Choi (email: cjh6685kr@kaist.ac.kr)";
  right_creator->setText(creator);
  right_creator->setAlignment(Qt::AlignCenter);
  right_creator->setAutoFillBackground(true);
  right_creator->setFixedSize(QSize(360,90));
  palette = right_creator->palette();
  palette.setColor(QPalette::Window, palepurple);
  right_creator->setPalette(palette);
  font = right_creator->font();
  font.setPointSize(9);
  right_creator->setFont(font);
  right_creator->setFrameStyle(QFrame::Panel | QFrame::Raised);
  right_creator->setLineWidth(3);

  QImage imgIn= QImage((uchar*) logo_img.data, logo_img.cols, logo_img.rows, logo_img.step, QImage::Format_RGB888);
  QPixmap pixmap = QPixmap::fromImage(imgIn);
  right_logo->setPixmap(pixmap);



  left_vbox->addWidget(left_text1);
  left_vbox->addWidget(left_3rd_img);
  left_vbox->addWidget(left_text2);
  left_vbox->addWidget(left_1st_img);

  right_hbox_btns->addWidget(falldown_button);
  right_hbox_btns->addWidget(pause_button);
  right_hbox_btns->addWidget(reset_button);
  right_hbox_btns->addWidget(skip_button);
  right_hbox_btns->setAlignment(Qt::AlignCenter);
  right_hbox1->addWidget(right_text1);
  right_hbox1->addWidget(right_text5);
  right_hbox2->addWidget(right_text2);
  right_hbox2->addWidget(right_text6);
  right_hbox3->addWidget(right_text3);
  right_hbox3->addWidget(right_text7);
  right_hbox4->addWidget(right_text4);
  right_hbox4->addWidget(right_text8);
  right_hbox6->addWidget(right_text9);
  right_hbox6->addWidget(right_text10);
  right_hbox5->addWidget(right_creator);
  right_hbox5->addWidget(right_logo);
  right_hbox5->setAlignment(Qt::AlignCenter);
  right_hbox_result->addWidget(right_result);
  right_hbox_result->setAlignment(Qt::AlignCenter);

  right_vbox->addWidget(refresh_button);
  right_vbox->addLayout(right_hbox_btns);
  right_vbox->addLayout(right_hbox1);
  right_vbox->addLayout(right_hbox2);
  right_vbox->addLayout(right_hbox3);
  right_vbox->addLayout(right_hbox4);
  right_vbox->addLayout(right_hbox6);
  right_vbox->addLayout(right_hbox5);
  right_vbox->setAlignment(Qt::AlignCenter);
  right_vbox->addLayout(right_hbox_result);


  main_hbox->addLayout(left_vbox);
  main_hbox->addLayout(right_vbox);

  setLayout(main_hbox);

  show();
  setWindowTitle(tr("KHNP competition window"));

  connect(falldown_button, &QPushButton::clicked, this, &khnp_comp::falldown_button_callback);
  connect(pause_button, &QPushButton::clicked, this, &khnp_comp::pause_button_callback);
  connect(reset_button, &QPushButton::clicked, this, &khnp_comp::reset_button_callback);
  connect(skip_button, &QPushButton::clicked, this, &khnp_comp::skip_button_callback);
  connect(refresh_button, &QPushButton::clicked, this, &khnp_comp::nothing);

  qt_initialized=true;
}

void khnp_comp::nothing(){
  repaint();
}


#endif
