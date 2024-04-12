// node that controls the joint velocities to keep a joint configuration
// subscribe to joint_poses
// publish to velocity controller
// offers service to activate/deactivate

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <map>
#include <math.h>
#include <cmath>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "custom_interfaces/srv/rob_pose.hpp"
#include <opencv2/core.hpp>

//#include <dmp/Trajectory.hpp>

#include <kinenik/kinenik_ur.h> //also declares JointPos

#include <iostream>
#include <fstream>

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

//#include <iostream>

#include <Eigen/Dense>  // Make sure to install Eigen library
#include <opencv2/core/eigen.hpp>

using namespace Eigen;
using namespace cv;

//#include <eigen3/Eigen/Eigen>
//#include <eigen3/Eigen/Core>
//#include <eigen3/Eigen/Geometry>


//#include <dmp/Trajectory.hpp>

using namespace std::chrono_literals;
//using namespace DmpBbo;
using namespace std::placeholders;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

// Define main program parameters

int control_rate = 500;                                 // frequency of control loop
int control_rate_ms = (float)1/control_rate * 1000;

// ROS node parameters
int rate = 1000;                                        // rate of ROS node
bool activate_stabilization = true;
bool got_initial_config = false;
bool logging = false;                                   // flag to enable logging
int counter = 0;                                        //counting the steps backwards after cube reached the goal

// Joint parameters
int n_joints = 12;                                      // number of UR joints plus gripper joint
int n_passive_joints = 5;                               // number of passive gripper joints
double vel_max = M_PI_2;//1.57;//M_PI;                           // maximum joint velocity
double vel_max_wrists = M_PI;
double f_vel_limit = 1.0;                               // joint velocity as a fraction of the maximum velocity
double joint_vel;
                        // vector of joint velocities
std::vector<std::vector<double>> history;               // vector of historical joint configurations

//double threshold = 0.01;

double scaler_cubic = 50000;
double scaler_square = 10000;

double roll = 0.0;
double pitch = M_PI;
double yaw = 0.0;


bool get_time_stamp = false;
double difference;
rclcpp::Clock clk;
rclcpp::Time stamp;
rclcpp::Time stamp_eef;
double time_pose_received;
bool dmp_mode = false;
//bool set_ur_joint_id = true;

bool linear_traj = false;
KinenikUR myrobot("UR5e");
//bool no_ik = false;
std::vector<JointPos> goal_conf;
std::vector<std::vector<double>> trajectory;
std::vector<double> goal_pose;
int n_steps = 200;
double step_size = 0.001;
int step_counter = 0;

std::string reference_frame = "base";
geometry_msgs::msg::TransformStamped transform_ur_eef;
std::vector<double> initial_transform;
std::vector<double> evolving_transform;

std::vector<double> ik_offsets = {0.025,-0.002,0.055};
//std::vector<double> ik_offsets = {0.0,-0.0,0.0};
std::vector<double> multipliers = {-1,-1,1};
double gripper_pose = 0.6;

std::vector<double> goal;
std::vector<double> goal_steps;

bool go_to_cart_pose = false;
std::vector<double> tcp_pose;
int timeout_ms = 0; //default: robot stays in configuration
int timeout_steps = control_rate/1000*timeout_ms;
int timeout_counter = 0;
bool timeout_triggered = 0;

MatrixXd jacobian(6, 6);
VectorXd cartesianDirection(6);
VectorXd joint_vels(6);
VectorXd cart_vels(6);
VectorXd current_joint_pos(6);
VectorXd current_joint_vel(6);
double manipulability_index;


std::map<std::string, int> map_joints_to_joint_states_id = {
    { "shoulder_pan_joint", 0 },
    { "shoulder_lift_joint", 1 },
    { "elbow_joint", 8 },
    { "wrist_1_joint", 2 },
    { "wrist_2_joint", 3 },
    { "wrist_3_joint", 4 },
    { "gripper_right_driver_joint", 5 },
    { "gripper_left_driver_joint", 7 },
    { "gripper_right_spring_link_joint", 6 },
    { "gripper_left_spring_link_joint", 11 },
    { "gripper_right_follower_joint", 9 },
    { "gripper_left_follower_joint", 10 }
};

std::vector<double> current_joint_configuration(n_joints,0.0);
std::vector<double> desired_joint_configuration(n_joints,0.0);
std::vector<double> requested_joint_configuration(n_joints,0.0);
//std::vector<double> set_joint_velocities(n_joints,0.0);

struct EulerAngles {
    double roll, pitch, yaw;
};
EulerAngles tcp_euler;
// Code from WIkipedia: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
// this implementation assumes normalized quaternion
// converts to Euler angles in 3-2-1 sequence
EulerAngles ToEulerAngles(geometry_msgs::msg::Quaternion q) {
    EulerAngles angles;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = std::sqrt(1 + 2 * (q.w * q.y - q.x * q.z));
    double cosp = std::sqrt(1 - 2 * (q.w * q.y - q.x * q.z));
    angles.pitch = 2 * std::atan2(sinp, cosp) - M_PI / 2;

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);

    return angles;
}

double dev2vel(double deviation){
  double vel = scaler_cubic*pow(deviation,3);//std::exp()
  if(vel > vel_max){vel = vel_max;}
  else if(vel < -vel_max){vel = -vel_max;}
  return vel;
}
double dev2vel2(double deviation){
  double vel;
  if(deviation<0.0){
    vel = -scaler_square*pow(deviation,2);//std::exp()
  }
  else {
    vel = scaler_square*pow(deviation,2);
  }

  if(vel > vel_max){vel = vel_max;}
  else if(vel < -vel_max){vel = -vel_max;}
  return vel;
}
double dev2vel2_wrists(double deviation){
  double vel;
  if(deviation<0.0){
    vel = -scaler_square*pow(deviation,2);//std::exp()
  }
  else {
    vel = scaler_square*pow(deviation,2);
  }

  if(vel > vel_max_wrists){vel = vel_max_wrists;}
  else if(vel < -vel_max_wrists){vel = -vel_max_wrists;}
  return vel;
}

std::vector<JointPos> get_ik(double x,double y,double z,double qr,double qp,double qy){
  std::vector<JointPos> theta_sol;
  myrobot.solveIK(x,y,z,qr,qp,qy,theta_sol);

  return theta_sol;
}

// Function to calculate the Jacobian matrix of UR5 robot
MatrixXd calculateJacobian(const VectorXd& jointAngles) {

    //UR5e from https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics/
    const double d1 = 0.1625;
    const double d2 = 0.0;
    const double d3 = 0.0;
    const double d4 = 0.1333;
    const double d5 = 0.0997;
    const double d6 = 0.0996;

    const double a1 = 0.0;
    const double a2 = -0.425;
    const double a3 = -0.3922;
    const double a4 = 0.0;
    const double a5 = 0.0;
    const double a6 = 0.0;

    const double alpha1 = M_PI_2;
    const double alpha2 = 0;
    const double alpha3 = 0;
    const double alpha4 = M_PI_2;
    const double alpha5 = -M_PI_2;
    const double alpha6 = 0;


    const double q1 = jointAngles(0);
    const double q2 = jointAngles(1);
    const double q3 = jointAngles(2);
    const double q4 = jointAngles(3);
    const double q5 = jointAngles(4);
    const double q6 = jointAngles(5);

    MatrixXd J(6, 6);

    // Calculate individual transformation matrices
    Matrix4d T01, T12, T23, T34, T45, T56, T06;
    T01 << cos(q1), -sin(q1)*cos(alpha1), sin(q1)*sin(alpha1), a1*cos(q1),
           sin(q1), cos(q1)*cos(alpha1), -cos(q1)*sin(alpha1), a1*sin(q1),
           0, sin(alpha1), cos(alpha1), d1,
           0, 0, 0, 1;

    // Transformation matrix from joint 1 to joint 2
    T12 << cos(q2), -sin(q2)*cos(alpha2), sin(q2)*sin(alpha2), a2*cos(q2),
          sin(q2), cos(q2)*cos(alpha2), -cos(q2)*sin(alpha2), a2*sin(q2),
          0, sin(alpha2), cos(alpha2), d2,
          0, 0, 0, 1;

       // Transformation matrix from joint 2 to joint 3
    T23 << cos(q3), -sin(q3)*cos(alpha3), sin(q3)*sin(alpha3), a3*cos(q3),
          sin(q3), cos(q3)*cos(alpha3), -cos(q3)*sin(alpha3), a3*sin(q3),
          0, sin(alpha3), cos(alpha3), d3,
          0, 0, 0, 1;

     // Transformation matrix from joint 3 to joint 4
     T34 << cos(q4), -sin(q4)*cos(alpha4), sin(q4)*sin(alpha4), a4*cos(q4),
          sin(q4), cos(q4)*cos(alpha4), -cos(q4)*sin(alpha4), a4*sin(q4),
          0, sin(alpha4), cos(alpha4), d4,
          0, 0, 0, 1;

     // Transformation matrix from joint 4 to joint 5
     T45 << cos(q5), -sin(q5)*cos(alpha5), sin(q5)*sin(alpha5), a5*cos(q5),
          sin(q5), cos(q5)*cos(alpha5), -cos(q5)*sin(alpha5), a5*sin(q5),
          0, sin(alpha5), cos(alpha5), d5,
          0, 0, 0, 1;

     // Transformation matrix from joint 5 to joint 6
     T56 << cos(q6), -sin(q6)*cos(alpha6), sin(q6)*sin(alpha6), a6*cos(q6),
          sin(q6), cos(q6)*cos(alpha6), -cos(q6)*sin(alpha6), a6*sin(q6),
          0, sin(alpha6), cos(alpha6), d6,
          0, 0, 0, 1;

    T06 = T01 * T12 * T23 * T34 * T45 * T56;
    //std::cout << "***\nTranformation matrix T01 for (" << q1 << "," << q2 << "," << q3 << "," << q4 << "," << q5 << "," << q6 << ")\n" << T01 <<"\n";
    //std::cout << "Tranformation matrix T02 for (" << q1 << "," << q2 << "," << q3 << "," << q4 << "," << q5 << "," << q6 << ")\n" << T01*T12 <<"\n";
    //std::cout << "Tranformation matrix T03 for (" << q1 << "," << q2 << "," << q3 << "," << q4 << "," << q5 << "," << q6 << ")\n" << T01*T12*T23 <<"\n";
    //std::cout << "Tranformation matrix T04 for (" << q1 << "," << q2 << "," << q3 << "," << q4 << "," << q5 << "," << q6 << ")\n" << T01*T12*T23*T34 <<"\n";
    //std::cout << "Tranformation matrix T05 for (" << q1 << "," << q2 << "," << q3 << "," << q4 << "," << q5 << "," << q6 << ")\n" << T01*T12*T23*T34*T45 <<"\n";
    //std::cout << "Tranformation matrix T06 for (" << q1 << "," << q2 << "," << q3 << "," << q4 << "," << q5 << "," << q6 << ")\n" << T06 <<"\n***\n";

    //std::cout << "Tranformation matrix T06 for (" << q1 << "," << q2 << "," << q3 << "," << q4 << "," << q5 << "," << q6 << ")\n" << T06 <<"\n***\n";

    J(0,0)=-sin(q1)*(a3*cos(q2 + q3) + a2*cos(q2));
    J(0,1)=-cos(q1)*(a3*sin(q2 + q3) + a2*sin(q2) - d5*(cos(q2 + q3)*cos(q4) - sin(q2 + q3)*sin(q4)) - d6*sin(q5)*(cos(q2 + q3)*sin(q4) + sin(q2 + q3)*cos(q4)));
    J(0,2)=cos(q1)*(d5*cos(q2 + q3 + q4) - a3*sin(q2 + q3) + d6*sin(q2 + q3 + q4)*sin(q5));
    J(0,3)=cos(q1)*(d5*cos(q2 + q3 + q4) + d6*sin(q2 + q3 + q4)*sin(q5));
    J(0,4)=d6*cos(q1)*cos(q2)*cos(q5)*sin(q3)*sin(q4) - d6*sin(q1)*sin(q5) + d6*cos(q1)*cos(q3)*cos(q5)*sin(q2)*sin(q4) + d6*cos(q1)*cos(q4)*cos(q5)*sin(q2)*sin(q3) - d6*cos(q1)*cos(q2)*cos(q3)*cos(q4)*cos(q5);
    J(0,5)=0;
    J(1,0)=cos(q1)*(a3*cos(q2 + q3) + a2*cos(q2));
    J(1,1)=-sin(q1)*(a3*sin(q2 + q3) + a2*sin(q2) - d5*(cos(q2 + q3)*cos(q4) - sin(q2 + q3)*sin(q4)) - d6*sin(q5)*(cos(q2 + q3)*sin(q4) + sin(q2 + q3)*cos(q4)));
    J(1,2)=sin(q1)*(d5*cos(q2 + q3 + q4) - a3*sin(q2 + q3) + d6*sin(q2 + q3 + q4)*sin(q5));
    J(1,3)=sin(q1)*(d5*cos(q2 + q3 + q4) + d6*sin(q2 + q3 + q4)*sin(q5));
    J(1,4)=d6*cos(q1)*sin(q5) + d6*cos(q2)*cos(q5)*sin(q1)*sin(q3)*sin(q4) + d6*cos(q3)*cos(q5)*sin(q1)*sin(q2)*sin(q4) + d6*cos(q4)*cos(q5)*sin(q1)*sin(q2)*sin(q3) - d6*cos(q2)*cos(q3)*cos(q4)*cos(q5)*sin(q1);
    J(1,5)=0;
    J(2,0)=0;
    J(2,1)=a3*cos(q2 + q3) - (d6*sin(q2 + q3 + q4 + q5))/2 + a2*cos(q2) + (d6*sin(q2 + q3 + q4 - q5))/2 + d5*sin(q2 + q3 + q4);
    J(2,2)=a3*cos(q2 + q3) - (d6*sin(q2 + q3 + q4 + q5))/2 + (d6*sin(q2 + q3 + q4 - q5))/2 + d5*sin(q2 + q3 + q4);
    J(2,3)=(d6*sin(q2 + q3 + q4 - q5))/2 - (d6*sin(q2 + q3 + q4 + q5))/2 + d5*sin(q2 + q3 + q4);
    J(2,4)=-d6*(sin(q2 + q3 + q4 + q5)/2 + sin(q2 + q3 + q4 - q5)/2);
    J(2,5)=0;
    J(3,0)=0;
    J(3,1)=sin(q1);
    J(3,2)=sin(q1);
    J(3,3)=sin(q1);
    J(3,4)=cos(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)) + sin(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3));
    J(3,5)=cos(q5)*sin(q1) - sin(q5)*(cos(q4)*(cos(q1)*cos(q2)*cos(q3) - cos(q1)*sin(q2)*sin(q3)) - sin(q4)*(cos(q1)*cos(q2)*sin(q3) + cos(q1)*cos(q3)*sin(q2)));
    J(4,0)=0;
    J(4,1)=-cos(q1);
    J(4,2)=-cos(q1);
    J(4,3)=-cos(q1);
    J(4,4)=cos(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2)) - sin(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1));
    J(4,5)=sin(q5)*(cos(q4)*(sin(q1)*sin(q2)*sin(q3) - cos(q2)*cos(q3)*sin(q1)) + sin(q4)*(cos(q2)*sin(q1)*sin(q3) + cos(q3)*sin(q1)*sin(q2))) - cos(q1)*cos(q5);
    J(5,0)=1;
    J(5,1)=0;
    J(5,2)=0;
    J(5,3)=0;
    J(5,4)=sin(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) - cos(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3));
    J(5,5)=-sin(q5)*(cos(q4)*(cos(q2)*sin(q3) + cos(q3)*sin(q2)) + sin(q4)*(cos(q2)*cos(q3) - sin(q2)*sin(q3)));

    //std::cout << "Jacobian matrix now:\n" << J <<"\n***\n";
    return J;
}

VectorXd calculateJointVelocities(const MatrixXd& jacobi, const VectorXd& cartesianDirection)
{
    // Check dimensions
    if (jacobi.rows() != 6 || jacobi.cols() != 6 || cartesianDirection.size() != 6)
    {
        throw std::invalid_argument("Invalid dimensions for Jacobian or cartesianDirection");
    }

    // Calculate joint velocities
    std::cout << "Error might be coming...\n";// << jacobi << "\n";
    //stamp = clk.now();
    CompleteOrthogonalDecomposition<MatrixXd> cod(jacobi);
    MatrixXd pseudo_inv = cod.pseudoInverse();//jacobi.completeOrthogonalDecomposition().pseudoInverse();
    //std::cout << "Computation time of pseudo_inverse: " << clk.now().nanoseconds()-stamp.nanoseconds()<< "\n";
    VectorXd jointVelocities = pseudo_inv * cartesianDirection;
    //VectorXd jointVelocities = jacobian.inverse() * cartesianDirection;

    return jointVelocities;
}

VectorXd calculateJointVelocities_cv(const MatrixXd& jacobian, const VectorXd& cartesianDirection)
{
    Mat jacobi;
    eigen2cv(jacobian, jacobi);

    // Check dimensions
    if (jacobian.rows() != 6 || jacobian.cols() != 6 || cartesianDirection.size() != 6)
    {
        throw std::invalid_argument("Invalid dimensions for Jacobian or cartesianDirection");
    }

    // Calculate joint velocities
    std::cout << "Error should not be coming...\n";// << jacobi << "\n";
    //stamp = clk.now();
    //CompleteOrthogonalDecomposition<MatrixXd> cod(jacobi);
    Mat pseudoInv;
    invert(jacobi, pseudoInv, DECOMP_SVD);
    //MatrixXd pseudo_inv = cod.pseudoInverse();//jacobi.completeOrthogonalDecomposition().pseudoInverse();
    //std::cout << "Computation time of pseudo_inverse: " << clk.now().nanoseconds()-stamp.nanoseconds()<< "\n";
    MatrixXd pseudo_inv;
    cv2eigen(pseudoInv, pseudo_inv);

    VectorXd jointVelocities = pseudo_inv * cartesianDirection;
    //VectorXd jointVelocities = jacobian.inverse() * cartesianDirection;

    return jointVelocities;
}


class CartesianVelocityPublisher : public rclcpp::Node
{
  public:
    CartesianVelocityPublisher() : Node("cartesian_velocity_publisher")
    {
      service_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      timer_cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

      subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rate, std::bind(&CartesianVelocityPublisher::topic_callback, this, _1));

      publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/forward_velocity_controller/commands", control_rate);
      timer_ = this->create_wall_timer(
      std::chrono::milliseconds(control_rate_ms), std::bind(&CartesianVelocityPublisher::timer_callback, this),timer_cb_group_);

      //service_ = this->create_service<custom_interfaces::srv::RobPose>("/velocity_controller/set_desired_rob_pose", std::bind(&CartesianVelocityPublisher::set_desired_rob_pose, this, _1, _2),rmw_qos_profile_services_default,service_cb_group_);

      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);



    }


  private:
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    rclcpp::Service<custom_interfaces::srv::RobPose>::SharedPtr service_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_ {nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    rclcpp::CallbackGroup::SharedPtr service_cb_group_;
    rclcpp::CallbackGroup::SharedPtr timer_cb_group_;



    void topic_callback(const sensor_msgs::msg::JointState & msg) const
    {


      current_joint_configuration[0] = msg.position[map_joints_to_joint_states_id["shoulder_pan_joint"]];
      current_joint_configuration[1] = msg.position[map_joints_to_joint_states_id["shoulder_lift_joint"]];
      current_joint_configuration[2] = msg.position[map_joints_to_joint_states_id["elbow_joint"]];
      current_joint_configuration[3] = msg.position[map_joints_to_joint_states_id["wrist_1_joint"]];
      current_joint_configuration[4] = msg.position[map_joints_to_joint_states_id["wrist_2_joint"]];
      current_joint_configuration[5] = msg.position[map_joints_to_joint_states_id["wrist_3_joint"]];
      current_joint_configuration[6] = msg.position[map_joints_to_joint_states_id["gripper_right_driver_joint"]];
      current_joint_configuration[7] = msg.position[map_joints_to_joint_states_id["gripper_left_driver_joint"]];
      current_joint_configuration[8] = msg.position[map_joints_to_joint_states_id["gripper_right_spring_link_joint"]];
      current_joint_configuration[9] = msg.position[map_joints_to_joint_states_id["gripper_left_spring_link_joint"]];
      current_joint_configuration[10] = msg.position[map_joints_to_joint_states_id["gripper_right_follower_joint"]];
      current_joint_configuration[11] = msg.position[map_joints_to_joint_states_id["gripper_left_follower_joint"]];

      current_joint_vel[0] = msg.velocity[map_joints_to_joint_states_id["shoulder_pan_joint"]];
      current_joint_vel[1] = msg.velocity[map_joints_to_joint_states_id["shoulder_lift_joint"]];
      current_joint_vel[2] = msg.velocity[map_joints_to_joint_states_id["elbow_joint"]];
      current_joint_vel[3] = msg.velocity[map_joints_to_joint_states_id["wrist_1_joint"]];
      current_joint_vel[4] = msg.velocity[map_joints_to_joint_states_id["wrist_2_joint"]];
      current_joint_vel[5] = msg.velocity[map_joints_to_joint_states_id["wrist_3_joint"]];

      current_joint_pos[0] = current_joint_configuration[0];
      current_joint_pos[1] = current_joint_configuration[1];
      current_joint_pos[2] = current_joint_configuration[2];
      current_joint_pos[3] = current_joint_configuration[3];
      current_joint_pos[4] = current_joint_configuration[4];
      current_joint_pos[5] = current_joint_configuration[5];

      //jacobian = calculateJacobian(current_joint_pos);
      manipulability_index = jacobian.determinant();
      //cart_vels = jacobian * current_joint_vel;

      //RCLCPP_INFO(this->get_logger(), "##Cartesian velocities from Jacobian now\n: %f, %f, %f, %f, %f, %f\n", cart_vels[0],cart_vels[1],cart_vels[2],cart_vels[3],cart_vels[4],cart_vels[5]);
      //RCLCPP_INFO(this->get_logger(), "**Joint velocities: %f, %f, %f, %f, %f, %f", current_joint_vel[0],current_joint_vel[1],current_joint_vel[2],current_joint_vel[3],current_joint_vel[4],current_joint_vel[5]);

/*      try {
        transform_ur_eef = tf_buffer_->lookupTransform(
          reference_frame, "wrist_3_link",
          tf2::TimePointZero);
        //time_eef = clk.now();
      } catch (const tf2::TransformException & ex) {
        RCLCPP_INFO(
          this->get_logger(), "Could not transform %s to %s: %s",
          reference_frame.c_str(), "wrist_3_link", ex.what());
        return;
      }
      stamp_eef = clk.now();

      tcp_euler = ToEulerAngles(transform_ur_eef.transform.rotation);
      tcp_pose = {(transform_ur_eef.transform.translation.x*multipliers[0])-ik_offsets[0],(transform_ur_eef.transform.translation.y*multipliers[1])-ik_offsets[1],(transform_ur_eef.transform.translation.z*multipliers[2])-ik_offsets[2],tcp_euler.roll,tcp_euler.pitch,tcp_euler.yaw,gripper_pose};
*/
      //get_eef_y = t.transform.translation.y;
      //RCLCPP_INFO(this->get_logger(), "Current tcp pose: '(%f, %f, %f, %f, %f, %f)'", tcp_pose[0], tcp_pose[1], tcp_pose[2], tcp_pose[3], tcp_pose[4], tcp_pose[5]);
      //RCLCPP_INFO(this->get_logger(), "Current original tcp pose: '(%f, %f, %f, %f, %f, %f)'\n***\n", transform_ur_eef.transform.translation.x, transform_ur_eef.transform.translation.y, transform_ur_eef.transform.translation.z, tcp_pose[3], tcp_pose[4], tcp_pose[5]);
      //RCLCPP_INFO(this->get_logger(), "Current Z: %f", tcp_pose[2]);
      //RCLCPP_INFO(this->get_logger(), "Message size: %ld", msg.position.size());

    }

    /*void set_desired_rob_pose(std::shared_ptr<custom_interfaces::srv::RobPose::Request>  req,
             std::shared_ptr<custom_interfaces::srv::RobPose::Response> res)
    {
      RCLCPP_INFO(this->get_logger(), "1 a");
      go_to_cart_pose = req->cart_pose;
      timeout_ms = req->timeout_ms;
      timeout_steps = (float)control_rate/1000*timeout_ms;
      //RCLCPP_INFO(this->get_logger(), "Timeout steps: %d", timeout_steps);
      // Check dimensions
      if (req->goal_dir.size() != 7){
        throw std::invalid_argument("Invalid dimensions (other than 7) for request of goal_dir / cartesianDirection & gripper pose");
      }
      cartesianDirection[0] = req->goal_dir[0];
      cartesianDirection[1] = req->goal_dir[1];
      cartesianDirection[2] = req->goal_dir[2];
      cartesianDirection[3] = req->goal_dir[3];
      cartesianDirection[4] = req->goal_dir[4];
      cartesianDirection[5] = req->goal_dir[5];
      gripper_pose = req->goal_dir[6];

      if(go_to_cart_pose){
        //don't go into the visual servoing loop
        //timeout_counter = timeout_steps+1;
        goal_conf = get_ik(cartesianDirection[0],cartesianDirection[1],cartesianDirection[2],cartesianDirection[3],cartesianDirection[4],cartesianDirection[5]);
        if(goal_conf.size()==0){
          RCLCPP_ERROR(this->get_logger(), "no IK found, staying at this configuration...");
        }

        for(int i = 0; i < n_joints; i++)
        {
          if(i<6){
            requested_joint_configuration[i] = goal_conf[0][i];
          }
          else if(i>=10){
            requested_joint_configuration[i] = -gripper_pose;
          }
          else{
            requested_joint_configuration[i] = gripper_pose;
          }
        }

        //go_to_cart_pose = false;
        RCLCPP_INFO(this->get_logger(), "Requested configuration from Cartesian pose: '(%f, %f, %f, %f, %f, %f, %f)'", goal_conf[0][0], goal_conf[0][1], goal_conf[0][2], goal_conf[0][3], goal_conf[0][4], goal_conf[0][5], gripper_pose);
      }
      //reset counter

      res->time_since_last_req_ms = control_rate_ms*timeout_counter;
      res->time_stamp = stamp_eef.nanoseconds();
      res->eef_x = transform_ur_eef.transform.translation.x;
      res->eef_y = transform_ur_eef.transform.translation.y;
      res->eef_yaw = tcp_euler.yaw;
      res->manipulability_index = manipulability_index;
      res->timeout_triggered = timeout_triggered;
      res->success = 1;
      timeout_counter = 0;
      RCLCPP_INFO(this->get_logger(), "1 e");
      //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "new desired configuration is set to (%f, %f, %f, %f, %f, %f, %f)", goal_conf[0][0], goal_conf[0][1], goal_conf[0][2], goal_conf[0][3], goal_conf[0][4], goal_conf[0][5], req->goal_conf[6]);

    }*/

    void timer_callback()
    {

          RCLCPP_INFO(this->get_logger(), "2ii1 0");
          jacobian = calculateJacobian(current_joint_pos);
          RCLCPP_INFO(this->get_logger(), "2ii1 1");
          //joint_vels = calculateJointVelocities(jacobian,cartesianDirection);
          joint_vels = calculateJointVelocities_cv(jacobian,cartesianDirection);
          RCLCPP_INFO(this->get_logger(), "2ii1 2");


    //publisher_->publish(message);
    RCLCPP_INFO(this->get_logger(), "2 e");
  }

};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto cartesian_velocity_node = std::make_shared<CartesianVelocityPublisher>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(cartesian_velocity_node);

  executor.spin();

  rclcpp::shutdown();
  return 0;
}
