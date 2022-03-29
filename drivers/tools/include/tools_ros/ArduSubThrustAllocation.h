#include <Eigen/Dense>
#include <auv_msgs/BodyForceRequest.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <medusa_gimmicks_library/MedusaGimmicks.h>
#include <ros/ros.h>

/**
 * @brief  ROS implementation of the thrust allocation. Receives forces applied to the vehicle and calculates the desired forces to each thruster based on the pseudo inverse of the thrust allocation matrix
 */
class ArduSubThrustAllocation {
public:
  /**
   * @brief  Thrust Allocation class constructor
   *
   * @param nh  ROS nodehandle to publish, subscribe and read relevant
   * parameters
   */
  ArduSubThrustAllocation(ros::NodeHandle &nh);

  /**
   * @brief  Function to initialize subscribers 
   *
   * @param nh  ROS nodehandle to publish, subscribe and read relevant
   */
  void initializeSubscribers(ros::NodeHandle &nh);

  /**
   * @brief  Function to initialize publishers 
   *
   * @param nh  ROS nodehandle to publish, subscribe and read relevant
   */
  void initializePublishers(ros::NodeHandle &nh);

  /**
   * @brief  Function to read parameters. Reads the thruster allocation matrix and computes its pseudo inverse 
   *
   * @param nh  ROS nodehandle to publish, subscribe and read relevant
   */
  void loadParams(ros::NodeHandle &nh);

  /**
   * @brief  Given a force vector for each thruster, saturate the norm of the vector based on the maximum force of the thruster
   */
  void saturateVector(Eigen::VectorXd &thr_thrust);
 
  /**
   * @brief  Function to read a thruster allocation matrix and compute its pseudo inverse
   *
   * @param nh
   */
  void readTAM(ros::NodeHandle &nh);

  /**
   * @brief  Function to read the ct parameters (conversion from thrust to RPM and vice versa)
   *
   * @param nh
   */
  void readCT(ros::NodeHandle &nh);

  /**
   * @brief  Callback function of the topic with the tau (force request)
   *
   * @param msg  Variable containing the force request
   */
  void thrusterAllocation(const auv_msgs::BodyForceRequest &msg);

private:
  ros::Subscriber ft_sub_;

  // thruster publishers
  ros::Publisher pwm_pub_;

  // max and min parameters
  float max_thrust_norm_;
  float min_thrust_norm_;

  Eigen::ArrayXd b_inv_;
  std::vector<double> ctf_, ctb_;
};
