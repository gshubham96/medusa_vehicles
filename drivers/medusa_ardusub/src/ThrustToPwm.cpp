#include "ThrustToPwm.h"

ThrustToPwm::ThrustToPwm(ros::NodeHandle &nh) {
  initializeSubscribers(nh);
  initializePublishers(nh);
  loadParams(nh);
}

void ThrustToPwm::initializeSubscribers(ros::NodeHandle &nh) {
  // subscribe to thrust
  ft_sub_ = nh.subscribe(MedusaGimmicks::getParameters<std::string>(nh, 
    "topics/subscribers/thrust_body_request", 
    "/thrust_body_request"),
    10, &ThrustToPwm::thrusterAllocation, this);
}

void ThrustToPwm::initializePublishers(ros::NodeHandle &nh) {
  pwm_pub_ = nh.advertise<mavros_msgs::OverrideRCIn>(
    MedusaGimmicks::getParameters<std::string>(nh, 
    "topics/publishers/thrusters",
    "/thrusters/RPM_Command"), 1);
}

void ThrustToPwm::loadParams(ros::NodeHandle &nh) {
  max_thrust_norm_ = nh.param("thrusters/max_thrust_norm", 22);
  min_thrust_norm_ = nh.param("thrusters/min_thrust_norm", -22);
  readTAM(nh);
  readCT(nh);
}

void ThrustToPwm::saturateVector(Eigen::VectorXd &thr_thrust) {
  int max_ind, min_ind;
  float maximum = thr_thrust.maxCoeff(&max_ind);
  float minimum = thr_thrust.minCoeff(&min_ind);

  // normalize vector in the case max_value is higher than max_value
  // min and max are independent
  float normalize = 1;

  normalize = std::max(fabs(minimum / min_thrust_norm_), normalize);
  normalize = std::max(fabs(maximum / max_thrust_norm_), normalize);

  for (int i = 0; i < thr_thrust.size(); i++) {
    thr_thrust[i] /= normalize;
  }
}

void ThrustToPwm::readTAM(ros::NodeHandle &nh) {
  std::vector<double> allocation_vector = MedusaGimmicks::getParameters<std::vector<double>>(nh, "thrusters/allocation_vector");

  // TODO: FIX create Thruster Allocation Matrix (B) with shape [num of forces][num of thrusters]
  b_inv_ = Eigen::ArrayXd::Ones(6);

  // Get the three ⊥r distances for roll, pitch and yaw
  for (int i = 0; i < 3; ++i) {
    b_inv_(3+i) = 1/allocation_vector[i] ;
  }
}

void ThrustToPwm::readCT(ros::NodeHandle &nh) {
  ctf_ = MedusaGimmicks::getParameters<std::vector<double>>(nh, "thrusters/ctf");
  ctb_ = MedusaGimmicks::getParameters<std::vector<double>>(nh, "thrusters/ctb");
}

void ThrustToPwm::thrusterAllocation(const auv_msgs::BodyForceRequest &msg) {
  Eigen::ArrayXd ft_req(6);
  ft_req << float(msg.wrench.force.x), 
            float(msg.wrench.force.y),
            float(msg.wrench.force.z), 
            float(msg.wrench.torque.x),
            float(msg.wrench.torque.y), 
            float(msg.wrench.torque.z);
  // std::cout << "force in  : " << ft_req << std::endl;

  // Compute the force necessary for each thruster
  Eigen::VectorXd thrust = b_inv_*ft_req;
  // std::cout << "thrust out: " << thrust << std::endl;

  // Saturate thrust
  saturateVector(thrust);
  // std::cout << "normalized force  : " << ft_req << std::endl;

  // Convert from force to % of RPM (because of the drivers - legacy)
  // msg has no header
  mavros_msgs::OverrideRCIn pwm;
  // set all unused channels to zero
  for (int i = 6; i < 18; i++) {
    pwm.channels[i] = 1500;
  }

  // set the six dof channels to required PWM
  std::vector<double> channels(6,1500);
  for (int i = 0; i < thrust.size(); ++i) {
    if (thrust[i] > 0) {
      channels[i] = ctf_[0]*thrust[i]*thrust[i] + ctf_[1]*thrust[i] + ctf_[2]  ;
    } else if (thrust[i] < 0) {
      channels[i] = ctb_[0]*thrust[i]*thrust[i] + ctb_[1]*thrust[i] + ctb_[2]  ;
    }
  }

  // ardusub has some weird input order
  // number (#)   = 0    , 1   , 2    , 3   , 4,     5
  // channels     = surge, sway, heave, roll, pitch, yaw
  // pwm.channels = pitch, roll, heave, yaw, surge, sway
  pwm.channels[0] = channels[4];
  pwm.channels[1] = channels[3];
  pwm.channels[2] = channels[2];
  pwm.channels[3] = channels[5];
  pwm.channels[4] = channels[0];
  pwm.channels[5] = channels[2];

  // std::cout << "ctf: " << ctf_[0] << " : " << ctf_[1] << " : " << ctf_[2] << std::endl;
  // std::cout << "force: " << thrust[0] << ", pwm: " << pwm.channels[0] << std::endl;

  // publish pwm values
  pwm_pub_.publish(pwm);
}

/* Create the Static Thruster Allocation Object */
int main(int argc, char **argv) {
  ros::init(argc, argv, "Static Thruster Allocation");
  ros::NodeHandle nh("~");
  ThrustToPwm thr(nh);
  ros::spin();
}