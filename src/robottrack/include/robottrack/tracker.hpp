#ifndef _TRACKER_HPP_
#define _TRACKER_HPP_

// Eigen
#include <Eigen/Eigen>

// ROS
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>

// STD
#include <memory>
#include <string>

#include "ExtendedKalmanFliter.hpp"
#include "robotinterfaces/Armors.h"
#include "robotinterfaces/Armor.h"
#include "robotinterfaces/Target.h"

enum class ArmorsNum { NORMAL_4 = 4, BALANCE_2 = 2, OUTPOST_3 = 3 };

class Tracker
{
public:
  Tracker(double max_match_distance, double max_match_yaw_diff);

  using Armors = robotinterfaces::Armors;
  using Armor = robotinterfaces::Armor;

  void init(const Armors::Ptr & armors_msg);

  void update(const Armors::Ptr & armors_msg);

  ExtendedKalmanFilter ekf;

  int tracking_thres;
  int lost_thres;

  enum State {
    LOST,
    DETECTING,
    TRACKING,
    TEMP_LOST,
  } tracker_state;

  std::string tracked_id;
  Armor tracked_armor;
  ArmorsNum tracked_armors_num;

  double info_position_diff;
  double info_yaw_diff;

  Eigen::VectorXd measurement;

  Eigen::VectorXd target_state;

  // To store another pair of armors message
  double dz, another_r;

private:
  void initEKF(const Armor & a);

  void updateArmorsNum(const Armor & a);

  void handleArmorJump(const Armor & a);

  double orientationToYaw(const geometry_msgs::Quaternion & q);

  Eigen::Vector3d getArmorPositionFromState(const Eigen::VectorXd & x);

  double max_match_distance_;
  double max_match_yaw_diff_;

  int detect_count_;
  int lost_count_;

  double last_yaw_;
};

#endif  // ARMOR_PROCESSOR__TRACKER_HPP_
