#include "TrackerNode.hpp"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "tracker_node");
    ros::NodeHandle nh;
    TrackerNode tracker_node(nh);
    
    ros::spin();
    return 0;
}

TrackerNode::TrackerNode(ros::NodeHandle nh):
    tf_listener(tf_buffer)
{
    target_frame = "odom";
    max_armor_distance = nh.param("max_armor_distance", 10.0);


    max_match_distance = nh.param("max_match_distance", 0.15);
    max_match_yaw_diff = nh.param("max_match_yaw_diff", 1.0);

    tracker = std::make_unique<Tracker>(max_match_distance, max_match_yaw_diff);
    tracker->tracking_thres = nh.param("tracking_thres", 5);
    lost_time_thres = nh.param("lost_time_thres", 0.3);

  // EKF
  // xa = x_armor, xc = x_robot_center
  // state: xc, v_xc, yc, v_yc, za, v_za, yaw, v_yaw, r
  // measurement: xa, ya, za, yaw
  // f - Process function
  auto f = [this](const Eigen::VectorXd & x) {
    Eigen::VectorXd x_new = x;
    x_new(0) += x(1) * dt;
    x_new(2) += x(3) * dt;
    x_new(4) += x(5) * dt;
    x_new(6) += x(7) * dt;
    return x_new;
  };
  // J_f - Jacobian of process function
  auto j_f = [this](const Eigen::VectorXd &) {
    Eigen::MatrixXd f(9, 9);
    // clang-format off
    f <<  1,   dt, 0,   0,   0,   0,   0,   0,   0,
          0,   1,   0,   0,   0,   0,   0,   0,   0,
          0,   0,   1,   dt, 0,   0,   0,   0,   0, 
          0,   0,   0,   1,   0,   0,   0,   0,   0,
          0,   0,   0,   0,   1,   dt, 0,   0,   0,
          0,   0,   0,   0,   0,   1,   0,   0,   0,
          0,   0,   0,   0,   0,   0,   1,   dt, 0,
          0,   0,   0,   0,   0,   0,   0,   1,   0,
          0,   0,   0,   0,   0,   0,   0,   0,   1;
    // clang-format on
    return f;
  };
  // h - Observation function
  auto h = [](const Eigen::VectorXd & x) {
    Eigen::VectorXd z(4);
    double xc = x(0), yc = x(2), yaw = x(6), r = x(8);
    z(0) = xc - r * cos(yaw);  // xa
    z(1) = yc - r * sin(yaw);  // ya
    z(2) = x(4);               // za
    z(3) = x(6);               // yaw
    return z;
  };
  // J_h - Jacobian of observation function
  auto j_h = [](const Eigen::VectorXd & x) {
    Eigen::MatrixXd h(4, 9);
    double yaw = x(6), r = x(8);
    // clang-format off
    //    xc   v_xc yc   v_yc za   v_za yaw         v_yaw r
    h <<  1,   0,   0,   0,   0,   0,   r*sin(yaw), 0,   -cos(yaw),
          0,   0,   1,   0,   0,   0,   -r*cos(yaw),0,   -sin(yaw),
          0,   0,   0,   0,   1,   0,   0,          0,   0,
          0,   0,   0,   0,   0,   0,   1,          0,   0;
    // clang-format on
    return h;
  };
  // update_Q - process noise covariance matrix
  s2qxyz = nh.param("sigma2_q_xyz", 20.0);
  s2qyaw = nh.param("sigma2_q_yaw", 100.0);
  s2qr = nh.param("sigma2_q_r", 800.0);
  auto u_q = [this]() {
    Eigen::MatrixXd q(9, 9);
    double t = dt, x = s2qxyz, y = s2qyaw, r = s2qr;
    double q_x_x = pow(t, 4) / 4 * x, q_x_vx = pow(t, 3) / 2 * x, q_vx_vx = pow(t, 2) * x;
    double q_y_y = pow(t, 4) / 4 * y, q_y_vy = pow(t, 3) / 2 * x, q_vy_vy = pow(t, 2) * y;
    double q_r = pow(t, 4) / 4 * r;
    // clang-format off
    //    xc      v_xc    yc      v_yc    za      v_za    yaw     v_yaw   r
    q <<  q_x_x,  q_x_vx, 0,      0,      0,      0,      0,      0,      0,
          q_x_vx, q_vx_vx,0,      0,      0,      0,      0,      0,      0,
          0,      0,      q_x_x,  q_x_vx, 0,      0,      0,      0,      0,
          0,      0,      q_x_vx, q_vx_vx,0,      0,      0,      0,      0,
          0,      0,      0,      0,      q_x_x,  q_x_vx, 0,      0,      0,
          0,      0,      0,      0,      q_x_vx, q_vx_vx,0,      0,      0,
          0,      0,      0,      0,      0,      0,      q_y_y,  q_y_vy, 0,
          0,      0,      0,      0,      0,      0,      q_y_vy, q_vy_vy,0,
          0,      0,      0,      0,      0,      0,      0,      0,      q_r;
    // clang-format on
    return q;
  };
  // update_R - measurement noise covariance matrix
  r_xyz_factor = nh.param("r_xyz_factor", 0.05);
  r_yaw = nh.param("r_yaw", 0.02);
  auto u_r = [this](const Eigen::VectorXd & z) {
    Eigen::DiagonalMatrix<double, 4> r;
    double x = r_xyz_factor;
    r.diagonal() << abs(x * z[0]), abs(x * z[1]), abs(x * z[2]), r_yaw;
    return r;
  };
  // P - error estimate covariance matrix
  Eigen::DiagonalMatrix<double, 9> p0;
  p0.setIdentity();
  tracker->ekf = ExtendedKalmanFilter{f, h, j_f, j_h, u_q, u_r, p0};

  
  try{
    transformStamped = tf_buffer.lookupTransform("odom", "camera",
                              ros::Time(0), ros::Duration(1.0));
  }
  catch (tf2::TransformException &ex) {
    ROS_ERROR("%s",ex.what());
  }
  armors_sub = nh.subscribe("detect_armors", 1, &TrackerNode::armorsCallback, this);
  target_pub = nh.advertise<robotinterfaces::Target>("target", 1);

  // Reset tracker service 
  using std::placeholders::_1;
  using std::placeholders::_2;
  using std::placeholders::_3;

  reset_tracker_srv = nh.advertiseService("/tracker_reset", &TrackerNode::resetTrackerCallback, this);

}

TrackerNode::~TrackerNode()
{
}

void TrackerNode::armorsCallback(const robotinterfaces::Armors::Ptr armors_msg)
{
    for(auto & armor : armors_msg->armors){
        geometry_msgs::PoseStamped ps;
        ps.header = armors_msg->header;
        ps.pose = armor.pose;
        try{
            // 执行坐标变换
            tf2::doTransform(armor.pose, armor.pose, transformStamped);
        }
        catch (tf2::TransformException &ex){
            ROS_ERROR("Error while transforming pose: %s", ex.what());
            return;
        }
    }

    // Filter abnormal armors
    armors_msg->armors.erase(
    std::remove_if(
      armors_msg->armors.begin(), armors_msg->armors.end(),
      [this](const robotinterfaces::Armor & armor) {
        return abs(armor.pose.position.z) > 1.2 ||
               Eigen::Vector2d(armor.pose.position.x, armor.pose.position.y).norm() >
                 max_armor_distance;
      }),
    armors_msg->armors.end());

    ros::Time time =  armors_msg->header.stamp;
    robotinterfaces::Target target_msg;
    target_msg.header.stamp = time;
    target_msg.header.frame_id = target_frame;

    // Update tracker
  if (tracker->tracker_state == Tracker::LOST) {
    tracker->init(armors_msg);
    target_msg.tracking = false;
  } else {
    dt = (time - last_time).toSec();
    tracker->lost_thres = static_cast<int>(lost_time_thres / dt);
    tracker->update(armors_msg);

    if (tracker->tracker_state == Tracker::DETECTING) {
      target_msg.tracking = false;
    } else if (
      tracker->tracker_state == Tracker::TRACKING ||
      tracker->tracker_state == Tracker::TEMP_LOST) {
      target_msg.tracking = true;
      // Fill target message
      const auto & state = tracker->target_state;
      target_msg.id = tracker->tracked_id;
      target_msg.armors_num = static_cast<int>(tracker->tracked_armors_num);
      target_msg.position.x = state(0);
      target_msg.velocity.x = state(1);
      target_msg.position.y = state(2);
      target_msg.velocity.y = state(3);
      target_msg.position.z = state(4);
      target_msg.velocity.z = state(5);
      target_msg.yaw = state(6);
      target_msg.v_yaw = state(7);
      target_msg.radius_1 = state(8);
      target_msg.radius_2 = tracker->another_r;
      target_msg.dz = tracker->dz;
    }
  }

  last_time = time;

  target_pub.publish(target_msg);
}

bool TrackerNode::resetTrackerCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  tracker->tracker_state = Tracker::LOST;
  res.success = true;
  res.message = "Tracker reset!";
  ROS_INFO("Tracker reset!");
  return true;
}
