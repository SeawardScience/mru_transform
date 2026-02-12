#ifndef MRU_TRANSFORM_MRU_TRANSFORM_H
#define MRU_TRANSFORM_MRU_TRANSFORM_H

#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"
#include <mru_transform/orientation_sensor.hpp>
#include <mru_transform/position_sensor.hpp>
#include <mru_transform/velocity_sensor.hpp>
#include <mru_transform/map_frame.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <std_srvs/srv/trigger.hpp> // Include the Trigger service header
// #include <mru_transform_interfaces/srv/set_map_datum.hpp>
#include <geographic_msgs/msg/geo_point.hpp>


namespace mru_transform
{

class MRUTransform
{
public:
  MRUTransform(rclcpp::Node::SharedPtr node_ptr);
  void update();
  void updatePosition(const rclcpp::Time &timestamp);
  void updateOrientation(const rclcpp::Time &timestamp);
  void updateVelocity(const rclcpp::Time &timestamp);

private:
  template<typename T, typename VST> bool updateLatest(T &value, const VST& sensors, const rclcpp::Time& now)
  {
    for(auto s: sensors){
      rclcpp::Time sensor_time = s->lastValue().header.stamp;
      // Skip uninitialized sensors (no data received yet)
      if(sensor_time.nanoseconds() == 0){
        RCLCPP_WARN_THROTTLE(
            node_ptr_->get_logger(),
            *node_ptr_->get_clock(),
            5000,
            "Sensor %s (%s) has no data yet (timestamp is 0)",
            s->name().c_str(),
            s->sensor_type.c_str()
            );
        continue;
      }

      // Sanity check: warn if sensor time is far from ROS clock
      int64_t ros_now_ns = node_ptr_->now().nanoseconds();
      int64_t sensor_ns = sensor_time.nanoseconds();
      int64_t clock_diff = std::abs(ros_now_ns - sensor_ns);
      if(clock_diff > 5000000000LL) {  // 5 seconds
        clock_synced_[s->name()] = false;
        RCLCPP_WARN_THROTTLE(
            node_ptr_->get_logger(),
            *node_ptr_->get_clock(),
            10000,
            "Sensor %s (%s) time differs from ROS clock by %.1fs (sensor=%ld ros=%ld)",
            s->name().c_str(),
            s->sensor_type.c_str(),
            clock_diff / 1e9,
            sensor_ns,
            ros_now_ns
            );
        continue;
      }

      // Alert when clock sync is restored
      if(clock_synced_.count(s->name()) && !clock_synced_[s->name()]){
        clock_synced_[s->name()] = true;
        RCLCPP_INFO(
            node_ptr_->get_logger(),
            "Sensor %s (%s) clock synced with ROS clock",
            s->name().c_str(),
            s->sensor_type.c_str()
            );
      }

      rclcpp::Time value_time = value.header.stamp;
      auto msg_age = now - sensor_time;

      // Detect clock reset (sensor restarted, power cycled, etc.)
      // If stored value is more than 5 seconds ahead of sensor, reset
      int64_t time_diff = value_time.nanoseconds() - sensor_time.nanoseconds();
      if(value_time.nanoseconds() > 0 && time_diff > 5000000000LL) {
        RCLCPP_WARN(node_ptr_->get_logger(),
                    "Sensor %s (%s) clock reset detected (stored value %.1fs ahead), resetting",
                    s->name().c_str(),
                    s->sensor_type.c_str(),
                    time_diff / 1e9);
        value = s->lastValue();
        std_msgs::msg::String active;
        active.data = s->name();
        active_sensor_pubs_[s->sensor_type]->publish(active);
        return true;
      }

      if(msg_age < sensor_timeout_){
        if(sensor_time >= value_time){
          if(sensor_time > value_time) {
            value = s->lastValue();
            std_msgs::msg::String active;
            active.data = s->name();
            active_sensor_pubs_[s->sensor_type]->publish(active);
          }
          return true;
        }
        else{
          double age = msg_age.seconds();
          RCLCPP_WARN_THROTTLE(
              node_ptr_->get_logger(),
              *node_ptr_->get_clock(),
              5000,
              "Sensor %s (%s) received out of order, %.3fs old",
              s->name().c_str(),
              s->sensor_type.c_str(),
              age
              );
        }
      }else{
        double age = msg_age.seconds();
        RCLCPP_WARN_THROTTLE(
            node_ptr_->get_logger(),
            *node_ptr_->get_clock(),
            5000,
            "Sensor %s (%s) timeout, %.1fs old",
            s->name().c_str(),
            s->sensor_type.c_str(),
            age
            );
      }
    }
    return false;
  }

  void mapDatumCallback(const geographic_msgs::msg::GeoPoint msg);

  void resetMapFrameService(const std_srvs::srv::Trigger::Request::SharedPtr request,
                            std_srvs::srv::Trigger::Response::SharedPtr response);

  // void setMapDatumService(const mru_transform_interfaces::srv::SetMapDatum::Request::SharedPtr request,
  //                           mru_transform_interfaces::srv::SetMapDatum::Response::SharedPtr response);


  // list of sensors, in order of priority
  std::vector<std::shared_ptr<PositionSensor> > position_sensors_;
  std::vector<std::shared_ptr<OrientationSensor> > orientation_sensors_;
  std::vector<std::shared_ptr<VelocitySensor> > velocity_sensors_;

  PositionSensor::ValueType latest_position_;
  OrientationSensor::ValueType latest_orientation_;
  VelocitySensor::ValueType latest_velocity_;

  std::map<std::string, rclcpp::Publisher<std_msgs::msg::String>::SharedPtr > active_sensor_pubs_;

  rclcpp::Duration sensor_timeout_ = rclcpp::Duration(1.0s);

  std::map<std::string, bool> clock_synced_;

  std::string base_frame_ = "base_link";
  std::string map_frame_ = "map";
  std::string odom_frame_ = "odom";
  std::string odom_topic_ = "odom";
  std::vector<std::string> sensor_names_ = {"example_sensor"};

  std::shared_ptr<MapFrame> mapFrame_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  nav_msgs::msg::Odometry odom_;

  rclcpp::Node::SharedPtr node_ptr_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_map_frame_service_;
  // rclcpp::Service<mru_transform_interfaces::srv::SetMapDatum>::SharedPtr set_map_datum_service_;

  rclcpp::Subscription<geographic_msgs::msg::GeoPoint>::SharedPtr map_datum_sub_;
};

} // namespace mru_transform

#endif
