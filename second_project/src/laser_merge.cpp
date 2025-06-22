#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <limits>
#include <cmath>

sensor_msgs::LaserScan front_scan;
sensor_msgs::LaserScan back_scan;
bool got_front = false;
bool got_back = false;

tf2_ros::Buffer tfBuffer;

void frontCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    front_scan = *msg;
    got_front = true;
}

void backCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    back_scan = *msg;
    got_back = true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "scan_merger_node");
    ros::NodeHandle nh;

    ros::Subscriber sub_front = nh.subscribe("/scan_front", 1, frontCallback);
    ros::Subscriber sub_back = nh.subscribe("/scan_back", 1, backCallback);
    ros::Publisher pub_scan = nh.advertise<sensor_msgs::LaserScan>("/scan_merged", 1);

    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::Rate rate(10);

    while (ros::ok()) {
        ros::spinOnce();

        if (got_front && got_back) {
            try {
                geometry_msgs::TransformStamped tf_front = tfBuffer.lookupTransform("base_link", front_scan.header.frame_id, ros::Time(0), ros::Duration(1.0));
                geometry_msgs::TransformStamped tf_back = tfBuffer.lookupTransform("base_link", back_scan.header.frame_id, ros::Time(0), ros::Duration(1.0));

                sensor_msgs::LaserScan merged_scan;
                merged_scan.header.stamp = ros::Time::now();
                merged_scan.header.frame_id = "base_link";
                merged_scan.angle_min = -M_PI;
                merged_scan.angle_max = M_PI;
                merged_scan.angle_increment = front_scan.angle_increment; 
                merged_scan.time_increment = front_scan.time_increment;
                merged_scan.scan_time = front_scan.scan_time;
                merged_scan.range_min = 0.25f;
                merged_scan.range_max = 20.0f;

                int num_ranges = std::floor((merged_scan.angle_max - merged_scan.angle_min) / merged_scan.angle_increment);
                merged_scan.ranges.assign(num_ranges, std::numeric_limits<float>::infinity());
                merged_scan.intensities.assign(num_ranges, 0.0f);

                auto insert_scan = [&](const sensor_msgs::LaserScan& scan, const geometry_msgs::TransformStamped& tf) {
                    tf2::Transform transform;
                    tf2::fromMsg(tf.transform, transform);

                    for (size_t i = 0; i < scan.ranges.size(); ++i) {
                        float angle = scan.angle_min + i * scan.angle_increment;
                        float range = scan.ranges[i];
                        if (range < 0.25f || range > 20.0f || std::isnan(range)) continue;

                        float x = range * std::cos(angle);
                        float y = range * std::sin(angle);
                        tf2::Vector3 pt(x, y, 0.0);
                        tf2::Vector3 pt_transformed = transform * pt;

                        float angle_in_base = std::atan2(pt_transformed.y(), pt_transformed.x());
                        float range_in_base = std::hypot(pt_transformed.x(), pt_transformed.y());
                        int index = static_cast<int>((angle_in_base - merged_scan.angle_min) / merged_scan.angle_increment);
                        if (index >= 0 && index < num_ranges && range_in_base < merged_scan.ranges[index]) {
                            merged_scan.ranges[index] = range_in_base;
                            if (i < scan.intensities.size()) {
                                merged_scan.intensities[index] = scan.intensities[i];
                            } else {
                                merged_scan.intensities[index] = 0.0f;
                            }
                        }
                    }
                };

                insert_scan(front_scan, tf_front);
                insert_scan(back_scan, tf_back);
                
                got_back = false;
                got_front = false;

                pub_scan.publish(merged_scan);

            } catch (tf2::TransformException &ex) {
                ROS_WARN("TF transform failed: %s", ex.what());
            }
        }

        rate.sleep();
    }

    return 0;
}
