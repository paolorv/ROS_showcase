#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <fstream>
#include <sstream>
#include <vector>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

struct Goal {
    double x, y, theta;
};

class GoalPublisher {
public:
    GoalPublisher() : ac_("move_base", true) {
        ROS_INFO("Waiting for the move_base action server to come up");
        ac_.waitForServer();
        
        loadGoalsFromCSV();
        sendNextGoal();
    }

private:
    MoveBaseClient ac_;
    std::vector<Goal> goals_;
    size_t current_goal_index_ = 0;

    void loadGoalsFromCSV() {
        std::string csv_path;
        ros::param::param<std::string>("~csv_file", csv_path, "csv/goals.csv");
        
        std::ifstream file(csv_path);
        if (!file.is_open()) {
            ROS_ERROR("Cannot open CSV file: %s", csv_path.c_str());
            return;
        }

        std::string line;
        while (std::getline(file, line)) {
            std::stringstream ss(line);
            std::string item;
            Goal goal;
            
            if (std::getline(ss, item, ',')) {
                goal.x = std::stod(item);
            }
            if (std::getline(ss, item, ',')) {
                goal.y = std::stod(item);
            }
            if (std::getline(ss, item, ',')) {
                goal.theta = std::stod(item);
            }
            
            goals_.push_back(goal);
        }
        
        ROS_INFO("Loaded %lu goals from CSV", goals_.size());
    }

    void sendNextGoal() {
        if (current_goal_index_ >= goals_.size()) {
            ROS_INFO("All goals completed!");
            return;
        }

        Goal& goal = goals_[current_goal_index_];
        
        move_base_msgs::MoveBaseGoal mb_goal;
        mb_goal.target_pose.header.frame_id = "map";
        mb_goal.target_pose.header.stamp = ros::Time::now();
        
        mb_goal.target_pose.pose.position.x = goal.x;
        mb_goal.target_pose.pose.position.y = goal.y;
        mb_goal.target_pose.pose.position.z = 0.0;
        
        tf2::Quaternion q;
        q.setRPY(0, 0, goal.theta);
        mb_goal.target_pose.pose.orientation = tf2::toMsg(q);

        ROS_INFO("Sending goal %lu: x=%.2f, y=%.2f, theta=%.2f", 
                 current_goal_index_ + 1, goal.x, goal.y, goal.theta);

        ac_.sendGoal(mb_goal,
                     boost::bind(&GoalPublisher::goalDoneCallback, this, _1, _2),
                     MoveBaseClient::SimpleActiveCallback(),
                     MoveBaseClient::SimpleFeedbackCallback());
    }

    void goalDoneCallback(const actionlib::SimpleClientGoalState& state,
                         const move_base_msgs::MoveBaseResultConstPtr& result) {
        if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("Goal %lu reached successfully", current_goal_index_ + 1);
        } else {
            ROS_WARN("Goal %lu failed: %s", current_goal_index_ + 1, state.toString().c_str());
        }
        
        current_goal_index_++;
        ros::Duration(1.0).sleep(); // Wait 1 second before next goal
        sendNextGoal();
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "goal_publisher");
    GoalPublisher gp;
    ros::spin();
    return 0;
}