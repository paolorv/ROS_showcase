// pr414 @PoliMi - 2025
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <first_project/secotor_times.h>
#include <vector>
#include <cmath>

class SectorTimesNode {
public:
    SectorTimesNode() : nh_("~") {
        // Define Monza track sector boundaries
        initializeMonzaSectors();
        
        // Subscribe to topics
        speed_sub_ = nh_.subscribe("/speedsteer", 10, &SectorTimesNode::speedCallback, this);
        gps_sub_ = nh_.subscribe("/swiftnav/front/gps_pose", 10, &SectorTimesNode::gpsCallback, this);
        
        // Publisher for sector times
        sector_pub_ = nh_.advertise<first_project::secotor_times>("/sector_times", 10);
        
        // Initialize variables
        current_sector_ = 0;
        previous_sector_ = 0;
        sector_start_time_ = ros::Time::now();
        sector_speeds_.clear();
        
        ROS_INFO("Monza Sector Times Node initialized");
    }

    void run() {
        ros::spin();
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber speed_sub_;
    ros::Subscriber gps_sub_;
    ros::Publisher sector_pub_;
    
    // Vehicle data
    double current_speed_ = 0.0;
    double current_lat_ = 0.0;
    double current_lon_ = 0.0;
    double prev_lat_ = 0.0;
    double prev_lon_ = 0.0;
    bool position_initialized_ = false;
    
    // Sector data
    int current_sector_ = 0;
    int previous_sector_ = 0;
    ros::Time sector_start_time_;
    std::vector<double> sector_speeds_;

    // Control markers
    bool isStart = true;
    int sector_check_sleep = 60; // Cooldown time for sector check after crossing a boundary
    int check_delay = 0; // Delay counter for sector check
    
    // Sector definitions
    struct SectorBoundary {
        double lat1, lon1;  // Start point
        double lat2, lon2;  // End point
        std::string name;  // Description of the sector boundary
    };
    std::vector<SectorBoundary> sector_boundaries_;
    
    
    void initializeMonzaSectors() {
        // Monza F1 Circuit sector boundaries using actual GPS coordinates
        SectorBoundary sector1;
        sector1.lat1 = 45.616194; sector1.lon1 = 9.280821;  // Start/Finish line
        sector1.lat2 = 45.630068; sector1.lon2 = 9.289492;  // End of Curva Grande
        sector1.name = "Start Line to Turn 4";
        sector_boundaries_.push_back(sector1);
        
        // Sector 2: Second Lesmo to Ascari entrance
        SectorBoundary sector2;
        sector2.lat1 = 45.630068; sector2.lon1 = 9.289492;  // End of Curva Grande
        sector2.lat2 = 45.623570; sector2.lon2 = 9.287208;  // Entrance to Ascari chicane
        sector2.name = "Turn 4 to Turn 8";
        sector_boundaries_.push_back(sector2);
        
        // Sector 3: Ascari to Start/Finish line (completing the lap)
        SectorBoundary sector3;
        sector3.lat1 = 45.623570; sector3.lon1 = 9.287208;  // Entrance to Ascari chicane
        sector3.lat2 = 45.616194; sector3.lon2 = 9.280821;  // Start/Finish line
        sector3.name = "Turn 8 to Start Line";
        sector_boundaries_.push_back(sector3);
        
        ROS_INFO("Initialized %lu Monza sectors", sector_boundaries_.size());
        for (size_t i = 0; i < sector_boundaries_.size(); i++) {
            ROS_INFO("Sector %lu: %s", i+1, sector_boundaries_[i].name.c_str());
        }
    }
    
    void speedCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
        // Extract speed from PointStamped message
        // Evaluate vector module
        double current_speed_x = msg->point.x;
        double current_speed_y = msg->point.y;
        current_speed_ = sqrt(current_speed_x * current_speed_x + current_speed_y * current_speed_y);

        // Add the speed to the current sector's speed array for average calculation (ignore if null)
        if (current_sector_ > 0 && current_speed_ > 0.1) {
            sector_speeds_.push_back(current_speed_);
        }
        
        ROS_DEBUG("Current speed: %.2f", current_speed_);
    }
    
    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
        // Update current position
        prev_lat_ = current_lat_;
        prev_lon_ = current_lon_;
        
        current_lat_ = msg->latitude;
        current_lon_ = msg->longitude;
        
        // Initialize previous position if this is first callback
        if (!position_initialized_) {
            prev_lat_ = current_lat_;
            prev_lon_ = current_lon_;
            position_initialized_ = true;
            return;
        }
        
        if (isStart) {
            current_sector_ = 1;
            isStart = false;
            sector_start_time_ = ros::Time::now();
            sector_speeds_.clear();
            ROS_INFO("Started sector %d (%s)", current_sector_, sector_boundaries_[current_sector_-1].name.c_str());
        }
        // Check if we're crossing a sector boundary if enough time has passed from last 
        if (check_delay > sector_check_sleep) {
            checkSectorBoundary();
            //check_delay = 0;
        }
        check_delay++;

        // Publish current sector info
        if (current_sector_ > 0) {
            publishSectorInfo();
        }
    }
    
    // Function to calculate distance between two GPS coordinates
    double calculateDistance(double lat1, double lon1, double lat2, double lon2) {
        // Haversine formula for distance between two GPS coordinates
        // Earth's radius in meters
        const double R = 6371000.0;
        
        // Convert to radians
        double phi1 = lat1 * M_PI / 180.0;
        double phi2 = lat2 * M_PI / 180.0;
        double delta_phi = (lat2 - lat1) * M_PI / 180.0;
        double delta_lambda = (lon2 - lon1) * M_PI / 180.0;
        
        double a = sin(delta_phi / 2) * sin(delta_phi / 2) +
                  cos(phi1) * cos(phi2) *
                  sin(delta_lambda / 2) * sin(delta_lambda / 2);
        double c = 2 * atan2(sqrt(a), sqrt(1 - a));
        
        return R * c;
    }

    void checkSectorBoundary() {
        // Determine the next sector to check (circular, so after last sector comes the first)
        int next_sector_to_check;
        
        if (current_sector_ == 0) {
            // If we haven't entered any sector yet, check the first one
            next_sector_to_check = 0;
        } else {
            // Check the next sector (with wrap-around)
            next_sector_to_check = current_sector_ % sector_boundaries_.size();
        }
        
        // Check if we're near the starting point of the next sector
        const double proximity_threshold = 0.0005; // 0.0005 degrees lat/long
        
        double lat_start = sector_boundaries_[next_sector_to_check].lat1;
        double lon_start = sector_boundaries_[next_sector_to_check].lon1;
        
        // Check if current position is close to the sector's starting point
        if (isNearPoint(current_lat_, current_lon_, lat_start, lon_start, proximity_threshold)) {
            
            // Reset the delay counter
            check_delay = 0;
            // We've entered the next sector
            previous_sector_ = current_sector_;
            current_sector_ = next_sector_to_check + 1; // Convert from 0-based to 1-based indexing
            
            // If we've changed sectors, calculate the previous sector's time
            if (previous_sector_ != current_sector_ && previous_sector_ > 0) {
                // Calculate sector time
                ros::Duration sector_time = ros::Time::now() - sector_start_time_;
                
                // Calculate mean speed for the sector
                double mean_speed = calculateMeanSpeed();
                
                // Reset for new sector
                sector_start_time_ = ros::Time::now();
                sector_speeds_.clear();
                
                ROS_INFO("Sector %d (%s) completed in %.3f seconds with mean speed %.2f km/h", 
                         previous_sector_, 
                         sector_boundaries_[previous_sector_-1].name.c_str(),
                         sector_time.toSec(), 
                         mean_speed);
            } else if (current_sector_ != previous_sector_) {
                // First sector detected
                sector_start_time_ = ros::Time::now();
                sector_speeds_.clear();
                ROS_INFO("Entered sector %d (%s)", 
                         current_sector_, 
                         sector_boundaries_[current_sector_-1].name.c_str());
            }
        }
    }

    bool isNearPoint(double lat, double lon, double target_lat, double target_lon, double threshold) {
        // Euclidean distance check in the latitude/longitude space for small distances
        double lat_diff = fabs(lat - target_lat);
        double lon_diff = fabs(lon - target_lon);
        
        // If both lat and lon differences are within threshold
        return (lat_diff <= threshold && lon_diff <= threshold);
    }
    
    double calculateMeanSpeed() {
        if (sector_speeds_.empty()) {
            return 0.0;
        }
        
        double sum = 0.0;
        for (const auto& speed : sector_speeds_) {
            sum += speed;
        }
        
        return sum / sector_speeds_.size();
    }
    
    void publishSectorInfo() {
        first_project::secotor_times msg;
        msg.current_sector = current_sector_;
        msg.current_sector_time = (ros::Time::now() - sector_start_time_).toSec();
        msg.current_sector_mean_speed = calculateMeanSpeed();
        
        sector_pub_.publish(msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "sector_times");
    
    SectorTimesNode node;
    node.run();
    
    return 0;
}