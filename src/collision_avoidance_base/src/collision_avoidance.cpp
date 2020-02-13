
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>


class CollisionAvoidance {
    protected:
        ros::Subscriber scanSub;
        ros::Subscriber cloudSub;
        ros::Subscriber velSub;
        ros::Publisher velPub;

        ros::NodeHandle nh;

        // This might be useful
        double radius;

        pcl::PointCloud<pcl::PointXYZ> lastpc,lastScan;

        void velocity_filter(const geometry_msgs::TwistConstPtr msg) {
            geometry_msgs::Twist filtered = findClosestAcceptableVelocity(*msg);
            velPub.publish(filtered);
        }

        void pc_callback(const sensor_msgs::PointCloud2ConstPtr msg) {
            pcl::fromROSMsg(*msg, lastpc);
            // unsigned int n = lastpc.size();
            // ROS_INFO("New point cloud: %d points",n);
            // for (unsigned int i=0;i<n;i++) {
            //     float x = lastpc[i].x;
            //     float y = lastpc[i].y;
            //     float z = lastpc[i].z;
            //     ROS_INFO("%d %.3f %.3f %.3f",i,x,y,z);
            // }
            // printf("\n\n\n");
        }

        void scan_callback(const sensor_msgs::LaserScanConstPtr msg) {
             unsigned int n = msg->ranges.size();
             //ROS_INFO("New point cloud: %d points",n);
             for (unsigned int i=0;i<n;i++) {
                 float theta = msg->angle_min + i * msg->angle_increment;
                 float x = msg->ranges[i]*cos(theta);
                 float y = msg->ranges[i]*sin(theta);
                 float z = 0;
             
				 if(x>0 && abs(y)<radius){
					lastScan.points.push_back(pcl::PointXYZ(x,y,z));
				 }
				 
                 //ROS_INFO("%d %.3f %.3f %.3f",i,x,y,z);
             }
             //printf("\n\n\n");
        }

        geometry_msgs::Twist findClosestAcceptableVelocity(const geometry_msgs::Twist & desired) {
            geometry_msgs::Twist res = desired;
            // TODO: modify desired using the laser point cloud or laser scan
            float distance=lastScan[0].x;
            float max_distance = 1;
            float min_distance = 0.2;
            for(int i=1;i<lastScan.size();i++){
				if(lastScan[i].x<distance){
					distance=lastScan[i].x;
				}
			}
			lastScan.clear();
            ROS_INFO("%.3f",distance);
            printf("\n");
            
			if(distance < max_distance && distance > min_distance){
				res.linear.x = (distance)*res.linear.x;//(distance-1)*res.linear.x
			}else if(distance < min_distance){
				res.linear.x =0;
			}else{
				res.linear.x =res.linear.x;
			}
            return res;
        }

    public:
        CollisionAvoidance() : nh("~"), radius(1.0) {
            scanSub = nh.subscribe("scans",1,&CollisionAvoidance::scan_callback,this);
            cloudSub = nh.subscribe("clouds",1,&CollisionAvoidance::pc_callback,this);
            velSub = nh.subscribe("cmd_vel",1,&CollisionAvoidance::velocity_filter,this);
            velPub = nh.advertise<geometry_msgs::Twist>("output_vel",1);
            nh.param("radius",radius,1.0);
        }

};

int main(int argc, char * argv[]) 
{
    ros::init(argc,argv,"collision_avoidance");

    CollisionAvoidance ca;

    ros::spin();
    // TODO: implement a security layer
}


