
#include <vector>
#include <string>
#include <map>
#include <list>


#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#define FREE 0xFF
#define UNKNOWN 0x80
#define OCCUPIED 0x00
#define WIN_SIZE 800

class OccupancyGridPlanner {
    protected:
        ros::NodeHandle nh_;
        ros::Subscriber og_sub_;
        ros::Subscriber target_sub_;
        ros::Publisher path_pub_;
        tf::TransformListener listener_;

        cv::Rect roi_;
        cv::Mat_<uint8_t> og_, cropped_og_;
        cv::Mat_<cv::Vec3b> og_rgb_, og_rgb_marked_;
        cv::Point3i og_center_;
        nav_msgs::MapMetaData info_;
        std::string frame_id_;
        std::string base_link_;
        unsigned int neighbourhood_;
        bool ready;
        bool debug;
        double radius;

        typedef std::multimap<float, cv::Point3i> Heap;

        // Callback for Occupancy Grids
        void og_callback(const nav_msgs::OccupancyGridConstPtr & msg) {
            info_ = msg->info;
            frame_id_ = msg->header.frame_id;
            // Create an image to store the value of the grid.
            og_ = cv::Mat_<uint8_t>(msg->info.height, msg->info.width,0xFF);
            og_center_ = cv::Point3i(-info_.origin.position.x/info_.resolution,-info_.origin.position.y/info_.resolution,0);

            // Some variables to select the useful bounding box 
            unsigned int maxx=0, minx=msg->info.width, 
                         maxy=0, miny=msg->info.height;
            // Convert the representation into something easy to display.
            for (unsigned int j=0;j<msg->info.height;j++) {
                for (unsigned int i=0;i<msg->info.width;i++) {
                    //"data" is not a matrice but a list which contains all the first column 
                    //elements then the second column ...  
                    int8_t v = msg->data[j*msg->info.width + i];
                    switch (v) {
                        case 0: 
                            og_(j,i) = FREE; 
                            break;
                        case 100: 
                            og_(j,i) = OCCUPIED; 
                            break;
                        case -1: 
                        default:
                            og_(j,i) = UNKNOWN; 
                            break;
                    }
                    // Update the bounding box of free or occupied cells.
                    if (og_(j,i) != UNKNOWN) {
                        minx = std::min(minx,i);
                        miny = std::min(miny,j);
                        maxx = std::max(maxx,i);
                        maxy = std::max(maxy,j);
                    }
                }
            }

            //STEP 1 
			double dilation_size =radius/info_.resolution;
			cv::Mat element = getStructuringElement(cv::MORPH_ELLIPSE ,//cv::MORPH_RECT
								   cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),
								   cv::Point( dilation_size, dilation_size ) );
                                   
			// Apply the dilation on obstacles (= erosion of black cells)
			cv::erode( og_, og_, element );


            if (!ready) {
                ready = true;
                ROS_INFO("Received occupancy grid, ready to plan");
            }
            // The lines below are only for display
            unsigned int w = maxx - minx;
            unsigned int h = maxy - miny;
            roi_ = cv::Rect(minx,miny,w,h);
            cv::cvtColor(og_, og_rgb_, CV_GRAY2RGB);
            // Compute a sub-image that covers only the useful part of the
            // grid.
            cropped_og_ = cv::Mat_<uint8_t>(og_,roi_);
            if ((w > WIN_SIZE) || (h > WIN_SIZE)) {
                // The occupancy grid is too large to display. We need to scale
                // it first.
                double ratio = w / ((double)h);
                cv::Size new_size;
                if (ratio >= 1) {
                    new_size = cv::Size(WIN_SIZE,WIN_SIZE/ratio);
                } else {
                    new_size = cv::Size(WIN_SIZE*ratio,WIN_SIZE);
                }
                cv::Mat_<uint8_t> resized_og;
                cv::resize(cropped_og_,resized_og,new_size);
                cv::imshow( "OccGrid", resized_og );
            } else {
                // cv::imshow( "OccGrid", cropped_og_ );
                cv::imshow( "OccGrid", og_rgb_ );
            }
        }



        cv::Point point3iToPoint(const cv::Point3i & currPoint) {
            return cv::Point(currPoint.x, currPoint.y);
		}

        // Generic test if a point is within the occupancy grid
        bool isInGrid(const cv::Point3i & P) {
            if ((P.x < 0) || (P.x >= (signed)info_.width) 
                    || (P.y < 0) || (P.y >= (signed)info_.height)) {
                return false;
            }
            return true;
        }


        double heuristic(const cv::Point3i & currP, const cv::Point3i & goalP) {
            return hypot(goalP.x - currP.x, goalP.y - currP.y);
		}



        // This is called when a new goal is posted by RViz. We don't use a
        // mutex here, because it can only be called in spinOnce.
        void target_callback(const geometry_msgs::PoseStampedConstPtr & msg) {
            tf::StampedTransform transform;
            geometry_msgs::PoseStamped pose;
            if (!ready) {
                ROS_WARN("Ignoring target while the occupancy grid has not been received");
                return;
            }
            ROS_INFO("Received planning request");
            og_rgb_marked_ = og_rgb_.clone();
            // Convert the destination point in the occupancy grid frame. 
            // The debug case is useful is the map is published without
            // gmapping running (for instance with map_server).
            if (debug) {
                pose = *msg;
            } else {
                // This converts target in the grid frame.
                listener_.waitForTransform(frame_id_,msg->header.frame_id,msg->header.stamp,ros::Duration(1.0));
                listener_.transformPose(frame_id_,*msg, pose);
                // this gets the current pose in transform
                listener_.lookupTransform(frame_id_,base_link_, ros::Time(0), transform);
            }
            // Now scale the target to the grid resolution and shift it to the
            // grid center.
            double t_yaw = tf::getYaw(pose.pose.orientation);
            cv::Point3i target = cv::Point3i(pose.pose.position.x / info_.resolution, pose.pose.position.y / info_.resolution,(unsigned int)round(t_yaw/(M_PI/4)) % 8)//manque une 3ème dimension
                + og_center_;
            ROS_INFO("Planning target: %.2f %.2f -> %d %d",
                        pose.pose.position.x, pose.pose.position.y, target.x, target.y);
            cv::circle(og_rgb_marked_,point3iToPoint(target), 10, cv::Scalar(0,0,255));
            cv::imshow( "OccGrid", og_rgb_marked_ );
            if (!isInGrid(target)) {
                ROS_ERROR("Invalid target point (%.2f %.2f %.2f) -> (%d %d %d)",
                        pose.pose.position.x, pose.pose.position.y,t_yaw, target.x, target.y,target.z);
                return;
            }
            // Only accept target which are FREE in the grid (HW, Step 5).
            if (og_(point3iToPoint(target)) != FREE) {
                ROS_ERROR("Invalid target point: occupancy = %d",og_((point3iToPoint(target)));
                return;
            }

            // Now get the current point in grid coordinates.
            cv::Point3i start;
            double s_yaw = 0;
            if (debug) {
                start = og_center_;
            } else {
                s_yaw = tf::getYaw(transform.getRotation());
                start = cv::Point3i(transform.getOrigin().x() / info_.resolution, transform.getOrigin().y() / info_.resolution,(unsigned int)round(s_yaw/(M_PI/4)) % 8)//manque une 3ème dimension
                    + og_center_;
            }
            ROS_INFO("Planning origin %.2f %.2f %.2f -> %d %d %d",
                    transform.getOrigin().x(), transform.getOrigin().y(),s_yaw, start.x, start.y,start.z);
            cv::circle(og_rgb_marked_,point3iToPoint(start), 10, cv::Scalar(0,255,0));
            cv::imshow( "OccGrid", og_rgb_marked_ );

            if (!isInGrid(start)) {
                ROS_ERROR("Invalid starting point (%.2f %.2f %.2f) -> (%d %d %d)",
                        transform.getOrigin().x(), transform.getOrigin().y(),s_yaw, start.x, start.y,start.z);
                return;
            }
            // If the starting point is not FREE there is a bug somewhere, but
            // better to check
            if (og_(point3iToPoint(start)) != FREE) {
                ROS_ERROR("Invalid start point: occupancy = %d",og_(point3iToPoint(start)));
                return;
            }
            ROS_INFO("Starting planning from (%d, %d %d) to (%d, %d %d)",start.x,start.y,start.z, target.x, target.y,target.z);
            // Here the Dijskstra algorithm starts 
            // The best distance to the goal computed so far. This is
            // initialised with Not-A-Number. 
            int dimension[3]={og_.size().width, og_.size().height, 8};

            cv::Mat_<float> cell_value(3,dimension, NAN);// this is a matrice with the same dim as "og_" and having float values
            // For each cell we need to store a pointer to the coordinates of
            // its best predecessor. 
            cv::Mat_<cv::Vec3s> predecessor(3,dimension); 

            // The neighbour of a given cell in relative coordinates. The order
            // is important. If we use 4-connexity, then we can use only the
            // first 4 values of the array. If we use 8-connexity we use the
            // full array.
            cv::Point3i neighbours[8][5];
            //Angle= 0
            neighbours[0][:] = {cv::Point3i(1,0,0), cv::Point3i(1,1,90), cv::Point3i(-1,-1,-90), cv::Point3i(2,1,45), cv::Point3i(2,-1,-45)};
            //Angle= 45
            neighbours[1][:] = {cv::Point3i(1,1,0), cv::Point3i(0,1,90), cv::Point3i(1,0,-90), cv::Point3i(2,1,45), cv::Point3i(1,2,-45)};
            //Angle= 90
            neighbours[2][:] = {cv::Point3i(1,0,0), cv::Point3i(1,1,90), cv::Point3i(-1,-1,-90), cv::Point3i(2,1,45), cv::Point3i(2,-1,-45)};
            //Angle= 135
            neighbours[3][:] = {cv::Point3i(1,0,0), cv::Point3i(1,1,90), cv::Point3i(-1,-1,-90), cv::Point3i(2,1,45), cv::Point3i(2,-1,-45)};
            //Angle= 180
            neighbours[4][:] = {cv::Point3i(1,0,0), cv::Point3i(1,1,90), cv::Point3i(-1,-1,-90), cv::Point3i(2,1,45), cv::Point3i(2,-1,-45)};
            //Angle= 225
            neighbours[5][:] = {cv::Point3i(1,0,0), cv::Point3i(1,1,90), cv::Point3i(-1,-1,-90), cv::Point3i(2,1,45), cv::Point3i(2,-1,-45)};
            //Angle= 270
            neighbours[6][:] = {cv::Point3i(1,0,0), cv::Point3i(1,1,90), cv::Point3i(-1,-1,-90), cv::Point3i(2,1,45), cv::Point3i(2,-1,-45)};
            //Angle= 315
            neighbours[7][:] = {cv::Point3i(1,0,0), cv::Point3i(1,1,90), cv::Point3i(-1,-1,-90), cv::Point3i(2,1,45), cv::Point3i(2,-1,-45)};

            // Cost of displacement corresponding the neighbours. Diagonal
            // moves are 44% longer.

            // float cost[8] = {1, 1, 1, 1, sqrt(2), sqrt(2), sqrt(2), sqrt(2)};
            float cost[2][5] = {{1, 1, 1, 2, 2},{sqrt(2), 1, 1, 2, 2})};
            // The core of Dijkstra's Algorithm, a sorted heap, where the first
            // element is always the closer to the start.
            Heap heap;
            heap.insert(Heap::value_type(heuristic(start,target), start)); //  Heap::value_type(0, start)
            cell_value(start.x,start.y,start.z) = 0;
            while (!heap.empty()) {
                // Select the cell at the top of the heap
                Heap::iterator hit = heap.begin();
                // the cell it contains is this_cell
                cv::Point3i this_cell = hit->second;
                // and its score is this_cost
                float this_cost = hit->first;
                // We can remove it from the heap now.
                heap.erase(hit);
                // Now see where we can go from this_cell
                for (unsigned int i=0;i<neighbourhood_;i++) {//5

                    cv::Point3i dest = this_cell + neighbours[this_cell.z][i];
                    dest.z = dest.z %8;
                    if (!isInGrid(dest)) {
                        // outside the grid
                        continue;
                    }
                    uint8_t og = og_(point3iToPoint(dest));
                    if (og != FREE) {
                        // occupied or unknown
                        continue;
                    }
                    float cv = cell_value(dest.x,dest.y,dest.z);
                    float new_cost = this_cost + ((i%2)==0)? (cost[0][i]) : (cost[1][i]); //condition ? result1 : result2               float new_cost = this_cost + cost[i]
                    if (isnan(cv) || (new_cost < cv)) {
                        // found shortest path (or new path), updating the
                        // predecessor and the value of the cell
                        predecessor.at<cv::Vec3s>(dest.x,dest.y,dest.z) = cv::Vec3s(this_cell.x,this_cell.y,this_cell.z);
                        cell_value(dest.x,dest.y,dest.z) = new_cost;
                        // And insert the selected cells in the map.
                        heap.insert(Heap::value_type(new_cost+heuristic(dest,target),dest)); // Heap::value_type(new_cost,dest)
                    }
                }
            }
            if (isnan(cell_value(target.x,target.y,target.z))) {
                // No path found
                ROS_ERROR("No path found from (%d, %d) to (%d, %d)",start.x,start.y,,start.z,target.x,target.y,target.z);
                return;
            }
            ROS_INFO("Planning completed");
            // Now extract the path by starting from goal and going through the
            // predecessors until the starting point
            std::list<cv::Point3i> lpath;
            while (target != start) {
                lpath.push_front(target);
                cv::Vec3s p = predecessor(target.x,target.y,target.z);
                target.x = p[0]; target.y = p[1], target.z=p[2];
            }
            lpath.push_front(start);
            // Finally create a ROS path message
            nav_msgs::Path path;
            path.header.stamp = ros::Time::now();
            path.header.frame_id = frame_id_;
            path.poses.resize(lpath.size());
            std::list<cv::Point3i>::const_iterator it = lpath.begin();
            unsigned int ipose = 0;
            while (it != lpath.end()) {
                // time stamp is not updated because we're not creating a
                // trajectory at this stage
                path.poses[ipose].header = path.header;
                cv::Point3i P = *it - og_center_;// Put the point "P" on the top-left of the box
                path.poses[ipose].pose.position.x = (P.x) * info_.resolution;
                path.poses[ipose].pose.position.y = (P.y) * info_.resolution;
                path.poses[ipose].pose.orientation.x = 0;
                path.poses[ipose].pose.orientation.y = 0;
                path.poses[ipose].pose.orientation.z = 0;
                path.poses[ipose].pose.orientation.w = 1;
                ipose++;
                it ++;
            }
            path_pub_.publish(path);
            ROS_INFO("Request completed");
        }



    public:
        OccupancyGridPlanner() : nh_("~") {
            int nbour = 4;
            ready = false;
            nh_.param("base_frame",base_link_,std::string("/bubbleRob")); //body
            nh_.param("debug",debug,false);
            nh_.param("neighbourhood",nbour,nbour);
            nh_.param("radius",radius,0.3);
            //nbour = number possible movements from one box (4 or 8 (if diagonal mvt is allowed)
            switch (nbour) {
                case 4: neighbourhood_ = nbour; break;
                case 5: neighbourhood_ = nbour; break;
                case 8: neighbourhood_ = nbour; break;
                default: 
                    ROS_WARN("Invalid neighbourhood specification (%d instead of 4 or 8)",nbour);
                    neighbourhood_ = 8;
            }
            og_sub_ = nh_.subscribe("occ_grid",1,&OccupancyGridPlanner::og_callback,this);
            target_sub_ = nh_.subscribe("goal",1,&OccupancyGridPlanner::target_callback,this);
            path_pub_ = nh_.advertise<nav_msgs::Path>("path",1,true);
        }
};

int main(int argc, char * argv[]) {
    ros::init(argc,argv,"occgrid_planner");
    OccupancyGridPlanner ogp;
    cv::namedWindow( "OccGrid", CV_WINDOW_AUTOSIZE );
    while (ros::ok()) {
        ros::spinOnce();
        if (cv::waitKey( 50 )== 'q') {
            ros::shutdown();
        }
    }
}
