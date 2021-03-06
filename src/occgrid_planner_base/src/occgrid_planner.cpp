
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
#include <std_msgs/Float32.h>
#include <math.h>


#define FREE 0xFF
#define UNKNOWN 0x80
#define OCCUPIED 0x00
#define WIN_SIZE 800


class SignalWifiManagement{
    protected:
        float signalWifi;
        cv::Point3i robotPosition;
    
    public:
        SignalWifiManagement(){
            signalWifi=0;
            robotPosition=cv::Point3i(0,0,0);
        }

        void setSignalWifi(float value){
            signalWifi=value;
        }

        void setRobotPosition(cv::Point3i Position){
            robotPosition=Position;
        }

        float getSignalWifi(){
            return signalWifi;
        }

        cv::Point3i getRobotPosition(){
            return robotPosition;
        }

};


class OccupancyGridPlanner {
    protected:
        ros::NodeHandle nh_;
        ros::Subscriber og_sub_;
        ros::Subscriber target_sub_;
        ros::Subscriber voltage_sub_; //Ajout monitoring battery
        ros::Subscriber signal_sub_; //Signal subscriber
        ros::Publisher path_pub_;
        ros::Publisher goal_pub_; //project
        tf::TransformListener listener_;
        ros::Timer timer;

        cv::Rect roi_;
        cv::Mat_<uint8_t> og_, cropped_og_;
        cv::Mat_<uint8_t> signalMap_, cropped_signal_;
        cv::Mat_<cv::Vec3b> og_rgb_, og_rgb_marked_, signalMap__rgb_;
        cv::Point3i og_center_;
        std::vector<cv::Point2i> frontierPoints_;
        nav_msgs::MapMetaData info_;
        std::string frame_id_;
        std::string base_link_;
        unsigned int neighbourhood_;
        bool ready;
        bool debug;
        double radius;
        float voltage_robot;//Ajout monitoring battery
        float signal_wifi;

        typedef std::multimap<float, cv::Point3i> Heap;
        //typedef std::multimap<float, cv::Point3i> SIGNAL_WIFI;

        std::vector<SignalWifiManagement> signal_wifi_list;



        //Find and store all frontierPoints in a 1 dim vector
        void findFrontierPoints(cv::Mat_<uint8_t> og_){
            int width=og_.size().width;
            int height=og_.size().height;
            
            //frontierPoints_.clear();
            std::vector<cv::Point2i> frontierPoints_2;
            frontierPoints_=frontierPoints_2;
            for (size_t i = 0; i < height ; i++)
            {
                for (size_t j = 0; j < width; j++)
                {


                    //if((og_(i,j)==FREE) && ((og_(i-1,j)==OCCUPIED) || (og_(i-1,j-1)==OCCUPIED) || (og_(i-1,j+1)==OCCUPIED) || (og_(i,j-1)==OCCUPIED)
                      //  || (og_(i,j+1)==OCCUPIED) || (og_(i+1,j)==OCCUPIED) || (og_(i+1,j-11)==OCCUPIED) || (og_(i+1,j+1)==OCCUPIED) ))
                    if((og_(i,j)==FREE) && ((og_(i-1,j)==UNKNOWN) || (og_(i-1,j-1)==UNKNOWN) || (og_(i-1,j+1)==UNKNOWN) || (og_(i,j-1)==UNKNOWN)
                        || (og_(i,j+1)==UNKNOWN) || (og_(i+1,j)==UNKNOWN) || (og_(i+1,j-1)==UNKNOWN) || (og_(i+1,j+1)==UNKNOWN) ))
                        {
                            cv::Point2i frontierPoint;
                            frontierPoint=cv::Point2i(j,i);
                            frontierPoints_.push_back(frontierPoint);
                            //if(frontierPoint.x>200){
                               // ROS_INFO("Frontier Point: %d %d",frontierPoint.x,frontierPoint.y);
                            //}
                            
                        }
                }
                
            }
            ROS_INFO("frontier point [0] = (%d, %d) ",frontierPoints_[0].x, frontierPoints_[0].y );
        }

        //Return the frontier point which is close to the Robot
        cv::Point2i frontierPointCloseToRobot(const cv::Point3i & currP){
            //float minDistanceToRobot= hypot(frontierPoints_[0].x - currP.x, frontierPoints_[0].y - currP.y);
            float minDistanceToRobot=10000;
            cv::Point2i closestFrontierPoint=frontierPoints_[0];

            for (size_t i = 0; i < frontierPoints_.size(); i++)
            {
                float distance=hypot(frontierPoints_[i].x - currP.x, frontierPoints_[i].y - currP.y);
                if((distance<minDistanceToRobot) && (distance>5)){
                    minDistanceToRobot=distance;
                    closestFrontierPoint=frontierPoints_[i];
                }
            }
            ROS_INFO("closestFrontierPoint = (%d, %d) ",closestFrontierPoint.x, closestFrontierPoint.y );
            return closestFrontierPoint;
            //return frontierPoints_[0];
        }


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
                            og_(j,i) = FREE; //OCCUPIED 
                            break;
                        case 100: 
                            og_(j,i) = OCCUPIED; //UNKNOWN
                            break;
                        case -1: 
                        default:
                            og_(j,i) = UNKNOWN; //FREE
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

            ROS_INFO("MSG height , width : %d , %d ",msg->info.height ,msg->info.width);
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
            //cv::cvtColor(signalMap_, signalMap__rgb_, CV_GRAY2RGB);
            // Compute a sub-image that covers only the useful part of the
            // grid.
            cropped_og_ = cv::Mat_<uint8_t>(og_,roi_);
            //cropped_signal_ = cv::Mat_<uint8_t>(og_,roi_);
           /* if ((w > WIN_SIZE) || (h > WIN_SIZE)) {
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

                //cv::Mat_<uint8_t> resized_signalMap;
                //cv::resize(cropped_signal_,resized_signalMap,new_size);
                //cv::imshow( "SignalMap", resized_signalMap );
            } else {
                // cv::imshow( "OccGrid", cropped_og_ );
                cv::imshow( "OccGrid", og_rgb_ );
                //cv::imshow( "SignalMap", signalMap__rgb_ );
            }*/
            cv::imshow( "OccGrid", og_rgb_ );


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

            cv::Point3i target;
            double t_yaw;


            //Ajout monitoring battery
            if(voltage_robot<60.0){
            // Now scale the target to the grid resolution and shift it to the
            // grid center.
                target = cv::Point3i(0,0,0)//manque une 3ème dimension
                + og_center_;
            } else {
            // Now scale the target to the grid resolution and shift it to the
            // grid center.
                t_yaw = tf::getYaw(pose.pose.orientation);
                target = cv::Point3i(pose.pose.position.x / info_.resolution, pose.pose.position.y / info_.resolution,(unsigned int)round(t_yaw/(M_PI/4)) % 8)//manque une 3ème dimension
                + og_center_;
            }




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
                //ROS_ERROR("Invalid target point: occupancy = %d",og_(point3iToPoint(target));
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
                //ROS_ERROR("Invalid start point: occupancy = %d",og_(point3iToPoint(start)));
                return;
            }

            /****************************************************************************/
            /*PROJECT:Store frontier points in a list and find closest poit to the Robot*/
            /****************************************************************************/
            
            //findFrontierPoints(og_);
            //cv::Point2i minTarget= frontierPointCloseToRobot(start);
            //ROS_INFO("Closest point to the Robot (%d %d)",minTarget.x,minTarget.y);

            /********************************/


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

            cv::Point3i neighbours[8][5]={
                //Angle= 0
                {cv::Point3i(1,0,0), cv::Point3i(1,1,1), cv::Point3i(1,-1,-1), cv::Point3i(0,0,1), cv::Point3i(0,0,-1)},
                //Angle= 45
                {cv::Point3i(1,1,0), cv::Point3i(0,1,1), cv::Point3i(1,0,-1), cv::Point3i(0,0,1), cv::Point3i(0,0,-1)},
                //Angle= 90
                {cv::Point3i(0,1,0), cv::Point3i(-1,1,1), cv::Point3i(1,1,-1), cv::Point3i(0,0,1), cv::Point3i(0,0,-1)},
                //Angle= 135
                {cv::Point3i(-1,1,0), cv::Point3i(-1,0,1), cv::Point3i(0,1,-1), cv::Point3i(0,0,1), cv::Point3i(0,0,-1)},
                //Angle= 180
                {cv::Point3i(-1,0,0), cv::Point3i(-1,-1,1), cv::Point3i(-1,1,-1), cv::Point3i(0,0,1), cv::Point3i(0,0,-1)},
                //Angle= 225
                {cv::Point3i(-1,-1,0), cv::Point3i(0,-1,1), cv::Point3i(-1,0,-1), cv::Point3i(0,0,1), cv::Point3i(0,0,-1)},
                //Angle= 270
                {cv::Point3i(0,-1,0), cv::Point3i(1,-1,1), cv::Point3i(-1,-1,-1), cv::Point3i(0,0,1), cv::Point3i(0,0,-1)},
                //Angle= 315
                {cv::Point3i(1,-1,0), cv::Point3i(1,0,1), cv::Point3i(0,-1,-1), cv::Point3i(0,0,1), cv::Point3i(0,0,-1)}
            };


            // Cost of displacement corresponding the neighbours. Diagonal
            // moves are 44% longer.

            // float cost[8] = {1, 1, 1, 1, sqrt(2), sqrt(2), sqrt(2), sqrt(2)};
            float cost[2][5] = {{1, 1, 1, 2, 2},{sqrt(2), 1, 1, 2, 2}};
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
                    dest.z = (dest.z + 8) % 8;
                    if (!isInGrid(dest)) {
                        // outside the grid
                        continue;
                    }
                    uint8_t og = og_(point3iToPoint(dest));
                    if(og == OCCUPIED){ //(og != FREE) {
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
                ROS_ERROR("No path found from (%d, %d) to (%d, %d)",start.x,start.y,start.z,target.x,target.y,target.z);
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
                
                tf::Quaternion q = tf::createQuaternionFromRPY(0,0,P.z*M_PI/4);
                tf::quaternionTFToMsg(q, path.poses[ipose].pose.orientation);

                //path.poses[ipose].pose.orientation.x = 0;
                //path.poses[ipose].pose.orientation.y = 0;
                //path.poses[ipose].pose.orientation.z = 0;
                //path.poses[ipose].pose.orientation.w = 1;
                ipose++;
                it ++;
            }
            path_pub_.publish(path);
            ROS_INFO("Request completed");
        }


        // This is called when a new goal is posted by RViz. We don't use a
        // mutex here, because it can only be called in spinOnce.
        void timer_callback(const ros::TimerEvent& e) {

            if (!ready) {
                ROS_WARN("Ignoring target while the occupancy grid has not been received");
                return;
            }
            ROS_INFO("Received planning request in timer_callback");

            // CALCULATE SATRT AND TARGET //
            //--------------------------------------------------------------------------------------------------------------------//
            cv::Point3i start;
            double s_yaw = 0;
            cv::Point3i target;
            double t_yaw = 0;

            tf::StampedTransform transform;
            // this gets the current pose in transform
            listener_.lookupTransform(frame_id_,base_link_, ros::Time(0), transform);

            if (debug) {
                start = og_center_;
            }else{
                s_yaw = tf::getYaw(transform.getRotation()) + M_PI;
                start = cv::Point3i(transform.getOrigin().x() / info_.resolution, transform.getOrigin().y() / info_.resolution,(unsigned int)round(s_yaw/(M_PI/4)) % 8)
                    + og_center_;
            }


            if (!isInGrid(start)) {
                ROS_ERROR("Invalid starting point (%.2f %.2f %.2f) -> (%d %d %d)",
                        info_.origin.position.x, info_.origin.position.y,s_yaw, start.x, start.y,start.z);
                return;
            }
 

            //Add battery minitoring
            if(voltage_robot<60.0){
            // Now scale the target to the grid resolution and shift it to the grid center.
                target = cv::Point3i(0,0,0) + og_center_;
            } else {
            // Now scale the target to the grid resolution and shift it to the grid center.
                findFrontierPoints(og_);
                cv::Point2i minTarget= frontierPointCloseToRobot(start);
                ROS_INFO("minTarget: (%d, %d)",minTarget.x,minTarget.y);
                double alpha = remainder(atan2((minTarget.y-start.y),minTarget.x-start.x)-start.z,2*M_PI);// + M_PI
                target = cv::Point3i(minTarget.x, minTarget.y,alpha+ M_PI); //+ og_center_; (unsigned int)round(alpha/(M_PI/4)) % 8 alpha
                ROS_INFO("og_center : (%d , %d)",og_center_.x,og_center_.y);
            }

            ROS_INFO("start: (%d , %d , %d) --> target: (%d , %d, %d ) , voltage = %2f", start.x, start.y, start.z, 
            target.x, target.y, target.z, voltage_robot);
            //--------------------------------------------------------------------------------------------------------------------------------//


            //DISPLAY START AND TARGET ON THE OCCGRID MAP //
            //--------------------------------------------------------------------------------------------------------------------------------//
            og_rgb_marked_ = og_rgb_.clone();

            cv::circle(og_rgb_marked_,point3iToPoint(start), 10, cv::Scalar(0,255,0));
            cv::imshow( "OccGrid", og_rgb_marked_ );
                                    
            cv::circle(og_rgb_marked_,point3iToPoint(target), 10, cv::Scalar(0,0,255),CV_FILLED);
            cv::imshow( "OccGrid", og_rgb_marked_ );

            for (size_t i = 0; i < frontierPoints_.size(); i++)
            {
                cv::circle(og_rgb_marked_,frontierPoints_[i], 1, cv::Scalar(0,255,0));
                cv::imshow( "OccGrid", og_rgb_marked_ );  
            }
            
            
            //STARTING PLANNING //
            //---------------------------------------------------------------------------------------------------------------------------------//
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

            cv::Point3i neighbours[8][5]={
                //Angle= 0
                {cv::Point3i(1,0,0), cv::Point3i(1,1,1), cv::Point3i(1,-1,-1), cv::Point3i(0,0,1), cv::Point3i(0,0,-1)},
                //Angle= 45
                {cv::Point3i(1,1,0), cv::Point3i(0,1,1), cv::Point3i(1,0,-1), cv::Point3i(0,0,1), cv::Point3i(0,0,-1)},
                //Angle= 90
                {cv::Point3i(0,1,0), cv::Point3i(-1,1,1), cv::Point3i(1,1,-1), cv::Point3i(0,0,1), cv::Point3i(0,0,-1)},
                //Angle= 135
                {cv::Point3i(-1,1,0), cv::Point3i(-1,0,1), cv::Point3i(0,1,-1), cv::Point3i(0,0,1), cv::Point3i(0,0,-1)},
                //Angle= 180
                {cv::Point3i(-1,0,0), cv::Point3i(-1,-1,1), cv::Point3i(-1,1,-1), cv::Point3i(0,0,1), cv::Point3i(0,0,-1)},
                //Angle= 225
                {cv::Point3i(-1,-1,0), cv::Point3i(0,-1,1), cv::Point3i(-1,0,-1), cv::Point3i(0,0,1), cv::Point3i(0,0,-1)},
                //Angle= 270
                {cv::Point3i(0,-1,0), cv::Point3i(1,-1,1), cv::Point3i(-1,-1,-1), cv::Point3i(0,0,1), cv::Point3i(0,0,-1)},
                //Angle= 315
                {cv::Point3i(1,-1,0), cv::Point3i(1,0,1), cv::Point3i(0,-1,-1), cv::Point3i(0,0,1), cv::Point3i(0,0,-1)}
            };


            // Cost of displacement corresponding the neighbours. Diagonal
            // moves are 44% longer.

            float cost[2][5] = {{1, 1, 1, 2, 2},{sqrt(2), 1, 1, 2, 2}};
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
                    dest.z = (dest.z + 8) % 8;
                    if (!isInGrid(dest)) {
                        // outside the grid
                        continue;
                    }
                    uint8_t og = og_(point3iToPoint(dest));
                    if(og == OCCUPIED){ //(og != FREE) {
                        // occupied or unknown
                        continue;
                    }
                    float cv = cell_value(dest.x,dest.y,dest.z);
                    float new_cost = this_cost + ((i%2)==0)? (cost[0][i]) : (cost[1][i]);
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
                ROS_ERROR("No path found from (%d, %d) to (%d, %d)",start.x,start.y,start.z,target.x,target.y,target.z);
                return;
            }
            ROS_INFO("Planning completed");

            // PUBLISH PATH //
            //--------------------------------------------------------------------------------------------------------------------------//
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
                ROS_INFO("PUBLISH point = (%d , %d)",P.x,P.y);
                path.poses[ipose].pose.position.x = (P.x) * info_.resolution;
                path.poses[ipose].pose.position.y = (P.y) * info_.resolution;
                
                tf::Quaternion q = tf::createQuaternionFromRPY(0,0,P.z*M_PI/4);
                tf::quaternionTFToMsg(q, path.poses[ipose].pose.orientation);

                //path.poses[ipose].pose.orientation.x = 0;
                //path.poses[ipose].pose.orientation.y = 0;
                //path.poses[ipose].pose.orientation.z = 0;
                //path.poses[ipose].pose.orientation.w = 1;
                ipose++;
                it ++;
            }
            path_pub_.publish(path);
            ROS_INFO("Request completed");
        }


        //Ajout monitoring battery
        void voltage_callback(const std_msgs::Float32ConstPtr & msg) {
            voltage_robot = msg->data;
        }

        void signal_callback(const std_msgs::Float32ConstPtr & msg) {
            signal_wifi = msg->data;
            //signal_wifi = static_cast<int>(signal_wifi*100);
            signal_wifi=signal_wifi*100;
            ROS_INFO("SIGNAL = %f",signal_wifi);

            int width=og_.size().width;
            int height=og_.size().height;

            if (!ready) {
                ROS_WARN("Ignoring target while the occupancy grid has not been received");
                return;
            }
            ROS_INFO("Received planning request");

            cv::Point3i start;
            double s_yaw = 0;

            tf::StampedTransform transform;
            // this gets the current pose in transform
            listener_.lookupTransform(frame_id_,base_link_, ros::Time(0), transform);

            s_yaw = tf::getYaw(transform.getRotation()) + M_PI;
            start = cv::Point3i(transform.getOrigin().x() / info_.resolution, transform.getOrigin().y() / info_.resolution,(unsigned int)round(s_yaw/(M_PI/4)) % 8)//manque une 3ème dimension
                    + og_center_;

            ROS_INFO("Actuel position = %d %d",transform.getOrigin().x() / info_.resolution,transform.getOrigin().y() / info_.resolution);

            //Create singal map
            //signalMap_=cv::Mat_<uint8_t>(height, width,0xFF);

            //for (unsigned int j=0;j<height;j++) {
                //for (unsigned int i=0;i<width;i++) {
            //Set signal value in a 10 unit radius
                    //uint8_t u_signalWifi=signal_wifi;
                    //signalMap_(j,i)=  u_signalWifi;
                    //ROS_INFO("SIGNALMAP value = %2f",u_signalWifi);

                //}
            //}

            //Display

            //SIGNAL_WIFI signal_wifi_point;
            //signal_wifi_point.insert(Heap::value_type(signal_wifi,start));
            //signal_wifi_list.push_back(signal_wifi_point);

            SignalWifiManagement signalWifiManagement;
            signalWifiManagement.setRobotPosition(start);

            signalWifiManagement.setSignalWifi(signal_wifi);

            signal_wifi_list.push_back(signalWifiManagement);

            
            signalMap_=cv::Mat_<uint8_t>(height, width,0xFF);
            cv::cvtColor(signalMap_, signalMap__rgb_, CV_GRAY2RGB);
            //signalMap__rgb_ = og_rgb_.clone();
            //cv::cvtColor(signalMap_, signalMap__rgb_, CV_BGR2Luv);
            for (size_t i = 0; i < signal_wifi_list.size(); i++)
            {
                cv::Point3i robotPosition = signal_wifi_list[i].getRobotPosition();
                float signal=signal_wifi_list[i].getSignalWifi();
                signal=static_cast<int>(signal);

                if(signal<30){
                    cv::circle(signalMap__rgb_,point3iToPoint(robotPosition), 10, cv::Scalar(signal,0,0),CV_FILLED);
                }else if(signal>70){
                    cv::circle(signalMap__rgb_,point3iToPoint(robotPosition), 10, cv::Scalar(0,signal,0),CV_FILLED);
                }else{
                    cv::circle(signalMap__rgb_,point3iToPoint(robotPosition), 10, cv::Scalar(0,0,signal),CV_FILLED);
                }
            }
            
            cv::imshow( "SignalMap", signalMap__rgb_ );
            //cv::imshow( "SignalMap", signalMap__rgb_ );

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
            //target_sub_ = nh_.subscribe("goal",1,&OccupancyGridPlanner::target_callback,this);
            //Project step
            voltage_sub_ = nh_.subscribe("voltage",1,&OccupancyGridPlanner::voltage_callback,this); //subscribe vrep/voltage
            //signal_sub_ = nh_.subscribe("/vrep/signal",1,&OccupancyGridPlanner::signal_callback,this); //subscribe vrep/signal
            path_pub_ = nh_.advertise<nav_msgs::Path>("path",1,true);
            ros::Duration(0.5).sleep();
            timer = nh_.createTimer(ros::Duration(10), &OccupancyGridPlanner::timer_callback,this);        

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
