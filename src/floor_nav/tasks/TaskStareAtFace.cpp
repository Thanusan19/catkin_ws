#include <math.h>
#include "TaskStareAtFace.h"
#include "floor_nav/TaskStareAtFaceConfig.h"
using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace floor_nav;

// #define DEBUG_GOTO

TaskIndicator TaskStareAtFace::initialise() 
{
    ROS_INFO("Setting Stare at Face to %.2f deg", cfg.target*180./M_PI);
    if (cfg.relative) {
        const geometry_msgs::Pose2D & tpose = env->getPose2D();
        initial_heading = tpose.theta;
    } else {
        initial_heading = 0.0;
    }
    return TaskStatus::TASK_INITIALISED;
}

TaskIndicator TaskStareAtFace::iterate()
{
    const std::vector<sensor_msgs::RegionOfInterest> & face_detect_ROI = env -> getface_detected(); 

    if (face_detect_ROI.empty()) {
        return TaskStatus::TASK_COMPLETED;  // As the robot detects some faces in the wall I changed Task_failed for Task_completed because it was crashing each time a fantom face was detected
    }


    uint width =face_detect_ROI[0].width;
    int imax=0;
    for(int i=1;i<face_detect_ROI.size();i++){
		if(width<face_detect_ROI[i].width){
			width = face_detect_ROI[i].width;
            imax = i;
		}
	}
    double wtarget = cfg.k_theta* (double(cfg.img_target-face_detect_ROI[imax].x_offset)-(double(face_detect_ROI[imax].width)/2.0));

    if (wtarget > cfg.max_angular_velocity) wtarget = cfg.max_angular_velocity;
    if (wtarget <-cfg.max_angular_velocity) wtarget =-cfg.max_angular_velocity;

    env->publishVelocity(0.0,wtarget);

    if (fabs(double(cfg.img_target-face_detect_ROI[imax].x_offset)-(double(face_detect_ROI[imax].width)/2.0)) < cfg.angle_threshold) {
		return TaskStatus::TASK_COMPLETED;
    }

	return TaskStatus::TASK_RUNNING;
}

TaskIndicator TaskStareAtFace::terminate()
{
    env->publishVelocity(0,0);
	return TaskStatus::TASK_TERMINATED;
}

DYNAMIC_TASK(TaskFactoryStareAtFace);
