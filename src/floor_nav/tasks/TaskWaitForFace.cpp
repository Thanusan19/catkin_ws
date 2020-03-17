#include <math.h>
#include "TaskWaitForFace.h"
#include "floor_nav/TaskWaitForFaceConfig.h"
using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace floor_nav;


TaskIndicator TaskWaitForFace::iterate()
{
    //const geometry_msgs::Pose2D & tpose = env->getPose2D();  pas besoin de getPose2D je pense
    const std::vector<sensor_msgs::RegionOfInterest> & face_detect_ROI = env -> getface_detected();

    uint width = 20;
    if (not(face_detect_ROI.empty()))
    {
        width =face_detect_ROI[0].width;
        for(int i=1;i<face_detect_ROI.size();i++)
        {
	        if(width<face_detect_ROI[i].width)
            {
		        width = face_detect_ROI[i].width;
		    }
	    } 
    }

    if (width>45) {
        // for(int i=0; i<face_detect_ROI.size();i++){
        // ROS_INFO("Detected Face at %u %u",face_detect_ROI[i].x_offset, face_detect_ROI[i].y_offset);
        // }
		return TaskStatus::TASK_COMPLETED;
    }
	return TaskStatus::TASK_RUNNING;
}

DYNAMIC_TASK(TaskFactoryWaitForFace);
