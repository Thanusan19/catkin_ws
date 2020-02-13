#include <math.h>
#include "TaskGoToPose.h"
#include "floor_nav/TaskGoToPoseConfig.h"
using namespace task_manager_msgs;
using namespace task_manager_lib;
using namespace floor_nav;

// #define DEBUG_GOTO
#ifdef DEBUG_GOTOPOSE
#warning Debugging task GOTOPOSE
#endif


TaskIndicator TaskGoToPose::initialise() 
{
    ROS_INFO("Going to %.2f %.2f",cfg.goal_x,cfg.goal_y);
    if (cfg.relative) {
		//Get the current position of the robot and initialize x_init and y_init
        const geometry_msgs::Pose2D & tpose = env->getPose2D();
        x_init = tpose.x;
        y_init = tpose.y;
    } else {
        x_init = 0.0;
        y_init = 0.0;
    }
    return TaskStatus::TASK_INITIALISED;
}


TaskIndicator TaskGoToPose::iterate()
{
	if(cfg.dumbSmart){//DUMP
		
		//Get the current position of the robot
		const geometry_msgs::Pose2D & tpose = env->getPose2D();
		//Get the distance between the current position and the goal
		double r = hypot(y_init + cfg.goal_y-tpose.y,x_init + cfg.goal_x-tpose.x);
		//TASK COMPLETED : if the distance is less than a value fixed by the cfg file
		if (r < cfg.dist_threshold) {
			double gamma = remainder(cfg.goal_angle-tpose.theta,2*M_PI);
			if(fabs(gamma)>0.1){
				double rot = ((gamma>0)?+1:-1)*cfg.max_angular_velocity;
				env->publishVelocity(0,rot);
			}else{
				return TaskStatus::TASK_COMPLETED;
			}
		}
		
		//Get the angle between the robot current angle and the goal angle
		double alpha = remainder(atan2((y_init + cfg.goal_y-tpose.y),x_init + cfg.goal_x-tpose.x)-tpose.theta,2*M_PI);
		//double alpha=cfg.goal_angle;
	#ifdef DEBUG_GOTOPOSE
		//Display all values
		printf("c %.1f %.1f %.1f g %.1f %.1f r %.3f alpha %.1f\n",
				tpose.x, tpose.y, tpose.theta*180./M_PI,
				cfg.goal_x,cfg.goal_y,r,alpha*180./M_PI);
	#endif
		//If the angle difference between current position and goal is > to PI/9 -->just turn
		if (fabs(alpha) > M_PI/9) {
			double rot = ((alpha>0)?+1:-1)*cfg.max_angular_velocity;
	#ifdef DEBUG_GOTOPOSE
			//Display 'rot' value
			printf("Cmd v %.2f r %.2f\n",0.,rot);
	#endif
			//publish the 'rot' on the velocity publisher
			env->publishVelocity(0,rot);
		} else {
			double vel = cfg.k_v * r;//Give a gain 'k-v' to the trnaslation speed and the distance 'r' is a variable
			double rot = std::max(std::min(cfg.k_alpha*alpha,cfg.max_angular_velocity),-cfg.max_angular_velocity);//Clacul a small rotation
			if (vel > cfg.max_velocity) vel = cfg.max_velocity;//Keep a max control on the 'rot' speed and 'translation' speed
	#ifdef DEBUG_GOTOPOSE
			printf("Cmd v %.2f r %.2f\n",vel,rot);
	#endif
			//Publish the velocity
			env->publishVelocity(vel, rot);
		}
		
		
	}else{//SMART 

		//Get the current position of the robot
		const geometry_msgs::Pose2D & tpose = env->getPose2D();
		//Get the distance between the current position and the goal
		double r = hypot(y_init + cfg.goal_y-tpose.y,x_init + cfg.goal_x-tpose.x);
		//Get the angle between the robot current angle and the goal angle
		double alpha = remainder(atan2((y_init + cfg.goal_y-tpose.y),x_init + cfg.goal_x-tpose.x)-tpose.theta,2*M_PI);
		double gamma = cfg.goal_angle-(tpose.theta+alpha);
				
				
		//TASK COMPLETED : if the distance is less than a value fixed by the cfg file
		if (r < cfg.dist_threshold) {
			if(gamma<cfg.gamma_threshold){
				env->publishVelocity(0,0);
				return TaskStatus::TASK_COMPLETED; 
			}
		}else{
			double vel = cfg.k_v * r;//Give a gain 'k-v' to the translation speed and the distance 'r' is a variable
			double rot = std::max(std::min(cfg.k_alpha*alpha+ gamma*cfg.k_gamma,cfg.max_angular_velocity),-cfg.max_angular_velocity);//Clacul a small rotation
			
			if (vel > cfg.max_velocity) vel = cfg.max_velocity;//Keep a max control on the 'rot' speed and 'translation' speed
			env->publishVelocity(vel, rot);
		}
		
		ROS_INFO("Current position r= %.2f y= %.2f theta= %.2f alpha= %.2f gamma= %.2f goal_angle= %.2f",r,tpose.y,tpose.theta,alpha,gamma,cfg.goal_angle);
	}
    
	return TaskStatus::TASK_RUNNING;
}

TaskIndicator TaskGoToPose::terminate()
{
    env->publishVelocity(0,0);
	return TaskStatus::TASK_TERMINATED;
}

DYNAMIC_TASK(TaskFactoryGoToPose);
