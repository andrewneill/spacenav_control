/*  Teleoperation of ABB robot by a 3d mouse
	The idea is, use a 3d mouse to jog a robot, giving 6 axis control

	TODO: 
	1. Add ability to do joint jog
	2. Additional axes (positioner etc)
	3. 
*/

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <moveit/move_group_interface/move_group.h>
#include "geometry_msgs/Point.h"

class JogRobotClass
{
public:
	JogRobot();

private:
	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);

	ros::NodeHandle nh_;
	moveit::planning_interface::MoveGroup arm;

	int linear_, angular_;
	double l_scale_, a_scale_;
	ros::Publisher vel_pub_;
	ros::Subscriber joy_sub_;

};

// bool JogRobotClass::moveToPose(geometry_msgs::Pose goalPose, std::string link){
// 	bool isPlanningSuccess;
// 	bool isMoveSuccess = false;
// 	moveit::planning_interface::MoveGroup::Plan my_plan;
// 	arm.setStartState(*arm.getCurrentState());
// 	arm.setPoseTarget(goalPose, link);

// 	// call the planner to compute the plan and visualize it
// 	isPlanningSuccess = arm.plan(my_plan);

// 	/* *****************  FOR DEBUGGING ***************/
// 	ROS_INFO("Visualizing plan %s", isPlanningSuccess?"":"FAILED");

// 	if(isPlanningSuccess){
// 		isMoveSuccess = arm.execute(my_plan);
// 		ros::Duration(0.5).sleep();
// 	}
// 	return isPlanningSuccess && isMoveSuccess;
// }

JogRobotClass::JogRobot():
linear_(1),
angular_(2),
moveit_config_(3)
{
	nh_.param("axis_linear", linear_, linear_);
	nh_.param("axis_angular", angular_, angular_);
	nh_.param("scale_angular", a_scale_, a_scale_);
	nh_.param("scale_linear", l_scale_, l_scale_);
	nh_.param("movit_config", moviet_config_);

	vel_pub_ = nh_.advertise<industrial::Velocity>("trajectory_msgs/JointTrajectoryPoint", 1); 
	joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &JogRobot::joyCallback, this);

}

void JogRobotClass::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
	bool planSuccess;
	
	//move_group_interface::MoveGroup group("irb1600id_moveit_config");
	move_group_interface::MoveGroup group(moveit_config_);
	arm("manipulator");
	
	//industrial::Velocity vel;
	geometry_msgs::Pose pose = arm.getCurrentPose().cartesianPosition;
	cartesianPosition.position.x += a_scale_*joy->axes[angular_];
	cartesianPosition.position.x += l_scale_*joy->axex[linear_];

	group.setPoseTarget(cartesianPosition);
	planSuccess = arm.plan(thePlan);
	if(isPlanningSuccess)
		arm.execute(thePlan);
	
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "spacenav_control");
	JogRobot spacenav_control;

	ros::spin();
}
















