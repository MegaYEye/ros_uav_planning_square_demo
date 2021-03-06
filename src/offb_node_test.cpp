/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <math.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <vector>
using namespace std;

class Planning {
public:
	Planning(){idx=0;}
	void test_curve() {
		vector<geometry_msgs::PoseStamped> out;
		ROS_INFO("----TEST SIGMOID-----");
		gen_sigmoid(5, 10,10,out,-1);
		for (auto p:out) {
			ROS_INFO("(%f,%f)",
								p.pose.position.x,
								p.pose.position.y);
		}
		vector<geometry_msgs::PoseStamped> out2;
		ROS_INFO("----TEST HALFSIGMOID-----");
		gen_halfsigmoid(5, 10,10,out2,-1);
		for (auto p:out2) {
			ROS_INFO("(%f,%f)",
								p.pose.position.x,
								p.pose.position.y);
		}
	}
	void test_genpath(){
		path_init_corner(20,5,10,1,20);
		for(auto p:path_points) {
			ROS_INFO("(%f,%f,%f)",p.pose.position.x,p.pose.position.y,p.pose.position.z);
		}

	}
	void path_init_corner(double time,double length,double width,double height,double ros_freq){
		idx=0;
		path_points.clear();
		int point_length=(int) (time*ros_freq*length*0.5/(length+width));
		int point_width=(int) (time*ros_freq*width*0.5/(length+width));
		geometry_msgs::PoseStamped tmp;
		tmp.pose.position.x=-length/2.0;
		tmp.pose.position.y=-width/2.0;
		tmp.pose.position.z=height;//hard code...
		ROS_INFO("length point is %d", point_length);
		ROS_INFO("width point is %d", point_width);

		vector<geometry_msgs::PoseStamped> out;
		

	    ROS_INFO("Generate point...");
	    gen_sigmoid(-length/2.0, length,point_length,out,1.0);
		for(int i=0;i<point_length;i++) {//x:-L/2 -> L/2, y:-w/2
			tmp.pose.position.y=-width/2.0;
			tmp.pose.position.x=length/2.0;
			path_points.push_back(tmp);
		}
		gen_sigmoid(-width/2.0, width,point_width,out,1.0);
		for(int i=0;i<point_width;i++) {//x: L/2, y:-w/2 -> w/2
			tmp.pose.position.x=length/2.0;
			tmp.pose.position.y=width/2.0;
			path_points.push_back(tmp);
		}
		gen_sigmoid(length/2.0, length,point_length,out,-1.0);
		for(int i=0;i<point_length;i++) {//x: L/2 -> -L/2, y:W/2 
			tmp.pose.position.y=width/2.0;
			tmp.pose.position.x=-length/2.0;
			path_points.push_back(tmp);
		}
		gen_sigmoid(width/2.0, width,point_width,out,-1.0);
		for(int i=0;i<point_width;i++) {//x: -L/2, y:W/2 -> -W/2
			tmp.pose.position.x=-length/2.0;
			tmp.pose.position.y=-width/2.0;
			path_points.push_back(tmp);
		}
	}
	void path_init_ratio(double time,double length,double width,double height,double ros_freq){
		idx=0;
		path_points.clear();
		int point_length=(int) (time*ros_freq*length*0.5/(length+width));
		int point_width=(int) (time*ros_freq*width*0.5/(length+width));
		geometry_msgs::PoseStamped tmp;
		tmp.pose.position.x=-length/2.0;
		tmp.pose.position.y=-width/2.0;
		tmp.pose.position.z=height;//hard code...
		ROS_INFO("length point is %d", point_length);
		ROS_INFO("width point is %d", point_width);


	    ROS_INFO("Generate point...");
	    double ratio=0.95;
	    double left_len,start,forward_dis;

	    left_len=length;
	    start=-length/2.0;
		for(int i=0;i<point_length;i++) {
			tmp.pose.position.x=start;
			path_points.push_back(tmp);

			forward_dis=left_len*(1-ratio);
			start=start+forward_dis;
			left_len=length/2.0-start;

		}

	    left_len=width;
	    start=-width/2.0;
		for(int i=0;i<point_width;i++) {
			tmp.pose.position.y=start;
			path_points.push_back(tmp);

			forward_dis=left_len*(1-ratio);
			start=start+forward_dis;
			left_len=width/2.0-start;

		}

	    left_len=length;
	    start=length/2.0;
		for(int i=0;i<point_length;i++) {
			tmp.pose.position.x=start;
			path_points.push_back(tmp);

			forward_dis=left_len*(1-ratio);
			start=start-forward_dis;
			left_len=start+length/2.0;
			
		}

	    left_len=width;
	    start=width/2.0;
		for(int i=0;i<point_width;i++) {
			tmp.pose.position.y=start;
			path_points.push_back(tmp);

			forward_dis=left_len*(1-ratio);
			start=start-forward_dis;
			left_len=width/2.0+start;
		}

	}
	void path_init_sigmoid(double time,double length,double width,double height,double ros_freq){
		idx=0;
		path_points.clear();
		int point_length=(int) (time*ros_freq*length*0.5/(length+width));
		int point_width=(int) (time*ros_freq*width*0.5/(length+width));
		geometry_msgs::PoseStamped tmp;
		tmp.pose.position.x=-length/2.0;
		tmp.pose.position.y=-width/2.0;
		tmp.pose.position.z=height;//hard code...
		ROS_INFO("length point is %d", point_length);
		ROS_INFO("width point is %d", point_width);

		vector<geometry_msgs::PoseStamped> out;
		

	    ROS_INFO("Generate point...");
	    gen_sigmoid(-length/2.0, length,point_length,out,1.0);
		for(int i=0;i<point_length;i++) {//x:-L/2 -> L/2, y:-w/2
			tmp.pose.position.y=-width/2.0;
			tmp.pose.position.x=out[i].pose.position.y;
			path_points.push_back(tmp);
		}
		gen_sigmoid(-width/2.0, width,point_width,out,1.0);
		for(int i=0;i<point_width;i++) {//x: L/2, y:-w/2 -> w/2
			tmp.pose.position.x=length/2.0;
			tmp.pose.position.y=out[i].pose.position.y;
			path_points.push_back(tmp);
		}
		gen_sigmoid(length/2.0, length,point_length,out,-1.0);
		for(int i=0;i<point_length;i++) {//x: L/2 -> -L/2, y:W/2 
			tmp.pose.position.y=width/2.0;
			tmp.pose.position.x=out[i].pose.position.y;
			path_points.push_back(tmp);
		}
		gen_sigmoid(width/2.0, width,point_width,out,-1.0);
		for(int i=0;i<point_width;i++) {//x: -L/2, y:W/2 -> -W/2
			tmp.pose.position.x=-length/2.0;
			tmp.pose.position.y=out[i].pose.position.y;
			path_points.push_back(tmp);
		}

	}
	void path_init_halfsigmoid(double time,double length,double width,double height,double ros_freq){
		idx=0;
		path_points.clear();
		int point_length=(int) (time*ros_freq*length*0.5/(length+width));
		int point_width=(int) (time*ros_freq*width*0.5/(length+width));
		geometry_msgs::PoseStamped tmp;
		tmp.pose.position.x=-length/2.0;
		tmp.pose.position.y=-width/2.0;
		tmp.pose.position.z=height;//hard code...
		ROS_INFO("length point is %d", point_length);
		ROS_INFO("width point is %d", point_width);

		vector<geometry_msgs::PoseStamped> out;
		

	    ROS_INFO("Generate point...");
	    gen_halfsigmoid(-length/2.0, length,point_length,out,1.0);
		for(int i=0;i<point_length;i++) {//x:-L/2 -> L/2, y:-w/2
			tmp.pose.position.y=-width/2.0;
			tmp.pose.position.x=out[i].pose.position.y;
			path_points.push_back(tmp);
		}
		gen_halfsigmoid(-width/2.0, width,point_width,out,1.0);
		for(int i=0;i<point_width;i++) {//x: L/2, y:-w/2 -> w/2
			tmp.pose.position.x=length/2.0;
			tmp.pose.position.y=out[i].pose.position.y;
			path_points.push_back(tmp);
		}
		gen_halfsigmoid(length/2.0, length,point_length,out,-1.0);
		for(int i=0;i<point_length;i++) {//x: L/2 -> -L/2, y:W/2 
			tmp.pose.position.y=width/2.0;
			tmp.pose.position.x=out[i].pose.position.y;
			path_points.push_back(tmp);
		}
		gen_halfsigmoid(width/2.0, width,point_width,out,-1.0);
		for(int i=0;i<point_width;i++) {//x: -L/2, y:W/2 -> -W/2
			tmp.pose.position.x=-length/2.0;
			tmp.pose.position.y=out[i].pose.position.y;
			path_points.push_back(tmp);
		}

	}
	void path_init_average(double time,double length,double width,double height,double ros_freq){
		idx=0;
		path_points.clear();
		int point_length=(int) (time*ros_freq*length*0.5/(length+width));
		int point_width=(int) (time*ros_freq*width*0.5/(length+width));
		geometry_msgs::PoseStamped tmp;
		tmp.pose.position.x=-length/2.0;
		tmp.pose.position.y=-width/2.0;
		tmp.pose.position.z=height;//hard code...
		ROS_INFO("length point is %d", point_length);
		ROS_INFO("width point is %d", point_width);


	    ROS_INFO("Generate point...");

		for(int i=0;i<point_length;i++) {
			tmp.pose.position.x=-length/2.0 + 1.0*i*length/point_length;
			path_points.push_back(tmp);
		}
		for(int i=0;i<point_width;i++) {
			tmp.pose.position.y=-width/2.0 + 1.0*i*width/point_width;
			path_points.push_back(tmp);
		}
		for(int i=0;i<point_length;i++) {
			tmp.pose.position.x=length/2.0 - 1.0*i*length/point_length;
			path_points.push_back(tmp);
		}
		for(int i=0;i<point_width;i++) {
			tmp.pose.position.y=width/2.0 - 1.0*i*width/point_width;
			path_points.push_back(tmp);
		}

	}
	geometry_msgs::PoseStamped next_point() {
		int p=idx%path_points.size();
		idx = idx+1;
		return path_points.at(p);
	}
protected:
	void gen_sigmoid(double start, double range,int point_num,vector<geometry_msgs::PoseStamped> &output,double positive) {
		int cnt=0;
		output.clear();
		for(double x=-5.0;cnt++<point_num;x+=10.0/point_num) {
			double y=start+positive*range/(1+exp(-x));
			geometry_msgs::PoseStamped tmp;
			tmp.pose.position.x=x/10.0+0.5;
			tmp.pose.position.y=y;
			tmp.pose.position.z=0;
			output.push_back(tmp);
		}
	}
	void gen_halfsigmoid(double start, double range,int point_num,vector<geometry_msgs::PoseStamped> &output,double positive) {
		int cnt=0;
		output.clear();
		for(double x=0;cnt++<point_num;x+=5.0/point_num) {
			double y=start+positive*range*(-0.5+1/(1+exp(-x)))*2;
			geometry_msgs::PoseStamped tmp;
			tmp.pose.position.x=x/5.0;
			tmp.pose.position.y=y;
			tmp.pose.position.z=0;
			output.push_back(tmp);
		}
	}
private:

	vector<geometry_msgs::PoseStamped> path_points; 
	int idx;
};




mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_location;

int check_conn_state=0;
int check_conn_pose=0;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
    check_conn_state=1;
}

void location_cb(const geometry_msgs::PoseStamped::ConstPtr& pose) {
	current_location=*pose;
    check_conn_pose=1;
}


geometry_msgs::PoseStamped pose_add(geometry_msgs::PoseStamped & a,geometry_msgs::PoseStamped & b) {
	geometry_msgs::PoseStamped tmp;
	tmp.pose.position.x=a.pose.position.x+b.pose.position.x;
	tmp.pose.position.y=a.pose.position.y+b.pose.position.y;
	tmp.pose.position.z=a.pose.position.z+b.pose.position.z;
	return tmp;
}


int main(int argc, char **argv)
{
	double width,length,height,period;
	int state_mach=0;
	Planning planning;

	double rosrate=40.0;//hard code..

    ros::init(argc, argv, "offb_node");
    planning.test_genpath();
    //=================topics=================
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 1, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::Subscriber loc_sub = nh.subscribe<geometry_msgs::PoseStamped>
    		("mavros/local_position/pose", 10,location_cb);

   	//=================params=================
    ros::NodeHandle param_n;

    param_n.param<double>("/infoday/width", width, 2);
    param_n.param<double>("/infoday/length", length, 2);
    param_n.param<double>("/infoday/height", height, 1);
    param_n.param<double>("/infoday/period", period, 30);
    ROS_INFO("(params) width:%f,length:%f,height:%f,period:%f",width,length,height,period);



    //=================observation by rviz=================
    ros::NodeHandle ph;
    ros::Publisher givenpath_pub = ph.advertise<nav_msgs::Path>("given_trajectory",1, true);
    ros::Publisher actualpath_pub = ph.advertise<nav_msgs::Path>("actual_trajectory",1, true);
    ros::Publisher realtimepath_pub = ph.advertise<nav_msgs::Path>("realtime_trajectory",1, true);
    //see: rosmsg show nav_msgs/Path
  	nav_msgs::Path givenpath;
  	nav_msgs::Path actualpath;
  	nav_msgs::Path realtimepath;
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(rosrate);


    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    //===============initialize===================
    geometry_msgs::PoseStamped switch2offboard;
    switch2offboard.pose.position.x = 0;
    switch2offboard.pose.position.y = 0;
    switch2offboard.pose.position.z = height;

    //send a few setpoints before starting
    // for(int i = 300; ros::ok() && i > 0; --i){
    //     local_pos_pub.publish(pose);
    //     ros::spinOnce();
    //     rate.sleep();
    // }

    // mavros_msgs::SetMode offb_set_mode;
    // offb_set_mode.request.custom_mode = "OFFBOARD";

    // mavros_msgs::CommandBool arm_cmd;
    // arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    geometry_msgs::PoseStamped tar;
    geometry_msgs::PoseStamped takeoff;
    geometry_msgs::PoseStamped offset;
    geometry_msgs::PoseStamped traj;
    geometry_msgs::PoseStamped realtime;
    geometry_msgs::PoseStamped tmp;

    mavros_msgs::State last_state=current_state;
    int fly_count=0;
    ros::Time sendtick1 = ros::Time::now();
    ros::Time sendtick2 = ros::Time::now();

    while(ros::ok()){
    	
    	realtime=current_location;	
		realtimepath.poses.push_back(current_location); 	
		
		if(realtimepath.poses.size()>1000) realtimepath.poses.erase(realtimepath.poses.begin());
        if(ros::Time::now()-sendtick1>ros::Duration(0.5)) {
            realtimepath.header.stamp=ros::Time::now();;
            realtimepath.header.frame_id="map";
            realtimepath_pub.publish(realtimepath);     
            sendtick1=ros::Time::now();
        }

		//reinitize when switch to manual control...
    	if (last_state.armed!=current_state.armed ||last_state.mode!=current_state.mode) {
    		ROS_INFO("state changed: armed: %d,mode:%s",current_state.armed==true,current_state.mode.c_str());
    	}
    	if ( 	(last_state.armed==true && current_state.armed==false) || 
    			(last_state.mode=="OFFBOARD" && current_state.mode!="OFFBOARD")
    		)
    	{
    		ROS_INFO("control restart, waiting signal...");
    		state_mach=0;
    	} 
    	switch(state_mach) {
    		case 0:
    			//we must start from disarm!
                if (check_conn_pose==0) ROS_WARN("no position data!");
                if (check_conn_state==0) ROS_WARN("no state data!");
    			if (current_state.armed==false &&
    				current_state.mode != "OFFBOARD") { 				
    				ROS_INFO("Step0 OK:WAITING...");
    				state_mach=1;
    			}
    			break;
    		case 1://then, wait the OFFBOARD command from the joystick.
    			offset=current_location;
    			tmp=pose_add(switch2offboard,offset);
    			local_pos_pub.publish(tmp);//need some prerequisite command before switching mode..
    			if (current_state.armed==true &&
    				current_state.mode == "OFFBOARD" ) {
    				ROS_INFO("Step1 OK:ARMED and OFFBOARD received! Go to start point...");
    				planning.path_init_corner(period,length,width,height,rosrate);
					givenpath.poses.clear();
					actualpath.poses.clear();
					fly_count=0;
    				state_mach=2;
    			}
    			break;
    		case 2://we have 30 secs to fly to starting point... (30 is an arbitary value)
				
				tar.pose.position.x=-length/2.0;
				tar.pose.position.y=-width/2.0;
				tar.pose.position.z=height;
				
				takeoff.pose.position.x=0;
				takeoff.pose.position.y=0;
				takeoff.pose.position.z=height;

				//10 secs to take off
				if (fly_count<10*rosrate) {
					tmp=pose_add(takeoff,offset);
				   	local_pos_pub.publish(tmp);
				   	ROS_INFO_THROTTLE(1, "(takeoff)time:%lf,tarpos:(%lf,%lf,%lf),actualpos:(%lf,%lf,%lf)"
				   						,fly_count/rosrate
				   						,tmp.pose.position.x
				   						,tmp.pose.position.y
				   						,tmp.pose.position.z
				   						,current_location.pose.position.x
				   						,current_location.pose.position.y
				   						,current_location.pose.position.z);	
				   					
				}
				else if (fly_count<30*rosrate) {
					tmp=pose_add(tar,offset);
				   	local_pos_pub.publish(tmp);
				   	ROS_INFO_THROTTLE(1, "(move)time:%lf,tarpos:(%lf,%lf,%lf),pos:(%lf,%lf,%lf)"
				   						,fly_count/rosrate
				   						,tmp.pose.position.x
				   						,tmp.pose.position.y
				   						,tmp.pose.position.z
				   						,current_location.pose.position.x
				   						,current_location.pose.position.y
				   						,current_location.pose.position.z);
				   					
				}
				else {
					state_mach=3;
				}
				fly_count++;
				
    			break;
    		case 3://follow the trajectory....
    			
				traj=planning.next_point();
				tmp=pose_add(traj,offset);
				local_pos_pub.publish(tmp);
				ROS_INFO_THROTTLE(0.5, "target:(%lf,%lf,%lf),actual:(%lf,%lf,%lf)"
									,tmp.pose.position.x
									,tmp.pose.position.y
									,tmp.pose.position.z
									,current_location.pose.position.x
									,current_location.pose.position.y
									,current_location.pose.position.z);



				//no use, just for rviz...
				givenpath.poses.push_back(tmp); 
				actualpath.poses.push_back(current_location); 	
				//limit the bandwidth			
				if(givenpath.poses.size()>1000) givenpath.poses.erase(givenpath.poses.begin());
				if(actualpath.poses.size()>1000) actualpath.poses.erase(actualpath.poses.begin());
	            if(ros::Time::now()-sendtick2>ros::Duration(0.5)) {
                    givenpath.header.stamp=ros::Time::now();;
                    givenpath.header.frame_id="map";
                    givenpath_pub.publish(givenpath); 
                    actualpath.header.stamp=ros::Time::now();;
                    actualpath.header.frame_id="map";
                    actualpath_pub.publish(actualpath); 
                    sendtick2=ros::Time::now();
                }

    			break;
    		default:
    			ROS_INFO("erroneous state_machine index!");


    	}
    	last_state=current_state;
        ros::spinOnce();
        rate.sleep();
        
    }


   

    return 0;
}

