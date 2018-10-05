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
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <vector>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
// #define DEBUG
 #define BOARD_DEBUG
using namespace std;

/*
step 1: wait for position of hole, airplane, take off signal...
step 2: calculate:
    - calculate 3D-plan of the gate
    - calculate point A and point B (point nearest to uav as point A)
step 3: take off and go to p_READY_A position
step 4: pass through
step 5: go to p_READY_B position
step 6: land...
*/
class Planning {
public:
    geometry_msgs::PoseStamped p_READY_A;
    geometry_msgs::PoseStamped p_READY_B;
    geometry_msgs::PoseStamped p_land;
    vector<geometry_msgs::PoseStamped> trajectory;

    Planning(){

    }
    void printinfo() {
        ROS_INFO("planning info:\nuav_start:(%f,%f,%f)\ngate_start:(%f,%f,%f,%f,%f,%f,%f)\npA:(%f,%f,%f)\npB:(%f,%f,%f)\nlanding:(%f,%f,%f)\n",
            p_uav_start.pose.position.x,p_uav_start.pose.position.y,p_uav_start.pose.position.z,
            p_gate_start.pose.position.x,p_gate_start.pose.position.y,p_gate_start.pose.position.z,
            p_gate_start.pose.orientation.x,p_gate_start.pose.orientation.y,p_gate_start.pose.orientation.z,p_gate_start.pose.orientation.w,
            p_READY_A.pose.position.x,p_READY_A.pose.position.y,p_READY_A.pose.position.z,
            p_READY_B.pose.position.x,p_READY_B.pose.position.y,p_READY_B.pose.position.z,
            p_land.pose.position.x,p_land.pose.position.y,p_land.pose.position.z);
    }
    void test() {
    }

    void add_traj(geometry_msgs::PoseStamped from, geometry_msgs::PoseStamped to){

        double dx=to.pose.position.x-from.pose.position.x;
        double dy=to.pose.position.y-from.pose.position.y;
        double dz=to.pose.position.z-from.pose.position.z;
        double dis=sqrt(dx*dx+dy*dy+dz*dz);
        geometry_msgs::PoseStamped tmp;

        for(double s=0;s<=dis;s+=delta) {
            tmp.pose.position.x  = from.pose.position.x + s*dx/dis;
            tmp.pose.position.y  = from.pose.position.y + s*dy/dis;
            tmp.pose.position.z  = from.pose.position.z + s*dz/dis;
            trajectory.push_back(tmp);
        }

    }
    bool is_gate_moving()
    {
        vector<double> v1;
        vector<double> v2;
        //this is a delay. Including plan_init delay
        if(ros::Time::now() - timer < ros::Duration(2)) {
            return true;
        }
        double dis= (p_gate_start.pose.position.x - p_gate_current.pose.position.x)*(p_gate_start.pose.position.x - p_gate_current.pose.position.x) + 
                    (p_gate_start.pose.position.y - p_gate_current.pose.position.y)*(p_gate_start.pose.position.y - p_gate_current.pose.position.y) +
                    (p_gate_start.pose.position.z - p_gate_current.pose.position.z)*(p_gate_start.pose.position.z - p_gate_current.pose.position.z);
        dis = sqrt(dis);
  
        q2plain(p_gate_start,v1);
        q2plain(p_gate_current,v2);

        double cos_t=v1[0]*v2[0]+v1[1]*v2[1]+v1[2]*v2[2];
        cos_t/=(sqrt(v1[0]*v1[0]+v1[1]*v1[1]+v1[2]*v1[2]))*(sqrt(v2[0]*v2[0]+v2[1]*v2[1]+v2[2]*v2[2]));

        if(dis>0.1 || cos_t<0.9848) {//+-10 degree
            filter++;        
        }
        else {
            filter=0;
        }

        if(filter>3) {
            filter=4;
            timer=ros::Time::now();
            return true;  
        }

        return false;
    }
    void plan_reset() {
        init=0;
    }
    void plan_init(geometry_msgs::PoseStamped g, geometry_msgs::PoseStamped u){
        trajectory.clear();
        p_uav_current=u;
        p_gate_current=g;
        p_uav_last=u;
        p_gate_last=g;
        p_uav_start=u;
        p_gate_start=g;
        plan_points();
        timer=ros::Time::now();
        crossed=0;
    }
    bool cross_checker() {
        double res1= a*(p_uav_current.pose.position.x)+
                     b*(p_uav_current.pose.position.y)+
                     c*(p_uav_current.pose.position.z)+
                     d;
        double res2= a*(p_uav_last.pose.position.x)+
                     b*(p_uav_last.pose.position.y)+
                     c*(p_uav_last.pose.position.z)+
                     d;
        if(res*res2<=0) 
            return true;
        return false;
    }
    void plan_cross_path() {
        add_traj(p_uav_current,p_READY_A);
        add_traj(p_READY_A, p_READY_B);      
    }
    void plan_landing_path() {
        add_traj(p_READY_B, p_land);
    }


    bool plan_traj(geometry_msgs::PoseStamped g, geometry_msgs::PoseStamped u){

        bool ret=true; //false: a path is regenerated... //true: path is ready...

        p_uav_current=u;
        p_gate_current=g;

        if(!init) {
            plan_init(g,u);
            init=1;
            ret = false;
        }

        if(is_gate_moving()) {
            if(!crossed) {
                plan_init(g,u);
                ret = false;
            }
            else {}//continue the action
            
            planned=0;
        }
        else {
            cross_checker();
            if (!planned) {
                if (crossed) {
                    plan_landing_path();
                }
                else {
                    plan_cross_path();    
                    plan_landing_path();       
                }
                planned=1;
            }
        
        }
        p_gate_last=p_gate_current;
        p_uav_last=p_uav_current;

        return ret;


    }


    void plan_points(){

        double x=p_uav_current.pose.position.x;
        double y=p_uav_current.pose.position.y;
        double z=p_uav_current.pose.position.z;
        double p_to_plain_dis=fabs(a*x+b*y+c*z+d)/sqrt(a*a+b*b+c*c);   
        //vector (a,b,c) is perpendicular to this plain...
        //so we get point p_READY_A and p_READY_B.

        p_READY_A.pose.position.x=p_gate_current.pose.position.x+cross_len*(a/(sqrt(a*a+b*b+c*c)));
        p_READY_A.pose.position.y=p_gate_current.pose.position.y+cross_len*(b/(sqrt(a*a+b*b+c*c)));
        p_READY_A.pose.position.z=p_gate_current.pose.position.z+cross_len*(c/(sqrt(a*a+b*b+c*c)));

        p_READY_B.pose.position.x=p_gate_current.pose.position.x-cross_len*(a/(sqrt(a*a+b*b+c*c)));
        p_READY_B.pose.position.y=p_gate_current.pose.position.y-cross_len*(b/(sqrt(a*a+b*b+c*c)));
        p_READY_B.pose.position.z=p_gate_current.pose.position.z-cross_len*(c/(sqrt(a*a+b*b+c*c)));


        //judge whether these two points belong to the same side of the partition by this plain.
        double s_readyA=a*p_READY_A.pose.position.x+
                        b*p_READY_A.pose.position.y+
                        c*p_READY_A.pose.position.z+
                        d;

        double s_uav=a*x+b*y+c*z+d;

        if (s_uav*s_readyA<0) {
            swap<geometry_msgs::PoseStamped>(p_READY_A,p_READY_B);

            if(p_to_plain_dis<cross_len/2.0) {
                p_READY_A.pose.position.x=p_gate_current.pose.position.x-max(0.1,p_to_plain_dis)*(a/(sqrt(a*a+b*b+c*c)));
                p_READY_A.pose.position.y=p_gate_current.pose.position.y-max(0.1,p_to_plain_dis)*(b/(sqrt(a*a+b*b+c*c)));
                p_READY_A.pose.position.z=p_gate_current.pose.position.z-max(0.1,p_to_plain_dis)*(c/(sqrt(a*a+b*b+c*c)));
            }

            p_land.pose.position.x=p_gate_current.pose.position.x+(land_len+cross_len)*(a/(sqrt(a*a+b*b+c*c)));
            p_land.pose.position.y=p_gate_current.pose.position.y+(land_len+cross_len)*(b/(sqrt(a*a+b*b+c*c)));
            p_land.pose.position.z=p_gate_current.pose.position.z+(land_len+cross_len)*(c/(sqrt(a*a+b*b+c*c)));
        }
        else{

            if(p_to_plain_dis<cross_len/2.0) {
                p_READY_A.pose.position.x=p_gate_current.pose.position.x+max(0.1,p_to_plain_dis)*(a/(sqrt(a*a+b*b+c*c)));
                p_READY_A.pose.position.y=p_gate_current.pose.position.y+max(0.1,p_to_plain_dis)*(b/(sqrt(a*a+b*b+c*c)));
                p_READY_A.pose.position.z=p_gate_current.pose.position.z+max(0.1,p_to_plain_dis)*(c/(sqrt(a*a+b*b+c*c)));
            }

            p_land.pose.position.x=p_gate_current.pose.position.x-(land_len+cross_len)*(a/(sqrt(a*a+b*b+c*c)));
            p_land.pose.position.y=p_gate_current.pose.position.y-(land_len+cross_len)*(b/(sqrt(a*a+b*b+c*c)));
            p_land.pose.position.z=p_gate_current.pose.position.z-(land_len+cross_len)*(c/(sqrt(a*a+b*b+c*c)));          
        }

        vector<double> v;
        q2plain(p_gate_current,v);
        a=v[0];
        b=v[1];
        c=v[2];
        d=v[3];


        double cross_len_a=cross_len/2.0;
        double cross_len_b=cross_len/2.0;


        if(p_to_plain_dis<cross_len/2.0) {
            cross_len_a=min(p_to_plain_dis,0.1);
        }      

         double s_uav=a*x+b*y+c*z+d;
         if(s_uav>0) {

         }     


    }

    
protected:

    void q2plain(geometry_msgs::PoseStamped p, vector<double> &v){

        double aa,bb,cc,dd;
        Eigen::Quaterniond q(p.pose.orientation.w,p.pose.orientation.x,p.pose.orientation.y,p.pose.orientation.z);
        q.normalize(); 

        Eigen::Vector3d v(1, 0, 0); 
        Eigen::Quaterniond eig_v; 
        eig_v.w() = 0; 
        eig_v.vec() = v; 

        Eigen::Quaterniond rotatedP=q*eig_v*q.inverse();
        Eigen::Vector3d rotatedV = rotatedP.vec();

        aa=rotatedV.x();
        bb=rotatedV.y();
        cc=rotatedV.z();
        dd=0-aa*p.pose.position.x-bb*p.pose.position.y-cc*p.pose.position.z;
        v.push_back(aa);
        v.push_back(bb);
        v.push_back(cc);
        v.push_back(dd);
        //ROS_INFO("%f*x+%f*y+%f*z+%f=0",aa,bb,cc,dd);
    }
private:
    
    double a,b,c,d;//ax+by+cz+d=0
    double cross_len=0.6;
    double land_len=0.3;

    
    double delta=0.025;//1m/s hard code..
    int planned=0;
    int init=0;
    int filter=0;

    ros::Time timer;

    geometry_msgs::PoseStamped p_gate_start;
    geometry_msgs::PoseStamped p_uav_start;

    geometry_msgs::PoseStamped p_gate_current;
    geometry_msgs::PoseStamped p_uav_current;

    geometry_msgs::PoseStamped p_gate_last;
    geometry_msgs::PoseStamped p_uav_last;   

};




mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_location;
geometry_msgs::PoseStamped global_pose_gate;
int check_conn_state=0;
int check_conn_pose=0;
int check_conn_gatepose=0;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
    check_conn_state=1;
}

void location_cb(const geometry_msgs::PoseStamped::ConstPtr& pose) {
    current_location=*pose;
    check_conn_pose=1;
}
void location_gate_cb(const geometry_msgs::TransformStamped::ConstPtr& pose) {
    geometry_msgs::TransformStamped tmp=*pose;
    global_pose_gate.header=tmp.header;
    global_pose_gate.pose.position.x=tmp.transform.translation.x;
    global_pose_gate.pose.position.y=tmp.transform.translation.y;
    global_pose_gate.pose.position.z=tmp.transform.translation.z;
    global_pose_gate.pose.orientation.x=tmp.transform.rotation.x;
    global_pose_gate.pose.orientation.y=tmp.transform.rotation.y;
    global_pose_gate.pose.orientation.z=tmp.transform.rotation.z;
    global_pose_gate.pose.orientation.w=tmp.transform.rotation.w;
    check_conn_gatepose=1;
    #ifdef BOARD_DEBUG
    global_pose_gate.pose.position.z=0.8;
    #endif
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
    int state_mach=0;
    Planning planning;
    planning.test();

    double rosrate=40.0;//hard code..

    ros::init(argc, argv, "hole_node");
    //the setpoint publishing rate MUST be faster than 2Hz
    
    //=================topics=================
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>
            ("/mavros/cmd/land");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");
    ros::Subscriber loc_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10,location_cb);
            #ifdef BOARD_DEBUG
    ros::Subscriber vicon_gate_sub = nh.subscribe<geometry_msgs::TransformStamped>
            ("vicon/board1/board1", 10,location_gate_cb);
            #else
    ros::Subscriber vicon_gate_sub = nh.subscribe<geometry_msgs::TransformStamped>
            ("vicon/gate/gate", 10,location_gate_cb);            
            #endif
    //=================params=================
    ros::NodeHandle param_n;
    ros::Rate rate(rosrate);
    // param_n.param<double>("/infoday/width", width, 2);
    // param_n.param<double>("/infoday/length", length, 2);
    // param_n.param<double>("/infoday/height", height, 1);
    // param_n.param<double>("/infoday/period", period, 30);
    // ROS_INFO("(params) width:%f,length:%f,height:%f,period:%f",width,length,height,period);


    //=================observation by rviz=================
    ros::NodeHandle ph;
    ros::Publisher givenpath_pub = ph.advertise<nav_msgs::Path>("given_trajectory",1, true);
    ros::Publisher actualpath_pub = ph.advertise<nav_msgs::Path>("actual_trajectory",1, true);
    ros::Publisher realtimepath_pub = ph.advertise<nav_msgs::Path>("realtime_trajectory",1, true);
    //see: rosmsg show nav_msgs/Path
    nav_msgs::Path givenpath;
    nav_msgs::Path actualpath;
    nav_msgs::Path realtimepath;

    int flight_tick=0;



    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    //===============initialize===================


    tf::TransformBroadcaster broadcaster;
    tf::TransformListener listener(ros::Duration(10));
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
    geometry_msgs::PoseStamped takeoff_location;
    geometry_msgs::PoseStamped land_location;
    geometry_msgs::PoseStamped offset;
    geometry_msgs::PoseStamped traj;
    geometry_msgs::PoseStamped realtime;
    geometry_msgs::PoseStamped tmp;
    geometry_msgs::Quaternion tmp_q;
    mavros_msgs::CommandTOL srv_land;
    srv_land.request.altitude = 1;
    srv_land.request.latitude = 0;
    srv_land.request.longitude = 0;
    srv_land.request.min_pitch = 0;
    srv_land.request.yaw = 0;

    mavros_msgs::State last_state=current_state;
    int fly_count=0;
    ros::Time sendtick1 = ros::Time::now();
    ros::Time sendtick2 = ros::Time::now();
    ros::Time tick_st;
    // delay slightly, to make tf work better..
    for(int i=0;i<30;i++) {
                broadcaster.sendTransform(
                tf::StampedTransform(
                    tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, 0)),
                    ros::time(0),"map", "world")
                );
                ros::spinOnce();
                rate.sleep();
    }



    while(ros::ok()){
        broadcaster.sendTransform(
        tf::StampedTransform(
            tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0.0, 0)),
            ros::time(0),"map", "world")
        );

        
        geometry_msgs::PoseStamped p_gate;
        try{
            listener.transformPose("map", global_pose_gate, p_gate);
            // cout<<p_gate<<endl;
        }
        catch(tf::TransformException& ex){
            ROS_ERROR("Received an exception trying to transform a point from \"world\" to \"map\": %s", ex.what());
            // cout<<p_gate;
            //when this happens, all data is zero
            continue;
        }

        if (state_mach!=0 &&state_mach!=1 && state_mach!=99 & state_mach!=100) {
            if (current_state.armed==0 || current_state.mode != "OFFBOARD") {
                
                ROS_WARN("FORCE LANDING TRIGGERED!!!!!state_mach:%d, armed:%d, mode:%s",state_mach,current_state.armed,current_state.mode.c_str());
                state_mach=99;
            }
        }

        switch(state_mach) {
            case 0:
                takeoff_location=current_location;
                takeoff_location.pose.position.z=1;
                local_pos_pub.publish(takeoff_location);
                if (current_state.armed!=0 &&
                    current_state.mode == "OFFBOARD" &&
                    check_conn_state!=0 &&
                    check_conn_pose !=0 &&
                    check_conn_gatepose !=0
                    ) {                 
                    tick_st=ros::Time::now();

                    ROS_INFO("Step0->1: all signal ready!");
                    planning.printinfo();
                    state_mach++;
                }
                else {
                    ROS_INFO_THROTTLE(1, "(step0) ARMED:%d, MODE:%s, connection{state:%d, uavpose:%d gatepos:%d}",
                                    current_state.armed,
                                    current_state.mode.c_str(),
                                    check_conn_state,
                                    check_conn_pose,
                                    check_conn_gatepose
                                    );                    
                }
                break;
            case 1:
                if(ros::Time::now()-tick_st<ros::Duration(10)) { 
                    #ifndef DEBUG
                    local_pos_pub.publish(takeoff_location);
                    #endif
                    ROS_INFO_THROTTLE(1,"(step1): taking off...");
                }
                else {
                    state_mach++;
                    flight_tick=0;
                    planning.plan_reset();
                    tick_st=ros::Time::now();
                    ROS_INFO("Step1->2: uav has taken off!");
                }
                break;
            case 2:
                if(! planning.plan_traj(global_pose_gate,current_location)) {//path is not ready yet..
                    #ifndef DEBUG
                    local_pos_pub.publish(current_location); //hover...
                    #endif 
                    flight_tick=0;
                }  
                else {
                    if(flight_tick<plan_traj.trajectory.size()) {
                        #ifndef DEBUG
                        local_pos_pub.publish(plan_traj.trajectory[flight_tick]);
                        #endif            
                        flight_tick++;        
                    }
                    else {
                        state_mach=99;
                        ROS_INFO("flight_tick:%d",flight_tick);
         