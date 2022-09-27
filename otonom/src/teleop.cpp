#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
float speedConst=0.6;
class RobotTeleop{
    public:
            RobotTeleop();
    private:
            void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
            ros::NodeHandle nh_;
            ros::Subscriber joy_sub_;
            ros::Publisher cmd_vel_pub_;

};

RobotTeleop::RobotTeleop(){
    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy",10,&RobotTeleop::joyCallback,this);
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel",10);

}

void RobotTeleop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy){
    geometry_msgs::Twist vel;
    geometry_msgs::Vector3 vel_linear;
    geometry_msgs::Vector3 vel_angular;
    if(joy->axes[7]==1 && speedConst<1){
        speedConst += 0.2;
    }else if(joy->axes[7]==-1 && speedConst>0.2){
        speedConst -= 0.2;
    }


    if(joy->axes[1] > 0.10 ){
        vel_linear.x =  speedConst;
    }else if( joy->axes[1]< -0.10 ){
         vel_linear.x =  -speedConst;
    }
    else{
        vel_linear.x = 0;
    }
    //vel_linear.x = joy->axes[1];
    vel_linear.y = 0;
    vel_linear.z=0;
    vel_angular.x=0;
    vel_angular.y=0;
    /*
    if(joy->buttons[4]==1 ){
            vel_angular.z=1;
            printf("sola donus -> vel angular = %f \n",vel_angular.z);
    }else if(joy->buttons[5]==1) {
            vel_angular.z=-1;
             printf("saga donus -> vel angular = %f \n",vel_angular.z);
    }else{
            vel_angular.z=0;
            printf("stabil -> vel angular = %f \n",vel_angular.z);

    } */
    vel_angular.z=joy->axes[3];
    printf("speed = %f \n",speedConst);
    vel.linear = vel_linear;
    vel.angular = vel_angular;
    cmd_vel_pub_.publish(vel);
}

int main(int argc , char** argv){
    ros::init(argc,argv, "robot_teleop");
    RobotTeleop rbt_tlp;
    ros::spin();
}