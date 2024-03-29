#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include <cmath>
#include <tf/transform_datatypes.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"

#define max_dist_to_goal 0.5

class decision {
private:

    ros::NodeHandle n;

    // communication with person_detector or person_tracke
    ros::Publisher pub_goal_reached;
    ros::Subscriber sub_goal_to_reach;

    ros::Publisher pub_person_reached;
    ros::Subscriber sub_person_to_reach;

    // ros::Subscriber sub_on_vertex;
	ros::Subscriber sub_robot_moving;

    // localization
    geometry_msgs::Point robot_coordinates;
    float robot_orientation;

    // communication with rotation_action
    ros::Publisher pub_rotation_to_do;
    ros::Subscriber sub_rotation_done;

    // communication with translation_action
    ros::Publisher pub_translation_to_do;
    ros::Subscriber sub_translation_done;

    bool cond_rotation;//boolean to check if there is a /rotation_to_do
    bool cond_translation;//boolean to check if there is a /translation_to_do

    float rotation_to_do;
    float rotation_done;
    float translation_to_do;
    float translation_done;

    bool new_goal_to_reach;//to check if a new /goal_to_reach is available or not
    bool new_person_to_reach;//to check if a new /person_to_reach is available or not
    bool new_loc;
    bool on_a_vertex;
    bool new_rotation_done;//to check if a new /rotation_done is available or not
    bool new_translation_done;//to check if a new /translation_done is available or not
    geometry_msgs::Point person_to_reach;
    geometry_msgs::Point goal_to_reach;
    geometry_msgs::Point goal_reached;

public:

decision() {

    // communication with localization
    ros::Subscriber sub_amcl = n.subscribe("amcl_pose", 1000, &decision::localizationCallback, this);
    // ros::Subscriber sub_robot_moving;

    // communication with moving_persons_detector or person_tracker
    pub_goal_reached = n.advertise<geometry_msgs::Point>("goal_reached", 1);
    sub_goal_to_reach = n.subscribe("goal_to_reach", 1, &decision::goal_to_reachCallback, this);

    pub_person_reached = n.advertise<geometry_msgs::Point>("person_reached", 1);
    sub_person_to_reach = n.subscribe("person_to_reach", 1, &decision::detection_doneCallback, this);
    // sub_on_vertex = n.subscribe("on_vertex", 1, &decision::on_vertexCallback, this);

    // communication with rotation_action
    pub_rotation_to_do = n.advertise<std_msgs::Float32>("rotation_to_do", 0);
    sub_rotation_done = n.subscribe("rotation_done", 1, &decision::rotation_doneCallback, this);
    cond_rotation = false;

    // communication with translation_action
    pub_translation_to_do = n.advertise<std_msgs::Float32>("translation_to_do", 0);
    sub_translation_done = n.subscribe("translation_done", 1, &decision::translation_doneCallback, this);
    cond_translation = false;

    new_person_to_reach = false;
    new_goal_to_reach = false;
    on_a_vertex = true; // we assume the robot is on a vertex at the beginning
    new_loc = false;
    new_rotation_done = false;
    new_translation_done = false;


    //INFINTE LOOP TO COLLECT LASER DATA AND PROCESS THEM
    ros::Rate r(10);// this node will work at 10hz
    while (ros::ok()) {
        ros::spinOnce();//each callback is called once
        update();
        r.sleep();//we wait if the processing (ie, callback+update) has taken less than 0.1s (ie, 10 hz)
    }

}

//UPDATE: main processing
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void update() {

    // we receive a new /person_to_reach not equal to zero and robair is not doing a translation or a rotation
    if ((new_person_to_reach) && ( !cond_translation ) && ( !cond_rotation ) && (person_to_reach.x != 0.0 && person_to_reach.y != 0.0)) {

        ROS_INFO("(decision_node) /person_to_reach received: (%f, %f)", person_to_reach.x, person_to_reach.y);

        // person_to_reach is already in relative coordinates

        // we have a rotation and a translation to perform
        // we compute the /translation_to_do
        translation_to_do = sqrt( ( person_to_reach.x * person_to_reach.x ) + ( person_to_reach.y * person_to_reach.y ) );

        if ( translation_to_do ) {
            cond_translation = true;

            //we compute the /rotation_to_do
            cond_rotation = true;
            rotation_to_do = acos( person_to_reach.x / translation_to_do );

            if ( person_to_reach.y < 0 )
                rotation_to_do *=-1;

            //we first perform the /rotation_to_do
            ROS_INFO("(decision_node) /rotation_to_do: %f", rotation_to_do*180/M_PI);
            std_msgs::Float32 msg_rotation_to_do;
            //to complete
            msg_rotation_to_do.data = rotation_to_do;
            pub_rotation_to_do.publish(msg_rotation_to_do);
        }
        else {
            geometry_msgs::Point msg_person_reached;
            msg_person_reached.x = 0;
            msg_person_reached.y = 0;

            ROS_INFO("(decision_node) /person_reached (%f, %f)", msg_person_reached.x, msg_person_reached.y);
            pub_person_reached.publish(msg_person_reached);
        }

		// new_person_to_reach = false;
        new_loc = false;
    }

    // DEBUG messages
    if (new_loc) {
        //ROS_INFO("DEBUG MSG: new_lock");
    }
    if (new_goal_to_reach) {
        //ROS_INFO("DEBUG MSG: new_goal_to_reach");
    }

    if ( ( new_loc ) && ( new_goal_to_reach ) && ( !cond_translation ) && ( !cond_rotation ) ) {

        ROS_INFO("(decision_node) /goal_to_reach received: (%f, %f)", goal_to_reach.x, goal_to_reach.y);

        // transforms goal_to_reach to relative coordinates
       goal_to_reach.x -= robot_coordinates.x;
       goal_to_reach.y -= robot_coordinates.y;

        // we have a rotation and a translation to perform
        // we compute the /translation_to_do
        translation_to_do = sqrt( ( goal_to_reach.x * goal_to_reach.x ) + ( goal_to_reach.y * goal_to_reach.y ) );

        if ( translation_to_do ) {

            cond_translation = true;

            //we compute the /rotation_to_do
            cond_rotation = true;
            rotation_to_do = acos( (goal_to_reach.x) / translation_to_do );

            if ( goal_to_reach.y < 0 )
                rotation_to_do *=-1;

            // transforms to relative orientation
            rotation_to_do -= robot_orientation;

            //we first perform the /rotation_to_do
            ROS_INFO("(decision_node) /rotation_to_do: %f", rotation_to_do*180/M_PI);
            std_msgs::Float32 msg_rotation_to_do;
            //to complete
            msg_rotation_to_do.data = rotation_to_do;
            ROS_INFO("msg_rotation_to_do : %f", msg_rotation_to_do.data);
            pub_rotation_to_do.publish(msg_rotation_to_do);

	    }
        else {
            geometry_msgs::Point msg_goal_reached;
            msg_goal_reached.x = goal_to_reach.x;
            msg_goal_reached.y = goal_to_reach.y;

            ROS_INFO("(decision_node) /goal_reached (%f, %f)", msg_goal_reached.x, msg_goal_reached.y);
            pub_goal_reached.publish(msg_goal_reached);
        }

        new_goal_to_reach = false;
        new_loc = false;

    }

    // new_person_to_reach = false;

    //we receive an ack from rotation_action_node. So, we perform the /translation_to_do
    if ( new_rotation_done ) {
        ROS_INFO("(decision_node) /rotation_done : %f", rotation_done*180/M_PI);
        cond_rotation = false;
        new_rotation_done = false;

        //the rotation_to_do is done so we perform the translation_to_do
        ROS_INFO("(decision_node) /translation_to_do: %f", translation_to_do);
        std_msgs::Float32 msg_translation_to_do;
        //to complete
        msg_translation_to_do.data = translation_to_do;
	    pub_translation_to_do.publish(msg_translation_to_do);
    }

    //we receive an ack from translation_action_node. So, we send an ack to the moving_persons_detector_node
    if ( new_translation_done ) {
        ROS_INFO("(decision_node) /translation_done : %f\n", translation_done);
        cond_translation = false;
        new_translation_done = false;

        //the translation_to_do is done so we send the goal_reached to the detector/tracker node
        geometry_msgs::Point msg_goal_reached;
        ROS_INFO("(decision_node) /goal_reached (%f, %f)", msg_goal_reached.x, msg_goal_reached.y);
        //to complete
        // TODO check if the goal has been reached
        pub_goal_reached.publish(msg_goal_reached);

        ROS_INFO(" ");
        ROS_INFO("(decision_node) waiting for a /goal_to_reach");
    }

}// update

//CALLBACKS
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void goal_to_reachCallback(const geometry_msgs::Point::ConstPtr& g) {
// process the goal received from moving_persons detector
	ROS_INFO("goal_to_reachCallback");
    new_goal_to_reach = true;
    goal_to_reach.x = g->x;
    goal_to_reach.y = g->y;
    ROS_INFO("gtr_callb %f %f ", goal_to_reach.x, goal_to_reach.y);

}


void detection_doneCallback(const geometry_msgs::Point::ConstPtr& person) {
    // TODO sub to it
    new_person_to_reach = true;
    person_to_reach.x = person->x;
    person_to_reach.y = person->y;
    // detected_person = person->data;
    ROS_INFO("persontr_callb %f %f ", person_to_reach.x, person_to_reach.y);
}

void localizationCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amcloutput) {
	//ROS_INFO("localizationCallback");
    new_loc = true;
    ROS_INFO("New data of AMCL received");
    // TODO localizationCallback
    robot_coordinates.x = amcloutput->pose.pose.position.x;
    robot_coordinates.y = amcloutput->pose.pose.position.y;
    robot_coordinates.z = 0.0;
    robot_orientation = tf::getYaw(amcloutput->pose.pose.orientation);

    ROS_INFO("Robot_coordinates: (%f, %f)", robot_coordinates.x, robot_coordinates.y);

    return;
}

void rotation_doneCallback(const std_msgs::Float32::ConstPtr& a) {
// process the angle received from the rotation node
	ROS_INFO("rotation_doneCallback");
    new_rotation_done = true;
    rotation_done = a->data;

}

void translation_doneCallback(const std_msgs::Float32::ConstPtr& r) {
// process the range received from the translation node

    new_translation_done = true;
    translation_done = r->data;

}

// Distance between two points
float distancePoints(geometry_msgs::Point pa, geometry_msgs::Point pb) {

    return sqrt(pow((pa.x-pb.x),2.0) + pow((pa.y-pb.y),2.0));

}

};

int main(int argc, char **argv){

    ROS_INFO("(decision_node) waiting for a /goal_to_reach");
    ros::init(argc, argv, "decision");

    decision bsObject;

    ros::spin();

    return 0;
}
