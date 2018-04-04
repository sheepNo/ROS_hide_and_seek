// next vertex choice using localization and robbot_moving data

#include "ros/ros.h"
#include "ros/time.h"
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/ColorRGBA.h"
#include <cmath>
#include "std_msgs/Bool.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

// used for on_vertex decision
#define max_dist_to_goal 0.5

class next_vertex_choice {

private:
    ros::NodeHandle n;

    ros::Subscriber sub_amcl = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", localizationCallback);
    ros::Subscriber sub_robot_moving;

    ros::Publisher pub_next_vertex;
    ros::Publisher pub_next_vertex_marker;

    // will be useful later for the decision node
    // ros::Publisher pub_on_vertex; TODO later for detection of persons once the robot is on a vertex

    // list of vertices we have to visit
    geometry_msgs::Point vertices_list[1000];
    int nb_vertices; // nb of vertices in the vertices_list list
    int current_vertex; // index of the current/previous vertex
    int next_vertex;

    bool on_a_vertex;

    // coordinates of the robot
    geometry_msgs::Point robot_coordinates;
    float robot_orientation;

    // geometry_msgs::Point next_vertex_coordinates;
    geometry_msgs::Point goal_to_reach;

    // graphical display
    int nb_pts; // DONE: the points need to display
    geometry_msgs::Point display[2000];
    std_msgs::ColorRGBA colors[2000];

    // to check if the robot is moving or not
    bool previous_robot_moving;
    bool current_robot_moving;
    bool new_robot; // new data from robot_moving

public:

next_vertex_choice() {

    sub_robot_moving = n.subscribe("robot_moving", 1, &next_vertex::robot_movingCallback, this);
    sub_localization = n.subscribe("amcl", 1, &next_vertex::localizationCallback, this);

    pub_next_vertex_marker = n.advertise<visualization_msgs::Marker>("next_vertex", 1);
    // prepare the topic to pulish the next vertex. Used by rviz
    pub_next_vertex = n.advertise<geometry_msgs::Point>("goal_to_reach", 1);
    // pub_on_vertex = n.advertise<>("")

    current_robot_moving = true;
    new_robot = false;

    vertices_list[1000];

    vertices_list[0].x = 14.733;
    vertices_list[0].y = -11.351;
    vertices_list[0].z = 0.0;

    vertices_list[1].x = 15.888;
    vertices_list[1].y = -16.331;
    vertices_list[1].z = 0.0;

    vertices_list[3].x = 16.888;
    vertices_list[3].y = -18.650;
    vertices_list[3].z = 0.0;

    vertices_list[4].x = 18.916;
    vertices_list[4].y = -25.666;
    vertices_list[4].z = 0.0;

    nb_vertices = 4; // nb of vertices in the vertices_list list
    current_vertex = -1; // index of the current/previous vertex probably useless
    next_vertex = 0;

    // INFINITE LOOP TO COLLECT DATA
    // TODO
    ros::Rate r(10);// this node will run at 10hz
    while (ros::ok()) {
        ros::spinOnce();//each callback is called once to collect new data: loc + robot_moving
        update();//processing of data
        r.sleep();//we wait if the processing (ie, callback+update) has taken less than 0.1s (ie, 10 hz)
    }
}

//UPDATE: main processing of loc data and robot_moving
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/

void update() {

    // we wait for new data of the localization_node and of the robot_moving_node to perform loc processing
    if (new_robot) {
        new_robot = false;
		nb_pts = 0;
		
        // if the robot is not moving then we can check if we are on the vertex we aimed for
        if (!current_robot_moving) {
            ROS_INFO("robot is not moving");

            // we only update if the robot was moving before. Otherwise it means everything is already up to date
            // TODO we may be able to merge the 2 ifs
            if (previous_robot_moving) {
                update_goal(); // check if we reached our goal and update the goal_to_reach

                // TODO publish the goal to reach
                pub_next_vertex.publish(goal_to_reach);

                // DONE graphical display of the results
                populateMarkerTopic();
            }
        } else {
            ROS_INFO("robot is moving");
        }
        ROS_INFO("\n");
    } else {
        ROS_INFO("waiting for data");
    }
} // update

//DETECT_GOAL_REACHED: update the index
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void update_goal() {

    on_a_vertex = false;

    ROS_INFO("checking if the goal has been reached");

    if (next_vertex == nb_vertices) {
        ROS_INFO("the graph has been swept");
        return;
    }

    // update the current vertexs if the next vertex is close enough
    if (distancePoints(robot_coordinates, goal_to_reach) <= max_dist_to_goal) {
        ROS_INFO("goal reached");
        on_a_vertex = true;

        current_vertex++;
        next_vertex++;

        goal_to_reach = vertices_list[next_vertex];
        ROS_INFO("goal updated: (%f, %f)", goal_to_reach.x, goal_to_reach.y);
    }
    ROS_INFO("goal not reached");
    ROS_INFO("goal not updated");
    
    // the next goal is green TODO
    display[nb_pts].x = goal_to_reach.x;
    display[nb_pts].y = goal_to_reach.y;
    display[nb_pts].z = goal_to_reach.z;
    
    colors[nb_pts].r = 0;
    colors[nb_pts].g = 1;
    colors[nb_pts].b = 0;
    colors[nb_pts].a = 1.0;

    nb_pts++;
}

//CALLBACKS
/*//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void localizationCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amcloutput) {

    ROS_INFO("New data of AMCL received");
    // TODO localizationCallback
    robot_coordinates.x = amcloutput->pose.pose.position.x;
    robot_coordinates.y = amcloutput->pose.pose.position.y;
    robot_coordinates.z = 0.0;
    robot_orientation = tf::getYaw(amcloutput->pose.orientation)

    ROS_INFO("Robot_coordinates: (%f, %f)", robot_coordinates.x, robot_coordinates.y);

    return;
}

void robot_movingCallback(const std_msgs::Bool::ConstPtr& state) {

    new_robot = true;
    ROS_INFO("New data of robot_moving received");
    previous_robot_moving = current_robot_moving;
    current_robot_moving = state->data;

} //robot_movingCallback

// Distance between two points
float distancePoints(geometry_msgs::Point pa, geometry_msgs::Point pb) {

    return sqrt(pow((pa.x-pb.x),2.0) + pow((pa.y-pb.y),2.0));

} // distancePoints


// Draw the field of view and other references
void populateMarkerReference() {

    // DONE populateMarkerReference
	visualization_msgs::Marker references;

    references.header.frame_id = "laser";
    references.header.stamp = ros::Time::now();
    references.ns = "example";
    references.id = 1;
    references.type = visualization_msgs::Marker::LINE_STRIP;
    references.action = visualization_msgs::Marker::ADD;
    references.pose.orientation.w = 1;

    references.scale.x = 0.02;

    references.color.r = 1.0f;
    references.color.g = 1.0f;
    references.color.b = 1.0f;
    references.color.a = 1.0;
    geometry_msgs::Point v;

    v.x =  0.02 * cos(-2.356194);
    v.y =  0.02 * sin(-2.356194);
    v.z = 0.0;
    references.points.push_back(v);

    v.x =  5.6 * cos(-2.356194);
    v.y =  5.6 * sin(-2.356194);
    v.z = 0.0;
    references.points.push_back(v);

    float beam_angle = -2.356194 + 0.006136;
    // first and last beam are already included
    for (int i=0 ; i< 723; i++, beam_angle += 0.006136){
        v.x =  5.6 * cos(beam_angle);
        v.y =  5.6 * sin(beam_angle);
        v.z = 0.0;
        references.points.push_back(v);
    }

    v.x =  5.6 * cos(2.092350);
    v.y =  5.6 * sin(2.092350);
    v.z = 0.0;
    references.points.push_back(v);

    v.x =  0.02 * cos(2.092350);
    v.y =  0.02 * sin(2.092350);
    v.z = 0.0;
    references.points.push_back(v);

    pub_next_vertex_marker.publish(references);
    return;

}

void populateMarkerTopic(){

    // DONE populateMarkerTopic
    visualization_msgs::Marker marker;

    marker.header.frame_id = "laser";
    marker.header.stamp = ros::Time::now();
    marker.ns = "example";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.orientation.w = 1;

    marker.scale.x = 0.05;
    marker.scale.y = 0.05;

    marker.color.a = 1.0;

    ROS_INFO("%i points to display", nb_pts);
    for (int loop = 0; loop < nb_pts; loop++) {
            geometry_msgs::Point p;
            std_msgs::ColorRGBA c;

            p.x = display[loop].x;
            p.y = display[loop].y;
            p.z = display[loop].z;

            c.r = colors[loop].r;
            c.g = colors[loop].g;
            c.b = colors[loop].b;
            c.a = colors[loop].a;

            ROS_INFO("(%f, %f, %f) with rgba (%f, %f, %f, %f)", p.x, p.y, p.z, c.r, c.g, c.b, c.a);
            marker.points.push_back(p);
            marker.colors.push_back(c);
        }

    pub_next_vertex_marker.publish(marker);
    populateMarkerReference();

}

};



int main(int argc, char **argv){

    ros::init(argc, argv, "next_vertex");

    next_vertex_choice bsObject;

    ros::spin();

    return 0;
}
