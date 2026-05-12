#include <iostream>
#include <string>

#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>

#include "phasespace_bringup/owl.hpp"

#include <phasespace_msgs/Camera.h>
#include <phasespace_msgs/Cameras.h>
#include <phasespace_msgs/Marker.h>
#include <phasespace_msgs/Markers.h>
#include <phasespace_msgs/Rigid.h>
#include <phasespace_msgs/Rigids.h>

//using namespace std;

int main(int argc, char **argv)
{
    // create the context
    OWL::Context owl;
    OWL::Cameras cameras;
    OWL::Markers markers;
    OWL::Rigids rigids;

    // initialize ROS
    ros::init(argc, argv, "phasespace_bringup");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // declare publishers
    ros::Publisher errors_pub = nh.advertise<std_msgs::String>("/phasespace/errors", 1000, true);
    ros::Publisher cameras_pub = nh.advertise<phasespace_msgs::Cameras>("/phasespace/cameras", 1000, true);
    ros::Publisher markers_pub = nh.advertise<phasespace_msgs::Markers>("/phasespace/markers", 1000);
    ros::Publisher rigids_pub = nh.advertise<phasespace_msgs::Rigids>("/phasespace/rigids", 1000);

    // load ip address from ros param
    std::string address;
    if (pnh.getParam("server_ip", address)) {
        ROS_INFO("Opening port at: %s", address.c_str());
    }
    else {
        ROS_ERROR("Failed to get ip address!!");
        // return 0;
    }
    // address = "192.168.1.230";

    // simple example
    if (owl.open(address) <= 0 || owl.initialize("timebase=1,1000000") <= 0) {
        ROS_ERROR("Connection failed!!");
        return 0;
    }

    // start streaming
    ROS_INFO("Starting streaming...");
    owl.streaming(1);
    
    ros::Rate r(1000);
    
    // main loop
    while (ros::ok() && owl.isOpen() && owl.property<int>("initialized")) {
        const OWL::Event *event = owl.nextEvent(1000);

        if (!event)
            continue;

        if (event->type_id() == OWL::Type::ERROR) {
            std::cerr << event->name() << ": " << event->str() << std::endl;
            std_msgs::String str;
            str.data = event->str();
            ROS_ERROR("Error : %s", event->str().c_str());
            errors_pub.publish(str);
        }
        else if(event->type_id() == OWL::Type::CAMERA) {
            if (event->name() == std::string("cameras") && event->get(cameras) > 0) {
                // read markers and publish
                phasespace_msgs::Cameras out;
                for (OWL::Cameras::iterator c = cameras.begin(); c != cameras.end(); c++) {
                    phasespace_msgs::Camera cout;
                    cout.id = c->id;
                    cout.flags = c->flags;
                    cout.cond = c->cond;
                    cout.x = c->pose[0] / 1000;
                    cout.y = -c->pose[2] / 1000;
                    cout.z = c->pose[1] / 1000;
                    cout.qw = c->pose[3];
                    cout.qx = c->pose[4];
                    cout.qy = c->pose[6];
                    cout.qz = c->pose[5];
                    out.cameras.push_back(cout);
                }

                cameras_pub.publish(out); 
            }
        }
        else if (event->type_id() == OWL::Type::FRAME) {
            if (event->find("markers", markers) > 0) {
                // read markers and publish
                phasespace_msgs::Markers out;
                for (OWL::Markers::iterator m = markers.begin(); m != markers.end(); m++) {
                    phasespace_msgs::Marker mout;
                    mout.id = m->id;
                    mout.time = m->time;
                    mout.flags = m->flags;
                    mout.cond = m->cond;
                    mout.x = m->x / 1000;
                    mout.y = m->z / 1000;
                    mout.z = m->y / 1000;
                    out.markers.push_back(mout);
                }

                markers_pub.publish(out);
            }

            if (event->find("rigids", rigids) > 0) {
                // read rigids and publish
                phasespace_msgs:: Rigids out;
                for (OWL::Rigids::iterator r = rigids.begin(); r != rigids.end(); r++) {
                    phasespace_msgs::Rigid rout;
                    rout.id = r->id;
                    rout.time = r->time;
                    rout.flags = r->flags;
                    rout.cond = r->cond;
                    rout.x = r->pose[0] / 1000;
                    rout.y = -r->pose[2] / 1000;
                    rout.z = r->pose[1] / 1000;
                    rout.qw = r->pose[3];
                    rout.qx = r->pose[4];
                    rout.qy = -r->pose[6];
                    rout.qz = r->pose[5];
                    out.rigids.push_back(rout);
                }

                rigids_pub.publish(out);
            }
        }

        r.sleep();
    } // while

    owl.done();
    owl.close();

    return 0;
}
