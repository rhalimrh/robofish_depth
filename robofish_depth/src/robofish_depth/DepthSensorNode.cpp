#include "robofish_depth/DepthSensorNode.h"

#include "std_msgs/Float64.h"

robofish_depth::DepthSensorNode::DepthSensorNode(const ros::NodeHandle& node) {
	n = node;

	load_parameters();

	try {
		d = new DepthA2D(device.c_str(), depth_calib_coeff, depth_calib_offset);
	} catch (ndcl::TerminalError e) {
		ROS_FATAL("Could not create DepthA2D object: %s", e.what());
		ros::shutdown();
		exit(1);
	}

	depth_pub = n.advertise<std_msgs::Float64>("depth", 1000);
	ROS_INFO("DepthSensorNode initialized as %s", ros::this_node::getName().c_str());
}

robofish_depth::DepthSensorNode::~DepthSensorNode() {
	delete d;
}

void robofish_depth::DepthSensorNode::run_node() {
	ROS_INFO("%s running at %i Hz", ros::this_node::getName().c_str(), d->depth_rate);

	ros::Rate loop_rate(d->depth_rate);
	while (ros::ok()) {
		try {
			std_msgs::Float64 msg;
			if (d->get_reading(msg.data)) {
				depth_pub.publish(msg);
			} else {
				// Only repeat once a minute at most
				ROS_WARN_THROTTLE(60, "Unable to retrieve depth sensor message");
			}
		} catch (DepthA2DError e) {
			ROS_ERROR_THROTTLE(10,"Unable to retrieve depth sensor message: %s", e.what());
		}

		ros::spinOnce();
		loop_rate.sleep();
	}
}

void robofish_depth::DepthSensorNode::load_parameters() {
	if (!n.getParam("depth/device", device)) {
		ROS_FATAL("Could not get a device file (parameter depth/device) for the compass!");
		ros::shutdown();
		exit(1);
	}

	if (!n.getParam("depth/coeff", depth_calib_coeff)) {
		depth_calib_coeff = 1;
		ROS_WARN("Could not get coefficient (parameter depth/coeff) for the depth sensor.  Using %lf ft/mV.", depth_calib_coeff);
	}

	if (!n.getParam("depth/offset", depth_calib_offset)) {
		depth_calib_offset = 0;
		ROS_WARN("Could not get offset (parameter depth/offset) for the depth sensor.  Using %lf ft.", depth_calib_offset);
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "depth_sensor");
	ros::NodeHandle n;

	robofish_depth::DepthSensorNode node(n);
	node.run_node();

	return 0;
}
