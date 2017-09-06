#include <pty.h>

#include "ndcl/num_utils.h"

#include "robofish_depth/DepthA2D.h"
#include "robofish_depth/pDepthA2D.h"

robofish_depth::pDepthA2D::pDepthA2D(ros::NodeHandle& node) {
	n = node;

	load_parameters();

	try {
		term = new ndcl::PseudoTerminal(DepthA2D::baud_rate);
	} catch (ndcl::TerminalError e) {
		ROS_FATAL("Could not open new PseudoTerminal: %s", e.what());
		ros::shutdown();
	}

	n.setParam("depth/device", term->get_slave_name());

	depth_sim_sub = n.subscribe("sim_depth", 1, &pDepthA2D::sim_cb, this);

	ROS_INFO("pDepthA2D node initialized as %s", ros::this_node::getName().c_str());
}

robofish_depth::pDepthA2D::~pDepthA2D() {
	delete term;
}

void robofish_depth::pDepthA2D::run_node() {
	ROS_INFO("%s running at %i Hz", ros::this_node::getName().c_str(), DepthA2D::depth_rate);

	ros::Timer timer = n.createTimer(ros::Duration(1.0/DepthA2D::depth_rate), &pDepthA2D::pub_cb, this);
	ros::spin();
}

void robofish_depth::pDepthA2D::load_parameters() {
	if (!n.getParam("depth/coeff", depth_calib_coeff)) {
		depth_calib_coeff = 0.01;
		ROS_WARN("Could not get coefficient (parameter depth/coeff) for the fake depth sensor.  Using %lf ft/mV.", depth_calib_coeff);
		n.setParam("depth/coeff", depth_calib_coeff);
	}

	if (!n.getParam("depth/offset", depth_calib_offset)) {
		depth_calib_offset = -0.23;
		ROS_WARN("Could not get offset (parameter depth/coeff) for the fake depth sensor.  Using %lf ft.", depth_calib_offset);
		n.setParam("depth/offset", depth_calib_offset);
	}
}

void robofish_depth::pDepthA2D::sim_cb(const std_msgs::Float64& msg) {
	depth = msg.data;
}

void robofish_depth::pDepthA2D::pub_cb(const ros::TimerEvent& event) {
	// Convert the double depth value to the corresponding unsigned int reading that would come from the depth sensor
	double sat = ndcl::saturate((depth - depth_calib_offset)/depth_calib_coeff, 1023, 0);
	unsigned short int val = ndcl::round(sat);
	
	byte msg[2];
	
	msg[0] = 0;
	msg[0] |= (val & 0x1F) << 2;   // Least significant five data bits
	msg[0] |= 0x01;                // Stop bit is always set
	
	msg[1] = 0x80;                 // Sequence bit for top five data bits
	msg[1] |= (val & 0x3E0) >> 3;  // The most significant five data bits
	msg[1] |= 0x01;                // Stop bit is always set
	
	// Set parity bits
	bool parity = false;
	byte data = msg[0] & 0xFC;
	while (data) {
		parity = !parity;
		data = data & (data - 1);
	}
	msg[0] |= parity ? 0x02 : 0x00;
	
	parity = false;
	data = msg[1] & 0xFC;
	while (data) {
		parity = !parity;
		data = data & (data - 1);
	}
	msg[1] |= parity ? 0x02 : 0x00;
	
	// Transmit
	term->send(msg, 2);
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "pseudo_depth_sensor");
	ros::NodeHandle n;

	robofish_depth::pDepthA2D node(n);
	node.run_node();

	return 0;
}
