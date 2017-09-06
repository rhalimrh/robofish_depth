#ifndef _pDepthA2D
#define _pDepthA2D

#include "ros/ros.h"
#include "std_msgs/Float64.h"

#include "ndcl/PseudoTerminal.h"

namespace robofish_depth {
	/** A class to test the DepthA2D class without hardware.
	 *
	 * This class encapsulates a ROS node that sets up a fake hardware
	 * device that publishes data in the same format as the depth sensor on
	 * the fish to make it easier to test the depth sensor code without having to
	 * plug in and power any actual hardware.  The node sets up a pseudo-terminal
	 * device, then automatically sets the depth sensor device parameter on
	 * the ROS master to point to the associated slave device.  Then it listens
	 * for simulated depth measurents on sim_depth and republishes them
	 * over the pseudo-terminal.
	 */
	class pDepthA2D {
		public:
			/** Constructor for the pDepthA2D.
			 *
			 * Sets the parameters needed by the depth A2D node and opens a
			 * pseudo-terminal.  It then sets the value of the
			 * depth/device parameter on the ROS parameter server to the
			 * filename of the pseudo-terminal opened here.
			 *
			 * @param node  handle for the node that the class commands
			 */
			pDepthA2D(ros::NodeHandle& node);

			/** Destructor for the pDepthA2D.
			 *
			 * Deletes the pDepthA2D object, closing the connection to the
			 * depth A2D through the pseudo-terminal.
			 */
			~pDepthA2D();

			/** Runs the ROS node for the pDepthA2D.
			 *
			 * Just spins.
			 */
			void run_node();

		private:
			/** Loads the parameters for the node.
			 *
			 * Does not use any parameters at present.
			 */
			void load_parameters();

			/** Callback for generating fake depth measurements.
			 *
			 * Accepts simulated depth measurements on the sim_depth
			 * topic and writes them to the pseudo-terminal for the real
			 * depth sensor node to read.
			 *
			 * @param msg  The float64 depth value to convert to uint depth sensor reading
			 */
			void sim_cb(const std_msgs::Float64& msg);

			/** Callback for writing fake depth measurements.
			 *
			 * Composes a fake depth message from the last published sim
			 * message and writes it to the pseudo-terminal for the real
			 * depth sensor node to read.
			 *
			 * @param event  debugging information
			 */
			void pub_cb(const ros::TimerEvent& event);

			/** The rate at which the DepthA2D produces readings in Hz.
			 */
			//static const int depth_rate = 10;

			/** The default baud rate of the DepthA2D.
			 */
			//static const int baud_rate = 19200;

			ros::NodeHandle n;
			ros::Subscriber depth_sim_sub;

			ndcl::PseudoTerminal* term;

			double depth_calib_coeff;
			double depth_calib_offset;

			double depth;
	};
}

#endif
