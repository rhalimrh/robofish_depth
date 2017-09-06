#ifndef _DepthSensorNode
#define _DepthSensorNode

#include "ros/ros.h"

#include "robofish_depth/DepthA2D.h"

namespace robofish_depth {
	/** A class to run the depth sensor ROS node.
	 *
	 * This class encapsulates the running of the robofish DepthSensor node,
	 * which publishes to the depth topic and reads the depth sensor.
	 *
	 * The class handles advertising the node on the proper topic and makes
	 * the parameters needed by the node available without having to put
	 * them in a global namespace.
	 */
	class DepthSensorNode {
		public:
			/** Constructor for the DepthSensorNode.
			 *
			 * Loads the parameters needed by the node, then tries to create
			 * the DepthA2D object, and finally, advertises to the depth
			 * topic.
			 *
			 * @param node  handle for the node that the class commands
			 */
			DepthSensorNode(const ros::NodeHandle& node);

			/** Destructor for the DepthSensorNode.
			 *
			 * Deletes the DepthA2D object, closing the connection to the
			 * depth sensor.
			 */
			~DepthSensorNode();

			/** Runs the ROS node for the depth sensor.
			 *
			 * Runs a loop at the depth sensor measurement rate reading the
			 * depth sensor and publishing the readings to the depth topic,
			 * then spinning once and sleeping.
			 */
			void run_node();

		private:
			/** Loads the parameters for the node.
			 *
			 * The depth/device parameter must be set in the parameter
			 * server of this method will fail.  The other parameters are
			 * less essential, but if not set, the depth sensor measurements
			 * will almost certainly be wrong.
			 */
			void load_parameters();

			ros::NodeHandle n;
			ros::Publisher depth_pub;

			DepthA2D* d;
			std::string device;

			double zero;
			double depth_calib_offset;
			double depth_calib_coeff;
	};
}

#endif
