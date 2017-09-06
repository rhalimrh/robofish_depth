#ifndef _DepthA2D
#define _DepthA2D

#include "ndcl/SerialPort.h"

namespace robofish_depth {
	/** A class to provide an interface to the depth sensor.
	 *
	 * This class enables programs to more easily use the depth sensor in the fish
	 * by encapsulating the serial interface of the sensor and providing serial
	 * message parsing.
	 *
	 * Note that the depth sensor A2D board transmits raw amplified voltage values
	 * from the sensor that correlate linearly to depth.  They can be converted by
	 * the formula
	 *
	 * depth = coeff*raw_value + offset
	 */
	class DepthA2D {
		public:
			/** Constructor for the DepthA2D.
			 *
			 * Opens a serial connection to the depth sensor A2D board and waits for
			 * data requests.
			 *
			 * @param device  string naming the device file that the serial input
			 *                from the sensor is connected to
			 * @param coef    double giving the slope of the
			 * @param off
			 * @throws SerialError  if the named device file cannot be opened
			 */
			DepthA2D(const char* device, double coef, double off);

			/** Destructor for the DepthA2D.
			 *
			 * Closes the serial connection to the depth sensor.
			 */
			~DepthA2D();

			/** Gets a reading from the depth sensor.
			 *
			 * Reads from the serial port until there are no more bytes or a depth sensor
			 * message is encountered. If this is successful, true is returned, otherwise
			 * false. No data is lost if only a partial message is received before
			 * the timeout.
			 *
			 * @param depth  a double reference that will be filled with the depth
			 *               in feet
			 * @returns              boolean indicating if a new reading was read and
			 *                       successfully parsed
			 * @throws DepthA2DError if a byte fails the parity check
			 * @throws SerialError   if the serial port cannot be read from (in
			 *                       theory, this should never happen)
			 */
			bool get_reading(double& depth);

			/** The rate at which the depth sensor produces readings in Hz.
			 */
			static const int depth_rate = 50;

			/** The default baud rate of the depth sensor.
			 */
			static const int baud_rate = 19200;

		private:
			/** Verifies that the byte passes parity.
			 *
			 * Computes the xor parity of the 5 data bits and compares it to the parity
			 * bit.  Jian also sets the last bit high always, so we verify that as well.
			 *
			 * @param c  byte to check
			 * @return   true if it is OK, false otherwise
			 */
			bool check_byte(byte c);

			ndcl::SerialPort* serial;

			double coeff;
			double offset;

			byte low;
	};

	/** An exception class thrown by the DepthA2D.
	 *
	 * DepthA2DError is thrown when there is a problem parsing a message from the
	 * serial port.
	 */
	class DepthA2DError : public ndcl::FormattedException {
		public:
			/** Constructor for DepthA2DError.
			 *
			 * The constructor takes a format string and arguments to fill in the string
			 * and saves the result to be returned by what().
			 *
			 * @param format  a format string similar to that in printf()
			 * @param ...     an argument list to fill in the format string
			 */
			DepthA2DError(const char* format, ...) throw() {
				va_list args;
				va_start(args, format);
				vsnprintf(detail, max_err_length, format, args);
				va_end(args);
			}

			/** Destructor for DepthA2DError.
			 *
			 * Cleans up.
			 */
			~DepthA2DError() throw() {
			}
	};
}

#endif
