#include <sys/socket.h>
#include <errno.h>
#include <cstring>

#include "ndcl/num_utils.h"
#include "ndcl/Timer.h"

#include "robofish_depth/DepthA2D.h"

robofish_depth::DepthA2D::DepthA2D(const char* device, double coef, double off) {
	serial = new ndcl::SerialPort(device, baud_rate);

	low = 0;
	coeff = coef;
	offset = off;
}

robofish_depth::DepthA2D::~DepthA2D() {
	delete serial;
}

bool robofish_depth::DepthA2D::get_reading(double& depth) {
	bool token_found = false;

	ndcl::Timer timer;
	while (!token_found && timer.elapsed() < 1.1/depth_rate) {
		if (serial->wait_for_data(1.1/depth_rate - timer.elapsed())) {
			while (serial->has_data()) {
				byte c = serial->get_byte();
				printf("%u\n", c);
				
				if (!check_byte(c)) {
					throw DepthA2DError("Byte failed checksum");
				}

				if (c & 0x80 && low != 0) {
					unsigned short int raw = ((c & 0x7C) << 3) | ((low & 0x7C) >> 2);
					depth = coeff*raw + offset;
					token_found = true;
				} else {
					low = c;
				}
			}
		}
	}

	return token_found;
}

bool robofish_depth::DepthA2D::check_byte(byte c) {
	// Accumulate xor parity of data bits into second bit position
	bool parity = false;
	byte data = c & 0xFC;
	while (data) {
		parity = !parity;
		data = data & (data - 1);
	}

	// Check that xor parity matches and last bit is set true
	if (c & 0x02) {
		return parity && (c & 0x01);
	} else {
		return !parity && (c & 0x01);
	}
}
