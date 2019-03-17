// EvohomeWirelessFW - RFBee firmware for evohome wireless communications
// Copyright (c) 2015 Hydrogenetic
// Copyright (c) 2019 Simon Arlott
//
// based on HoneyCommLite - Alternative RFBee firmware to communicate with
//                 Evohome / Hometronix / CM67z and other Honeywell 868MHz based RF devices.
//
// Copyright (C) 2012 JB137
// Copyright (C) 2011 Wladimir Komarow
//
// and work from CrazyDiamond and others at http://www.domoticaforum.eu/viewtopic.php?f=7&t=5806
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//

// Compile for RFbee using board: Arduino Pro or Pro Mini (3.3V, 8MHz) w/ATmega 168

#include "CCx.h"
#include "CCxCfg.h"

#define VERSION_NO "1.0"

#define SYNC_ON_32BITS

#define GDO0_INT 0 // INT0(PD2) wired to GDO0 on CC1101
#define GDO2_INT 1 // INT1(PD3) wired to GDO2 on CC1101 (CCx_IOCFG2==0x0B Serial Clock. Synchronous to the data in synchronous serial mode. In RX mode, data is set up on the falling edge by CC1101 when GDOx_INV=0. In TX mode, data is sampled by CC1101 on the rising edge of the serial clock when GDOx_INV=0.)

#define GDO0_PD 4 // PD2(INT0) wired to GDO0 on CC1101 (CCx_IOCFG0==0x0C Serial Synchronous Data Output. Used for synchronous serial mode.)
#define GDO2_PD 8 // PD3(INT1) wired to GDO2 on CC1101

                      // using the knowledge that the sync bytes after the preamble are 0xFF 0x00 0x33 0x55 0x53,
                      // that the values are sent LSB first and start bit = 0 and stop bit = 1 we create the 32 bit sync value
#if !defined(SYNC_ON_32BITS)
#define SYNC_WORD     (uint16_t)0x5595 // This is the last 2 bytes of the preamble / sync words..on its own it can be confused with the end of block when the last bit of the checksum is 0 (the following 0x55 pattern is then converted to 0xFF)
#else
#undef SYNC_WORD
#define SYNC_WORD    ((uint32_t)0x59955595) // The 32 bit version of the synchronisation sequence
#endif

//#define DEBUG

enum progMode {
	pmIdle,
	pmSendBadPacket,
	pmSendHavePacket,
	pmSendReady,
	pmSendActive,
	pmSendFinished,
};

#define TYPE_MASK 0x3
#define TYPE_SHIFT 4

enum enType {
	enRQ,
	enI,
	enW,
	enRP,
};

#define DEV_MASK 0x3
#define DEV_SHIFT 2

enum enDev {
	enDev0 = 0x01,
	enDev1 = 0x02,
	enDev2 = 0x04,
};

#define PARAM_MASK 0x3
#define PARAM_SHIFT 0

enum marker {
	maComplete,
	maResync,
	maInvalid,
	maOverflow,
};

const byte manc_enc[16] = { 0xAA, 0xA9, 0xA6, 0xA5, 0x9A, 0x99, 0x96, 0x95, 0x6A, 0x69, 0x66, 0x65, 0x5A, 0x59, 0x56, 0x55 };
const byte pre_sync[5] = { 0xFF, 0x00, 0x33, 0x55, 0x53 };
const uint8_t dev_map[4] = {
	enDev0 | enDev1 | enDev2,
	                  enDev2,
	enDev0 |          enDev2,
	enDev0 | enDev1,
};

byte out_preamble;
byte out_type;
byte out_devices;
byte out_len;

byte in_header;
byte in_type;
byte in_devices;
byte in_params;

byte unmap_dev(byte header) {
	return dev_map[(header >> DEV_SHIFT) & DEV_MASK];
}

byte map_dev(byte devices) {
	for (byte n = 0; n < sizeof(dev_map); n++) {
		if (dev_map[n] == devices) {
			return n << DEV_SHIFT;
		}
	}
	return 0xFF;
}

#define MAX_PACKETS 7

struct recv_packet {
	int8_t index;
#ifdef DEBUG
	uint32_t timestamp;
#endif
	boolean ready;
	uint8_t length;
	enum marker status;
	uint8_t rssi;
	uint8_t data[128];
	struct recv_packet *next;
};
struct recv_packet recv_buffer[MAX_PACKETS];
struct recv_packet *head;
struct recv_packet *tail;
struct recv_packet *write;
volatile int8_t readIndex = -1;
volatile boolean available = false;
volatile boolean read_rssi = false;
volatile byte rssi;
volatile byte send_buffer[128];

byte pm = pmIdle;
volatile byte sm = pmIdle;

byte bit_counter = 0;
byte byte_buffer = 0;

volatile boolean in_sync = false;
#if !defined(SYNC_ON_32BITS)
uint16_t sync_buffer = 0;
#else
uint32_t sync_buffer = 0;
#endif
boolean highnib = true;
boolean last_bit;

byte sp = 0;
byte op = 0;
byte pp = 0;

char param[10];

uint8_t timer_interrupts = 0;

struct decode_packet {
	enum enType type;
	uint8_t has_devices;
	uint8_t devices_read;
	uint8_t has_params;
	uint8_t params_read;

	uint32_t devices[3];
	uint8_t params[2];
	uint16_t command;
	uint8_t check;
} __attribute__((packed));


static inline boolean decode_header(struct decode_packet *packet, const uint8_t *data, uint8_t &n) {
	uint8_t header = data[n];
	packet->check += data[n++];

	if (header & 0xC0) {
		Serial.print(F("*Unknown header=0x"));
		Serial.println(header, HEX);
		return false;
	}

	packet->type = (enum enType)((header >> TYPE_SHIFT) & TYPE_MASK);
	packet->has_devices = unmap_dev(header);
	packet->has_params = (header >> PARAM_SHIFT) & PARAM_MASK;
	return true;
}

static inline boolean decode_device(struct decode_packet *packet, uint8_t device, const uint8_t *data, uint8_t &n, uint8_t length) {
	if (n + 2 >= length) {
		return false;
	}

	packet->devices[device] = (uint32_t)data[n] << 16;
	packet->check += data[n++];

	packet->devices[device] |= (uint32_t)data[n] << 8;
	packet->check += data[n++];

	packet->devices[device] |= data[n];
	packet->check += data[n++];
	return true;
}

static inline boolean decode_devices(struct decode_packet *packet, const uint8_t *data, uint8_t &n, uint8_t length) {
	uint8_t device_bit = enDev0;

	for (uint8_t i = 0; i < 3; i++) {
		if (packet->has_devices & device_bit) {
			if (!decode_device(packet, i, data, n, length)) {
				return false;
			}
			packet->devices_read |= device_bit;
		}
		device_bit <<= 1;
	}

	return true;
}

static inline boolean decode_params(struct decode_packet *packet, const uint8_t *data, uint8_t &n, uint8_t length) {
	for (uint8_t i = 0; i < 2; i++) {
		if (packet->has_params & (2 >> i)) {
			if (n + 1 < length) {
				return false;
			}

			packet->params[i] = data[n];
			packet->check += data[n++];

			packet->params_read |= (2 >> i);
		}
	}

	return true;
}

static inline boolean decode_command(struct decode_packet *packet, const uint8_t *data, uint8_t &n, uint8_t length) {
	if (n + 1 >= length) {
		return false;
	}

	packet->command = (uint16_t)data[n] << 8;
	packet->check += data[n++];

	packet->command |= data[n];
	packet->check += data[n++];

	return true;
}

static inline void print_header(struct decode_packet *packet) {
	char tmp[11];

	if (packet->type == enI) {
		Serial.print(" I ");
	} else if (packet->type == enRQ) {
		Serial.print("RQ ");
	} else if (packet->type == enRP) {
		Serial.print("RP ");
	} else if (packet->type == enW) {
		Serial.print(" W ");
	}

	if (packet->has_params & 2) {
		if (packet->params_read & 2) {
			sprintf(tmp, "%03u ", packet->params[0]);
			Serial.print(tmp);
		} else {
			Serial.print("??? ");
		}
	} else {
		Serial.print("--- ");
	}

	uint8_t device_bit = enDev0;

	for (uint8_t i = 0; i < 3; i++) {
		if (packet->has_devices & device_bit) {
			if (packet->devices_read & device_bit) {
				sprintf(tmp, "%02u:%06lu ", (unsigned int)(packet->devices[i] >> 18) & 0x3F, (unsigned long)(packet->devices[i] & 0x3FFFF));
				Serial.print(tmp);
			} else {
				Serial.print("??:?????? ");
			}
		} else {
			Serial.print("--:------ ");
		}
		device_bit <<= 1;
	}

	sprintf(tmp, "%04X ",packet->command);
	Serial.print(tmp);
}

static inline boolean decode_print_content(struct decode_packet *packet, const uint8_t *data, uint8_t &n, uint8_t length) {
	char tmp[4];

	if (n >= length) {
		return false;
	}

	uint8_t len = data[n];
	packet->check += data[n++];

	sprintf(tmp, "%03u ", len);
	Serial.print(tmp);

	while (len-- > 0) {
		if (n >= length) {
			return false;
		}

		uint8_t value = data[n];
		packet->check += data[n++];

		sprintf(tmp, "%02X", value);
		Serial.print(tmp);
	}

	return true;
}

static inline boolean decode_print_checksum(struct decode_packet *packet, const uint8_t *data, uint8_t &n, uint8_t length) {
	if (n >= length) {
		return false;
	}

	uint8_t check = data[n];
	packet->check += data[n++];

	if (packet->check != 0) {
		char tmp[7];

		Serial.print(F("*CHK*"));
		sprintf(tmp, "+%02X=%02X", check, packet->check);
		Serial.print(tmp);
	}

	return true;
}

SIGNAL(TIMER0_COMPA_vect) {
	// Only run after being idle for 5ms
	if (timer_interrupts >= 5 && !available) {
		// When reading is finished, put the buffer on the end of the list
		if (readIndex != -1) {
			tail->next = &recv_buffer[readIndex];
			tail = &recv_buffer[readIndex];
			// tail->next was set to NULL by the read process
			readIndex = -1;
		}

		// Take the top off the list for reading
		if (head->ready) {
			readIndex = head->index;
			head = head->next;
			available = true;
		}

		timer_interrupts = 0;
	} else {
		timer_interrupts++;
	}
}

void finish_recv_buffer(enum marker status) {
	in_sync = false;

	if (write->length > 0) {
#ifdef DEBUG
		write->timestamp = micros();
#endif
		write->status = status;
		write->ready = true;
		write->rssi = rssi;

		write = write->next;
		if (write == NULL) {
			write = head;
			head = head->next;

			tail->next = write;
			tail = write;
			tail->next = NULL;

			write->ready = false;
			write->length = 0;
		}
	}

	rssi = 0;
}

// Interrupt to receive data and find_sync_word
void sync_clk_in() {
	static byte byte_bit;
	byte new_bit = (PIND & GDO0_PD); // sync data

	// keep our buffer rolling even when we're in sync
	sync_buffer <<= 1;
	if (new_bit) {
		sync_buffer |= 1;
	}

	if (sync_buffer == SYNC_WORD) {
		if (in_sync) {
			// abort and restart
			finish_recv_buffer(maResync);
		}

		bit_counter = 0;
		byte_buffer = 0;
		byte_bit = 0x10;
		in_sync = true;
		read_rssi = true;

		timer_interrupts = 0;
	} else if (in_sync) {
		if (bit_counter > 0) { // Skip start bit
			if (bit_counter < 9) {
				if (bit_counter % 2) {
					if (new_bit) {
						byte_buffer |= byte_bit;
					}

					// for each byte, byte_bit cycles through 0x10 0x20 0x40 0x80 (0x00) 0x01 0x02 0x04 0x08 (0x10)
					byte_bit <<= 1;
					if (byte_bit == 0x10) { // finished all 8 bits
						if (write->length == sizeof(write->data)) {
							finish_recv_buffer(maOverflow);
							return;
						}
						// we can not see raw 0x35 here as our buffer is already manchester decoded we rely on rejection of 0x35 as it is not manchester encoded to end our packet
						write->data[write->length++] = byte_buffer;
						byte_buffer = 0;
					} else if (byte_bit == 0x00) { // finished the high 4 bits, now do the low 4 bits
						byte_bit = 0x01;
					}
				} else {
					if (new_bit == last_bit) { // manchester encoding must always have 1 transition per bit pair
						finish_recv_buffer(maInvalid);
						return;
					}
				}
				last_bit = new_bit;
			} else { // Last bit has been received
				if (!new_bit) {
					finish_recv_buffer(maComplete);
					return;
				}
				bit_counter = 0;
				return;
			}
		} else {
			if (new_bit) {
				finish_recv_buffer(maInvalid);
				return;
			}
		}
		bit_counter++;

		timer_interrupts = 0;
	}
}

// Interrupt to send data
void sync_clk_out() {
	if (sm != pmSendActive) {
		return;
	}
	if (bit_counter < 9) {
		if (!bit_counter) {
			PORTD &= ~GDO0_PD;
			if (out_preamble < 5) {
				byte_buffer = 0x55;
				out_preamble++;
			} else if (pp < 5) {
				byte_buffer = pre_sync[pp++];
			} else {
				if (sp<op) {
					byte_buffer = send_buffer[sp];
					if (highnib) {
						byte_buffer = manc_enc[(byte_buffer >> 4) & 0xF];
					} else {
						if (write->length < sizeof(write->data)) {
							write->data[write->length++] = byte_buffer;
						}
						byte_buffer = manc_enc[byte_buffer & 0xF];
						sp++;
					}
					highnib = !highnib;
				} else if (sp <= op + 4) {
					if (sp == op) {
						byte_buffer = 0x35;
					} else {
						byte_buffer = 0x55;
					}
					sp++;
				} else {
					sm = pmSendFinished;
					return;
				}
			}
		} else {
			if (byte_buffer & 0x01) {
				PORTD |= GDO0_PD;
			} else {
				PORTD &= ~GDO0_PD;
			}
			byte_buffer >>= 1;
		}
		bit_counter++;
	} else {
		PORTD |= GDO0_PD;
		bit_counter = 0;
	}
}

// Setup
void setup() {
	delay(200); // probably not a bad idea to wait until power etc. have stabilised a little

	// Power up and configure the CC1101
	CCx.PowerOnStartUp();
	CCx.Setup(0);
	while (((CCx.Write(CCx_SIDLE, 0) >> 4) & 7) != 0);
	while (((CCx.Write(CCx_SRX, 0) >> 4) & 7) != 1); // will calibrate when going to rx
	CCx.Write(CCx_FSCTRL0, -20);

	// Data is received at 38k4 (packet bytes only at 19k2 due to manchester encoding)
	// 115k2 provides enough speed to perform processing and write the received
	// bytes to serial, but cannot be used reliably with an 8MHz clock
	Serial.begin(250000);

	Serial.println(F("##############################################################"));
	Serial.println(F("# EvohomeWirelessFW v" VERSION_NO));
	Serial.println(F("# Licensed under GPL-3.0+ <http://spdx.org/licenses/GPL-3.0+>"));

	head = &recv_buffer[0];
	tail = &recv_buffer[MAX_PACKETS - 1];
	write = head;
	for (uint8_t i = 0; i < MAX_PACKETS; i++) {
		recv_buffer[i].index = i;
		if (i == MAX_PACKETS - 1) {
			recv_buffer[i].next = NULL;
		} else {
			recv_buffer[i].next = &recv_buffer[i + 1];
		}
	}

	// Set up a regular timer interrupt to maintain the read buffer
	OCR0A = 0x01;
	TIMSK0 |= _BV(OCIE0A);

	// Attach the find_sync_word interrupt function to the
	// falling edge of the serial clock connected to INT(1)
	attachInterrupt(GDO2_INT, sync_clk_in, FALLING);
}

// Main loop
void loop() {
	if (sm == pmSendFinished) {
		detachInterrupt(GDO2_INT);
		in_sync = false;
		bit_counter = 0;
		finish_recv_buffer(maComplete);
		pinMode(2,INPUT);
		while(((CCx.Write(CCx_SRX, 0) >> 4) & 7) != 1);
		attachInterrupt(GDO2_INT, sync_clk_in, FALLING);
		pp = 0;
		op = 0;
		sp = 0;
		sm = pmIdle;
	}
	if (read_rssi) {
		byte rssi_tmp;

		CCx.Read(CCx_RSSI, &rssi_tmp);
		rssi = (rssi_tmp < 128) ? (rssi_tmp + 128) : (rssi_tmp - 128); // not the RSSI, but a scale of 0-255
		read_rssi = false;
	}
	if (available) {
		struct recv_packet r_packet;

		if (recv_buffer[readIndex].ready) {
			memcpy(&r_packet, &recv_buffer[readIndex], offsetof(struct recv_packet, data) + recv_buffer[readIndex].length);
			recv_buffer[readIndex].ready = false;
			recv_buffer[readIndex].length = 0;
			recv_buffer[readIndex].next = NULL;
			available = false;
		} else {
			return;
		}

		if (!r_packet.ready) {
			return;
		}

#ifdef DEBUG
		Serial.print("# timestamp=");
		Serial.print(r_packet.timestamp, DEC);
		Serial.print(" index=");
		Serial.print(r_packet.index, DEC);
		Serial.print(" age=");
		Serial.println(micros() - r_packet.timestamp, DEC);
#endif

		struct decode_packet d_packet;
		uint8_t n = 0;
		char tmp[5];

		memset(&d_packet, 0, sizeof(d_packet));

		sprintf(tmp, "%03u ", r_packet.rssi);
		Serial.print(tmp);

		if (!decode_header(&d_packet, r_packet.data, n)) {
			return;
		}

		if (!decode_devices(&d_packet, r_packet.data, n, r_packet.length)) {
			goto incomplete;
		}

		if (!decode_params(&d_packet, r_packet.data, n, r_packet.length)) {
			goto incomplete;
		}

		if (!decode_command(&d_packet, r_packet.data, n, r_packet.length)) {
			goto incomplete;
		}

		print_header(&d_packet);

		if (!decode_print_content(&d_packet, r_packet.data, n, r_packet.length)) {
			goto incomplete_content;
		}

		if (!decode_print_checksum(&d_packet, r_packet.data, n, r_packet.length)) {
			goto incomplete_content;
		}

		Serial.println();
		return;

incomplete:
		print_header(&d_packet);

incomplete_content:
		if (r_packet.status == maComplete) {
			Serial.println(F("*INCOMPLETE*END*"));
		} else if (r_packet.status == maResync) {
			Serial.println(F("*INCOMPLETE*RESYNC*"));
		} else if (r_packet.status == maInvalid) {
			Serial.println(F("*INCOMPLETE*INVALID*"));
		} else if (r_packet.status == maOverflow) {
			Serial.println(F("*INCOMPLETE*OVERFLOW*"));
		}
	} else if (sm < pmSendReady) {
		if (Serial.available()) {
			char out = Serial.read();
			if (out == '\r') {
			} else if (out == '\n' && sm == pmSendHavePacket) { // send on lf
				byte sc = 0;
				for (int n = 0; n < op; n++) {
					sc += send_buffer[n];
				}
				send_buffer[op++] = -sc;
				sm = pmSendReady;
			} else if (out == '\x11' || out == '\n') { // escape or bad packet
				pp = 0;
				op = 0;
				sp = 0;
				sm = pmIdle;
			} else if (sm == pmSendBadPacket) { // don't do any more processing if the packet is bad
			} else if (sp < 7) {
				if (out == ' ') {
					if (pp) {
						param[pp] = 0;
						if (sp == 0) {
							out_devices = 0;
							if (!strcmp(param, "I")) {
								out_type = enI;
							} else if (!strcmp(param, "RQ")) {
								out_type = enRQ;
							} else if (!strcmp(param, "RP")) {
								out_type = enRP;
							} else if (!strcmp(param, "W")) {
								out_type = enW;
							} else {
								sm = pmSendBadPacket;
								return;
							}
							send_buffer[op++] = 0;
						} else if (sp == 1) {
						} else if (sp >= 2 && sp <= 4) {
							if (param[0] != '-') {
								out_devices |= 1 << (sp - 2);
								uint8_t idType;
								uint32_t idAddr;
								sscanf(param, "%02hhu:%06lu", &idType, &idAddr);
								idAddr |= (uint32_t)idType << 18;
								send_buffer[op++] = (idAddr >> 16) & 0xFF;
								send_buffer[op++] = (idAddr >> 8) & 0xFF;
								send_buffer[op++] = idAddr & 0xFF;
							}
						} else if (sp == 5) {
							send_buffer[0] |= map_dev(out_devices);
							if (send_buffer[0] == 0xFF) {
								sm = pmSendBadPacket;
								return;
							}
							send_buffer[0] |= out_type << TYPE_SHIFT;
							uint16_t cmd;
							sscanf(param, "%04X", &cmd);
							send_buffer[op++] = (cmd >> 8) & 0xFF;
							send_buffer[op++] = cmd & 0xFF;
						} else if (sp == 6) {
							sscanf(param, "%03hhu", &out_len);
							send_buffer[op++] = out_len;
						}

						pp = 0;
						sp++;
					}
				} else if (pp < 9) {
					param[pp++] = out;
				}
			} else {
				param[pp++] = out;
				if (pp == 2) {
					param[pp] = 0;
					byte pay;
					sscanf(param, "%02hhX", &pay);
					send_buffer[op++] = pay;
					pp = 0;
					if (op > (out_len + 10) || op > 127) {
						sm = pmSendBadPacket;
					} else if (op == (out_len + 10)) {
						sm = pmSendHavePacket;
					}
				}
			}
		}
	} else if (sm == pmSendReady && !in_sync) {
		detachInterrupt(GDO2_INT);
		while (((CCx.Write(CCx_SIDLE, 0) >> 4) & 7) != 0);
		while (((CCx.Write(CCx_STX, 0) >> 4) & 7) != 2); // will calibrate when going to tx
		pinMode(2, OUTPUT);
		sm = pmSendActive;
		highnib = true;
		bit_counter = 0;
		sp = 0;
		pp = 0;
		out_preamble = 0;
		finish_recv_buffer(maComplete);
		attachInterrupt(GDO2_INT, sync_clk_out, RISING);
	}
}
