// EvohomeWirelessFW - RFBee firmware for evohome wireless communications
// Copyright (c) 2015 Hydrogenetic
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

//some interesting notes on optimisation...
//http://blog.kriegsman.org/2013/12/01/optimizing-10-lines-of-arduino-code-youre-wrong-so-am-i/

#include "CCx.h"
#include "CCxCfg.h"

#define VERSION_NO "0.9alpha"

#define SYNC_ON_32BITS

#define GDO0_INT 0 // INT0(PD2) wired to GDO0 on CC1101
#define GDO2_INT 1 // INT1(PD3) wired to GDO2 on CC1101 (CCx_IOCFG2==0x0B Serial Clock. Synchronous to the data in synchronous serial mode. In RX mode, data is set up on the falling edge by CC1101 when GDOx_INV=0. In TX mode, data is sampled by CC1101 on the rising edge of the serial clock when GDOx_INV=0.)

#define GDO0_PD 4 // PD2(INT0) wired to GDO0 on CC1101 (CCx_IOCFG0==0x0C Serial Synchronous Data Output. Used for synchronous serial mode.)
#define GDO2_PD 8 // PD3(INT1) wired to GDO2 on CC1101

                      // using the knowledge that the sync bytes after the preamble are 0xFF 0x00 0x33 0x55 0x53,
                      // that the values are sent LSB first and start bit = 0 and stop bit = 1 we create the 32 bit sync value
#if !defined(SYNC_ON_32BITS)
#define SYNC_WORD     (uint16_t)0x5595//This is the last 2 bytes of the preamble / sync words..on its own it can be confused with the end of block when the last bit of the checksum is 0 (the following 0x55 pattern is then converted to 0xFF)
#else
#undef SYNC_WORD
#define SYNC_WORD    ((uint32_t)0x59955595) // The 32 bit version of the synchronisation sequence
#endif

enum progMode {
	pmIdle,
	pmSendBadPacket,
	pmSendHavePacket,
	pmSendReady,
	pmSendActive,
	pmSendFinished,
};

enum enflags{
	enDev0=1,
	enDev1=enDev0<<1,
	enDev2=enDev1<<1,
	enRQ=enDev2<<1,
	enRP=enRQ<<1,
	enI=enRP<<1,
	enW=enI<<1,
};

enum marker {
	maComplete,
	maResync,
	maInvalid,
	maOverflow,
};

const byte manc_enc[16] = { 0xAA, 0xA9, 0xA6, 0xA5, 0x9A, 0x99, 0x96, 0x95, 0x6A, 0x69, 0x66, 0x65, 0x5A, 0x59, 0x56, 0x55 };
const byte pre_sync[5] = { 0xFF, 0x00, 0x33, 0x55, 0x53 };
const byte header_flags[16] = { 0x0F, 0x0C, 0x0D, 0x0B, 0x27, 0x24, 0x25, 0x23, 0x47, 0x44, 0x45, 0x43, 0x17, 0x14, 0x15, 0x13 };

byte out_flags;
byte out_len;

byte in_header;
byte in_flags;

byte unpack_flags(byte header) {
	return header_flags[(header >> 2) & 0x0F];
}

byte pack_flags(byte flags) {
	for (byte n = 0; n < 16; n++) {
		if (header_flags[n] == flags) {
			return n << 2;
		}
	}
	return 0xFF;
}

#define MAX_PACKETS 4

struct recv_packet {
	boolean ready;
	uint8_t length;
	enum marker status;
	byte rssi;
	char freqOffset;
	byte data[128];
};
struct recv_packet recv_buffer[MAX_PACKETS] = { { false, 0, maInvalid, 0, 0, { 0 } } };
volatile uint8_t readIndex = 0;
volatile uint8_t writeIndex = 0;
volatile boolean available = false;
volatile boolean read_rssi = false;
volatile byte rssi;
volatile char freqOffset;
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
byte bm;

byte sp = 0;
byte op = 0;
byte pp = 0;

byte check = 0;
byte pkt_pos = 0;
byte devidCount = 0;
uint32_t devid;
uint32_t src_devid;
byte *pDev = (byte*)&devid;
uint16_t cmd;
char tmp[20];
byte len;
byte pos = 3;
char param[10];

char averageFrequencyOffset = 0; // use char for signed 8 bit type

void finish_recv_buffer(enum marker status) {
	if (recv_buffer[writeIndex].length > 0) {
		recv_buffer[writeIndex].status = status;
		recv_buffer[writeIndex].ready = true;
		recv_buffer[writeIndex].rssi = rssi;
		recv_buffer[writeIndex].freqOffset = freqOffset;
		rssi = 0;
		freqOffset = 0;
		available = true;

		writeIndex = (writeIndex + 1) % MAX_PACKETS;
		if (recv_buffer[writeIndex].ready) {
			recv_buffer[writeIndex].ready = false;
			readIndex = (readIndex + 1) % MAX_PACKETS;
		}
		recv_buffer[writeIndex].length = 0;
	}

	in_sync = false;
}

// Interrupt to receive data and find_sync_word
void sync_clk_in() {
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
		bm = 0x10;
		in_sync = true;
		read_rssi = true;
	} else if (in_sync) {
		if (bit_counter > 0) { // Skip start bit
			if (bit_counter < 9) {
				if (bit_counter % 2) {
					if (new_bit) {
						byte_buffer |= bm;
					}
					bm <<= 1;
					if (bm == 0x10) {
						if (recv_buffer[writeIndex].length == sizeof(recv_buffer[writeIndex].data)) {
							finish_recv_buffer(maOverflow);
							return;
						}
						// we can not see raw 0x35 here as our buffer is already manchester decoded we rely on rejection of 0x35 as it is not manchester encoded to end our packet
						recv_buffer[writeIndex].data[recv_buffer[writeIndex].length++] = byte_buffer;
						byte_buffer = 0;
					} else if (!bm) {
						bm = 0x01;
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
			if (out_flags < 5) {
				byte_buffer = 0x55;
				out_flags++;
			} else if (pp<5) {
				byte_buffer = pre_sync[pp++];
			} else {
				if (sp<op) {
					byte_buffer = send_buffer[sp];
					if (highnib) {
						byte_buffer = manc_enc[(byte_buffer >> 4) & 0xF];
					} else {
						if (recv_buffer[writeIndex].length < sizeof(recv_buffer[writeIndex].data)) {
							recv_buffer[writeIndex].data[recv_buffer[writeIndex].length++] = byte_buffer;
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
	delay(200); //probably not a bad idea to wait until power etc. have stabilised a little

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
	Serial.println(F("# EvohomeWirelessFW v" VERSION_NO " Copyright (c) 2015 Hydrogenetic"));
	Serial.println(F("# Licensed under GPL-3.0+ <http://spdx.org/licenses/GPL-3.0+>"));

	// Attach the find_sync_word interrupt function to the
	// falling edge of the serial clock connected to INT(1)
	attachInterrupt(GDO2_INT, sync_clk_in, FALLING);
}

void handleFreqOffset(struct recv_packet *packet, int ok) {
#if 0
	if (ok) {
		averageFrequencyOffset = filter(averageFrequencyOffset, averageFrequencyOffset + packet->freqOffset);
		CCx.Write(CCx_FSCTRL0, averageFrequencyOffset);
	}
#endif

	if (src_devid != 0) {
		sprintf(tmp,"# %02hu:%06lu", (uint8_t)(src_devid >> 18) & 0x3F, src_devid & 0x3FFFF);
		Serial.print(tmp);
		Serial.print(F(" RSSI="));
		Serial.print(packet->rssi, DEC);
		Serial.print(F(" FREQEST="));
		Serial.println(packet->freqOffset, DEC);
#if 0
		Serial.print(F(" FACCT="));
		Serial.println(averageFrequencyOffset, DEC);
#endif
	}
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
		CCx.Read(CCx_FREQEST, (byte*)&freqOffset);
		rssi = (rssi_tmp < 128) ? (rssi_tmp + 128) : (rssi_tmp - 128); // not the RSSI, but a scale of 0-255
		read_rssi = false;
	}
	if (available) {
		struct recv_packet packet = { false, 0, maInvalid, 0, 0, { 0 } };

		cli();
		if (available) {
			if (recv_buffer[readIndex].ready) {
				memcpy(&packet, &recv_buffer[readIndex], offsetof(struct recv_packet, data) + recv_buffer[readIndex].length);
				recv_buffer[readIndex].ready = false;
				readIndex = (readIndex + 1) % MAX_PACKETS;

				if (!recv_buffer[readIndex].ready) {
					available = false;
				}
			}
		}
		sei();

		if (!packet.ready) {
			return;
		}

		check = 0;
		pkt_pos = 0;
		pos = 3;

		for (uint8_t n = 0; n < packet.length; n++) {
			byte in = packet.data[n];

			check += in;
			if (pkt_pos == 0) {
				in_header = in;
				if ((in & 0xC0) || (in & 3) == 3) { //we don't recognise a header when 2 high reserved bits are set or both parameters bits are set simultaneously (we only have room for 1 parameter in our output - need more feedback could this be a parameter mode?)
					in_flags = 0;
				} else {
					in_flags = unpack_flags(in_header);
				}
				sprintf(tmp, "%03u ", packet.rssi);
				Serial.print(tmp);
				// sprintf(tmp, "%02X-%02X: ", in, in_flags);
				// Serial.print(tmp);
				if (in_flags & enI) {
					Serial.print(" I ");
				} else if (in_flags & enRQ) {
					Serial.print("RQ ");
				} else if (in_flags & enRP) {
					Serial.print("RP ");
				} else if (in_flags & enW) {
					Serial.print(" W ");
				} else {
					Serial.print(F("*Unknown header=0x"));
					Serial.println(in, HEX);
					return;
				}
				Serial.print("--- "); // parameter not supported... not been observed yet
				src_devid = 0;
				devidCount = (in_flags & enDev0) ? 1 : 0;
				if (in_flags & enDev1) devidCount++;
				if (in_flags & enDev2) devidCount++;
				pDev = (byte*)&devid + 2; // platform specific
			} else if (pkt_pos <= pos) { // ids, support 1 or 2 atm (1, 2 or 3 ids are valid)
				*pDev-- = in; // platform specific
				if (pkt_pos == 3) {
					if (!(in_flags & enDev0) && !(in_flags & enDev1)) {
						Serial.print("--:------ --:------ ");
					} else {
						pos += 3;
					}
					if (src_devid == 0) {
						src_devid = devid;
					}
					sprintf(tmp,"%02hu:%06lu ", (uint8_t)(devid >> 18) & 0x3F, devid & 0x3FFFF);
					Serial.print(tmp);
					pDev = (byte*)&devid + 2; // platform specific
				} else if (pkt_pos == 6) {
					if (!(in_flags & enDev1)) {
						Serial.print("--:------ ");
					}
					if (src_devid == 0) {
						src_devid = devid;
					}
					sprintf(tmp,"%02hu:%06lu ", (uint8_t)(devid >> 18) & 0x3F, devid & 0x3FFFF);
					Serial.print(tmp);
					if (!(in_flags & enDev2)) {
						Serial.print("--:------ ");
					}
					pDev = (byte*)&devid + 2;//platform specific
				}
			} else if (pkt_pos <= pos + 2) { // command
				if (pkt_pos == 4) { //Skip the value
					pos++;
				} else if (pkt_pos == pos + 1) {
					cmd = in << 8;
				} else if (pkt_pos == pos + 2) {
					cmd |= in;
					sprintf(tmp, "%04X ", cmd);
					Serial.print(tmp);
				}
			} else if (pkt_pos == pos + 3) { // len
				len = in;
				sprintf(tmp, "%03hu ", len);
				Serial.print(tmp);
			} else if (pkt_pos <= pos + 3 + len) { // payload
				sprintf(tmp, "%02hX", in);
				Serial.print(tmp);
			} else if (pkt_pos == pos + 4 + len) { // checksum
				if (check == 0) {
					Serial.println();
					handleFreqOffset(&packet, 1);
				} else {
					Serial.println(F("*CHK*"));
					handleFreqOffset(&packet, 0);
				}
				return;
			} else {
				Serial.println(F("*E-DATA*"));
				handleFreqOffset(&packet, 0);
				return;
			}
			pkt_pos++;
		}

		if (packet.status == maComplete) {
			Serial.println(F("*INCOMPLETE*END*"));
		} else if (packet.status == maResync) {
			Serial.println(F("*INCOMPLETE*RESYNC*"));
		} else if (packet.status == maInvalid) {
			Serial.println(F("*INCOMPLETE*INVALID*"));
		} else if (packet.status == maOverflow) {
			Serial.println(F("*INCOMPLETE*OVERFLOW*"));
		}

		handleFreqOffset(&packet, 0);
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
							out_flags = 0;
							if (!strcmp(param, "I")) {
								out_flags |= enI;
							} else if (!strcmp(param, "RQ")) {
								out_flags |= enRQ;
							} else if (!strcmp(param, "RP")) {
								out_flags |= enRP;
							} else if (!strcmp(param, "W")) {
								out_flags |= enW;
							}
							send_buffer[op++] = 0;
						} else if (sp == 1) {
						} else if (sp >= 2 && sp <= 4) {
							if (param[0] != '-') {
								out_flags |= 1 << (sp - 2);
								uint8_t idType;
								uint32_t idAddr;
								sscanf(param, "%02hhu:%06lu", &idType, &idAddr);
								idAddr |= (uint32_t)idType << 18;
								send_buffer[op++] = (idAddr >> 16) & 0xFF;
								send_buffer[op++] = (idAddr >> 8) & 0xFF;
								send_buffer[op++] = idAddr & 0xFF;
							}
						} else if (sp == 5) {
							send_buffer[0] |= pack_flags(out_flags);
							if (send_buffer[0] == 0xFF) {
								sm = pmSendBadPacket;
								return;
							}
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
			out_flags = 0; // reuse for preamble counter
			finish_recv_buffer(maComplete);
			attachInterrupt(GDO2_INT, sync_clk_out, RISING);
	}
}

char filter(char average, char sample) {
	int tmp = average / 8;
	average -= tmp;
	average += sample / 8;
	return average;
}
