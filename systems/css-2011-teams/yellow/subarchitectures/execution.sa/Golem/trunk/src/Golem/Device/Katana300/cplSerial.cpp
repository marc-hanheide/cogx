/*
 *   Katana Native Interface - A C++ interface to the robot arm Katana.
 *   Copyright (C) 2005 Neuronics AG
 *   Check out the AUTHORS file for detailed contact information.
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <Golem/Device/Katana300/cplSerial.h>
#include <Golem/Device/Katana300/CRC.h>
#include <assert.h>
#include <iostream>


/*
bool CCplSerialZero::load_tbl() {

	// set table to zero -----------------------------------//

	memset(cmd,0,sizeof(cmd));

	// set command table from the katana -------------------//

	cmd[(int)'B'].send_sz	= 1;
	cmd[(int)'B'].read_sz	= 3;
	cmd[(int)'X'].send_sz	= 1;
	cmd[(int)'X'].read_sz	= 127;
	cmd[(int)'Y'].send_sz	= 1;
	cmd[(int)'Y'].read_sz	= 84;
	cmd[(int)'Z'].send_sz	= 1;
	cmd[(int)'Z'].read_sz	= 1;
	cmd[(int)'K'].send_sz	= 11;
	cmd[(int)'K'].read_sz	= 2;
	cmd[(int)'O'].send_sz	= 12;
	cmd[(int)'O'].read_sz	= 2;
	cmd[(int)'C'].send_sz	= 5;
	cmd[(int)'C'].read_sz	= 3;
	cmd[(int)'D'].send_sz	= 2;
	cmd[(int)'D'].read_sz	= 8;
	cmd[(int)'U'].send_sz	= 3;
	cmd[(int)'U'].read_sz	= 5;
	cmd[(int)'E'].send_sz	= 2;
	cmd[(int)'E'].read_sz	= 18;
	cmd[(int)'F'].send_sz	= 12;
	cmd[(int)'F'].read_sz	= 2;
	cmd[(int)'V'].send_sz	= 3;
	cmd[(int)'V'].read_sz	= 13;
	cmd[(int)'N'].send_sz	= 3;
	cmd[(int)'N'].read_sz	= 13;
	cmd[(int)'P'].send_sz	= 25;
	cmd[(int)'P'].read_sz	= 3;
	cmd[(int)'Q'].send_sz	= 7;
	cmd[(int)'Q'].read_sz	= 2;
	cmd[(int)'E'+128].send_sz	= 2;
	cmd[(int)'E'+128].read_sz	= 18;
	cmd[(int)'L'].send_sz	= 38;
	cmd[(int)'L'].read_sz	= 2;
	cmd[(int)'L'+128].send_sz	= 3;
	cmd[(int)'L'+128].read_sz	= 2;
	cmd[(int)'G'].send_sz   = 14;
	cmd[(int)'G'].read_sz   = 2;
	cmd[(int)'A'].send_sz	= 3;
	cmd[(int)'A'].read_sz	= 2;
	cmd[(int)'S'].send_sz	= 6;
	cmd[(int)'S'].read_sz	= 6;
	cmd[(int)'R'].send_sz	= 3;
	cmd[(int)'R'].read_sz	= 3;

	//------------------------------------------------------//

	return true;
}

void CCplSerialZero::defineProtocol(byte _kataddr) {
	hdr.size = 19;
	for (int i=0; i<16; i++) {	//fill header
		hdr.data[i] = 0;	//16x zero
	} hdr.data[16] = 1;		//convention
	hdr.data[17] = _kataddr;	//complete header
}

bool CCplSerialZero::init(CCdlBase* _device, byte _kataddr) {
	device = _device;
	defineProtocol(_kataddr);
	return load_tbl();
}

TRetCOMM CCplSerialZero::comm(const byte* _pack, byte* _buf, byte* _size) {
	//------------------------------------------------------------------//

	memset(send_buf,0,256);				//override old values
	hdr.data[hdr.size-1] = cmd[_pack[0]].send_sz;	//complete header
	memcpy(send_buf, hdr.data, hdr.size);
	memcpy(send_buf+hdr.size,_pack,hdr.data[hdr.size-1]);
	device->send(send_buf,hdr.size+hdr.data[hdr.size-1]);	//send to dev
	if (device->lastOP() != lopDONE) {
		return COMM_RTOERR;			//comm failed
	}

	//------------------------------------------------------------------//

	memset(read_buf,0,256);				//read through device
	*_size = device->recv(read_buf,cmd[_pack[0]].read_sz);
	if (device->lastOP() != lopDONE) {
		return COMM_RTOERR;			//comm failed
	}

	memcpy(_buf,read_buf,*_size);			//read_buf to _buf
	return COMM_NO_ERR;				//yeah :)

	//------------------------------------------------------------------//
}
*/


bool CCplSerialCRC::load_tbl() {

	bool status = true;

	// set table to zero
	memset(cmd,0,sizeof(cmd));

	// set a tentative command table for the katana
	cmd[(int)'B'].send_sz	= 1; // fixed to 1, used to actualize table
	cmd[(int)'B'].read_sz	= 3; // fixed to 3, used to actualize table
	cmd[(int)'X'].send_sz	= 1; // fixed to 1, used to actualize table
	cmd[(int)'X'].read_sz	= 139; // adjusted according to master firmware version
	cmd[(int)'Y'].send_sz	= 1;
	cmd[(int)'Y'].read_sz	= 84;
	cmd[(int)'Z'].send_sz	= 1;
	cmd[(int)'Z'].read_sz	= 1;
	cmd[(int)'K'].send_sz	= 11;
	cmd[(int)'K'].read_sz	= 2;
	cmd[(int)'O'].send_sz	= 12;
	cmd[(int)'O'].read_sz	= 2;
	cmd[(int)'C'].send_sz	= 5;
	cmd[(int)'C'].read_sz	= 3;
	cmd[(int)'D'].send_sz	= 2;
	cmd[(int)'D'].read_sz	= 8;
	cmd[(int)'U'].send_sz	= 3;
	cmd[(int)'U'].read_sz	= 5;
	cmd[(int)'E'].send_sz	= 2;
	cmd[(int)'E'].read_sz	= 18;
	cmd[(int)'F'].send_sz	= 12;
	cmd[(int)'F'].read_sz	= 2;
	cmd[(int)'V'].send_sz	= 3;
	cmd[(int)'V'].read_sz	= 13;
	cmd[(int)'N'].send_sz	= 3;
	cmd[(int)'N'].read_sz	= 13;
	cmd[(int)'P'].send_sz	= 25;
	cmd[(int)'P'].read_sz	= 3;
	cmd[(int)'Q'].send_sz	= 7;
	cmd[(int)'Q'].read_sz	= 2;
	cmd[(int)'E'+128].send_sz	= 2;
	cmd[(int)'E'+128].read_sz	= 18;
	cmd[(int)'L'].send_sz	= 38;
	cmd[(int)'L'].read_sz	= 2;
	cmd[(int)'L'+128].send_sz	= 3;
	cmd[(int)'L'+128].read_sz	= 2;
	cmd[(int)'G'].send_sz	= 14;	// UPDATE @@@
	cmd[(int)'G'].read_sz	= 2;	// UPDATE @@@
	cmd[(int)'G'+128].send_sz	= 3;	// UPDATE @@@
	cmd[(int)'G'+128].read_sz	= 2;	// UPDATE @@@
	cmd[(int)'A'].send_sz	= 3;
	cmd[(int)'A'].read_sz	= 2;
	cmd[(int)'S'].send_sz	= 6;
	cmd[(int)'S'].read_sz	= 6;
	cmd[(int)'R'].send_sz	= 3;
	cmd[(int)'R'].read_sz	= 3;
	cmd[(int)'I'].send_sz	= 2;
	cmd[(int)'I'].read_sz	= 3;
	cmd[(int)'M'].send_sz	= 5;
	cmd[(int)'M'].read_sz	= 4;
	cmd[(int)'T'].send_sz	= 5;
	cmd[(int)'T'].read_sz	= 2;

	// variable declarations
	byte	packet[32]; //packet
	byte	buffer[256]; //readbuf
	byte	size = 0; //readbuf size

	// get the master firmware version from the katana and actualize X.read_sz
	packet[0] = 'B';
	//std::cout << "about to get master firmware version from the katana" << std::endl; // TEST
	comm(packet, buffer, &size);
	//std::cout << "about to get master firmware version from the katana [comm returned]" << std::endl; // TEST
	if (((short) size != 3) || (buffer[0] != 'b')) {
		status = false;
	} else {
		short version = (short) buffer[1];
		short revision = (short) buffer[2];
		mMasterVersion = version;
		mMasterRevision = revision;
		// FW - 4.15: katana command table length is 121
		if ((version == 4) && (revision <= 15)) {
			cmd[(int)'X'].read_sz	= 121;
		}
		// FW 4.16 - 4.18: katana command table length is 127
		else if ((version == 4) && (revision <= 18)) {
			cmd[(int)'X'].read_sz	= 127;
		}
		// FW from 4.19 (Katana 1.1): katana command table length is 139
		else if ((version == 4) && (revision <= 39)) {
			cmd[(int)'X'].read_sz	= 139;
		}
		// FW from 4.40 (Katana 1.2): katana command table length is 181
		// ! empty records allowed !
		else if ((version >= 4) && (revision >= 40)) {
			cmd[(int)'X'].read_sz	= 181;
		}
		// FW from 0.x (Katana 1.2): katana command table length is 181
		// ! empty records allowed !
		else if (version == 0) {
			cmd[(int)'X'].read_sz	= 181;
		}
	}

/* // FIXME: uncomment to load the whole table from the katana
	// get the katana command table and actualize the tentative table
    int i, katana_entry_size, cmd_entry_size;
    byte command;
	packet[0] = 'X';
	comm(packet, buffer, &size);
	if ((buffer[0] != 'x') || ((short) size != cmd[(int)'X'].read_sz)) {
		status = false;
	} else {
		for (i = 0; i < ((cmd[(int)'X'].read_sz - 1) / 6); i++) {
			if (!buffer[6*i+1]) {
				break;
			}
			command = buffer[6*i+2];
			katana_entry_size = buffer[6*i+4];
			cmd_entry_size = cmd[(int)command].send_sz;
			if (cmd_entry_size != katana_entry_size) {
				cmd[(int)command].send_sz	= katana_entry_size;
//std::cout << (char) command<< ".send_sz changed to " << katana_entry_size << std::endl;
			}
			katana_entry_size = buffer[6*i+5];
			cmd_entry_size = cmd[(int)command].read_sz;
			if (cmd_entry_size != katana_entry_size) {
				// hack, because X.read_sz not always correct in received table
				if (command != 'X') {
					cmd[(int)command].read_sz	= katana_entry_size;
//std::cout << (char) command<< ".read_sz changed to " << katana_entry_size << std::endl;
				}
			}
		}
	}
*/
	return status;
}

void CCplSerialCRC::defineProtocol(byte _kataddr) {
	hdr.size = 3;
	hdr.data[0] = 1;	//convention
	hdr.data[1] = _kataddr;	//complete header
}

bool CCplSerialCRC::init(CCdlBase* _device, byte _kataddr) {
	device = _device;
	defineProtocol(_kataddr);
	return load_tbl();
}


void CCplSerialCRC::comm(const byte* pack, byte* buf, byte* size) {

	// This method enssamble the packet with the header, data, and CRC.
	// Sends it and receives the answer.

	const char cmdchar = pack[0];
	//std::cout << "CCplSerialCRC::comm: " << cmdchar << std::endl; // TEST

	memset(send_buf,0,256);							//override old values
	hdr.data[hdr.size-1] = cmd[pack[0]].send_sz;	//complete header
	memcpy(send_buf, hdr.data, hdr.size);
	memcpy(send_buf+hdr.size,pack,hdr.data[hdr.size-1]);

	short crc   = CRC_CHECKSUM((uint8*)pack,hdr.data[hdr.size-1]);
	byte  bufsz = hdr.size + hdr.data[hdr.size-1];
	send_buf[bufsz++] = (byte)(crc >> 8);			//hi-byte
	send_buf[bufsz++] = (byte)(crc & 0xFF);			//lo-byte

	memset(read_buf,0,256);							//read through device
	byte read_sz = cmd[pack[0]].read_sz + 2;

	short tries_recv = 0;
	while(true) {

		send(send_buf, bufsz, NUMBER_OF_RETRIES_SEND); // pass exceptions from this since already retried

		try {
			recv(read_buf,read_sz,size);
			memcpy(buf,read_buf,*size); // copy read_buf to _buf
		} catch ( ReadNotCompleteException & ) {
			if(tries_recv < NUMBER_OF_RETRIES_RECV) {
				++tries_recv;
				continue;
			}
			throw;

		} catch ( WrongCRCException & ) {
			if(tries_recv < NUMBER_OF_RETRIES_RECV) {
				++tries_recv;
				continue;
			} else {
				throw;
			}
		} catch ( FirmwareException & ) {
			throw;
		} catch ( Exception & ) {
			// FIXME why? --MRE
// 			if(errorFlag == true){
// 			}
		}

		break; // write succeeded

	}
}

void CCplSerialCRC::getMasterFirmware(short* fw, short* rev) {
	*fw = mMasterVersion;
	*rev = mMasterRevision;
}

void CCplSerialCRC::send(byte* send_buf, byte bufsz, short retries) {
	short r = 0;
	while(true) {
		try {
			device->send(send_buf, bufsz); //send to device
		} catch ( WriteNotCompleteException & ) {
			if(r < retries) {
				++r;
				continue;
			} else {
				throw;
			}
		} catch ( Exception & ) {
			throw; // throw other exceptions immediately
		}
		break;
	}
}

void CCplSerialCRC::recv(byte* read_buf, byte read_sz, byte* size) {

	// Receives the packet and checks the CRC.
	*size = device->recv(read_buf, read_sz);		//receives from device

	bool getErrorMessage = false;
	//check for error from Katana:
	if(read_buf[0] == KATANA_ERROR_FLAG){
		std::cout << "Error flag received:\n";
                assert(*size == 3);
		getErrorMessage = true;
		read_sz = *size; // FIXME: should not return the now invalid buffer?
	} else {
		if (*size != read_sz) {
			throw ReadNotCompleteException("?"); // FIXME: should get _ipAddr for nicer error message
		}
        }

	*size -= 2;
	byte bhi = read_buf[read_sz-2];
	byte blo = read_buf[read_sz-1];

	short crc = CRC_CHECKSUM((uint8*)read_buf,*size);
	byte hi = (byte)(crc >> 8);
	byte lo = (byte)(crc & 0xFF);

	if ((hi != bhi) || (lo != blo)) {
		std::cout << "warning: crc error, throwing exception" << std::endl;
		throw WrongCRCException();
	}

	if (getErrorMessage) {
                byte buf[57];
                buf[0] = 0; // ignored
                buf[1] = 0; // ignored
                buf[2] = 0; // ignored
		buf[3] = KATANA_ERROR_FLAG+1;
		try {
			send(buf, 4, 1);
			byte size_errormsg = 57;
			recv(buf, 57, &size_errormsg);
		} catch (...) {
			std::cout << "Error while requesting error message!\n";
		}

		if (buf[0] != KATANA_ERROR_FLAG+1) {
			std::cout << "bad response to error request\n";
		}
		byte lastCommand = buf[1];
		byte errorCode = buf[2];
		byte device = buf[3];
		std::string errorString((char*)buf+4);
                if (device != 0) {
                        errorString += " (axis ";
                        errorString += '0' + device;
                        errorString += ")";
                }
		//std::cout << "\"" << errorString << "\"\n";
                throw FirmwareException(errorString, static_cast<signed char>(errorCode), device, lastCommand);
	}
}
