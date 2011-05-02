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

#include <Golem/Device/Katana300/kmlBase.h>

#include <vector>

bool CKatBase::init(
    const TKatGNL _gnl,
    const TKatMOT _mot,
    const TKatSCT _sct,
    const TKatEFF _eff,
    CCplBase* _protocol) {

	//init vars
	gnl = _gnl;
	mot = _mot;
	sct = _sct;
	eff = _eff;

	protocol = _protocol;

	//init motors
	mot.arr = new CMotBase[mot.cnt];
	for (int m=0; m<mot.cnt; m++) {
		if (!mot.arr[m].init(this,mot.desc[m],protocol)) {
			delete[] mot.arr;
			return false;
		}
	}

	//check out whether the motors use new parameters
	recvNMP();
	//init sensor contollers
	sct.arr = new CSctBase[sct.cnt];
	for (int s=0; s<sct.cnt; s++) {
		if (!sct.arr[s].init(this,sct.desc[s],protocol)) {
			delete[] sct.arr;
			return false;
		}
	}
	return true;
}


void CKatBase::recvMFW() {
	byte	p[32];		//packet
	byte	buf[256];	//readbuf
	byte	sz = 0;		//readbuf size
	p[0] = 'B';

	protocol->comm(p,buf,&sz);

	mfw.ver = buf[1];
	mfw.rev = buf[2];

}

void CKatBase::recvIDS() {
	byte	p[32];		//packet
	byte	buf[256];	//readbuf
	byte	sz = 0;		//readbuf size
	p[0] = 'Y';

	protocol->comm(p,buf,&sz);

	memcpy(ids.strID,buf+1,sz-1);
	ids.strID[sz-3] = 0;

}

void CKatBase::recvCTB() {
	byte	p[32];		//packet
	byte	buf[256];	//readbuf
	byte	sz = 0;		//readbuf size
	p[0] = 'X';

	protocol->comm(p,buf,&sz);

	memcpy(ctb.cmdtbl,buf+1,sz-1);
	ctb.cmdtbl[sz-1] = 0;

}

//'G'et every 'M'otor's 'S'tatus flag
void CKatBase::recvGMS() {
	int i;
	byte	p[32];		//packet
	byte	buf[256];	//readbuf
	byte	sz = 0;		//readbuf size
	p[0] = 'N';
	p[1] = 1;
	p[2] = 0;

	protocol->comm(p,buf,&sz);

	for (i=0; i<mot.cnt; i++) {
		mot.arr[i].pvp.msf = (TMotStsFlg)buf[i+1];
	}
}

void CKatBase::recvECH() {
	byte	p[32];		//packet
	byte	buf[256];	//readbuf
	byte	sz = 0;		//readbuf size
	p[0] = 'Z';

	protocol->comm(p,buf,&sz);

	ech.echo =  buf[0];
	if (ech.echo != 'z') {
		throw ParameterReadingException("ECH");
	}
}

void CKatBase::recvCBX() {
	byte	p[32];		//packet
	byte	buf[256];	//readbuf
	byte	sz = 0;		//readbuf size
	p[0] = 'U';
	p[1] = 0;
	p[2] = 0;

	protocol->comm(p,buf,&sz);

	cbx.inp[0] = !(buf[2] & 4);
	cbx.inp[1] = !(buf[2] & 32);

}

void CKatBase::sendCBX(const TKatCBX* _cbx) {
	byte	p[32];		//packet
	byte	buf[256];	//readbuf
	byte	sz = 0;		//readbuf size

	p[0] = 'U';
	p[1] = 0;

	p[2] = _cbx->out[0] ? 52 : 36;
	protocol->comm(p,buf,&sz);

	p[2] = _cbx->out[1] ? 53 : 37;
	protocol->comm(p,buf,&sz);

	cbx.out[0] = _cbx->out[0];
	cbx.out[1] = _cbx->out[1];

}

void CKatBase::sendTPSP() {

	byte	p[32];					//packet
	byte	buf[256];				//readbuf
	byte	sz = 0;					//readbuf size
	byte	wm = 0;					// which motor should be moved
	TMotCmdFlg comm_flag = MCF_ON;	// motor status after moving
	byte	max_accel = 1;			// max_accel

	for (int i=0; i<5; i++) {		// parallel movements only work for 5
		if (mot.arr[i].freedom)		// motors
			wm |= 0x01 << i;		// sets the motors that should be moved by
		// setting the proper bits
	};

	p[0] = 'P';						// command
	p[1] = 1;						// always 1
	p[2] = wm;						// which motor should be moved
	p[3] = comm_flag;				// motor status after moving
	p[4] = max_accel;

	for (int i=1; i<=5; i++) {
		p[4*i+1] = (byte)(mot.arr[i-1].GetTPS()->tarpos >> 8);
		p[4*i+2] = (byte)(mot.arr[i-1].GetTPS()->tarpos);
		if (mot.arr[i-1].nmp) { // check whether the motor use the new parameters
			p[4*i+3] = (byte)(mot.arr[i-1].GetDYL()->maxpspeed_nmp >> 8);
			p[4*i+4] = (byte)(mot.arr[i-1].GetDYL()->maxpspeed_nmp);
		} else {
			p[4*i+3] = (byte)(mot.arr[i-1].GetDYL()->maxpspeed >> 8);
			p[4*i+4] = (byte)(mot.arr[i-1].GetDYL()->maxpspeed);
		};
	}

	protocol->comm(p,buf,&sz);

}

void CKatBase::recvNMP() {
	byte	p[32];		//packet
	byte	buf[256];	//readbuf
	byte	sz = 0;		//readbuf size
	p[0] = 'A';
	p[1] = 0;
	p[2] = 1;

	try {
		protocol->comm(p,buf,&sz);
	} catch ( ReadWriteNotCompleteException & ) { // this exception is not fatal
		for (int i=0; i<mot.cnt; i++) {
			mot.arr[i].nmp = false;
		}
		return; // return here
	} catch ( Exception & ) {
		throw;
	}

	for (int i=0; i<mot.cnt; i++) {
		mot.arr[i].nmp = true;
	}

}

void CKatBase::getMasterFirmware(short* fw, short* rev) {
	*fw = mMasterVersion;
	*rev = mMasterRevision;
}

void CKatBase::enableCrashLimits() {
	// adjust second byte of packet according to katana model
	short version, revision;
	int motor = 1; // master ON with KatHD300
	getMasterFirmware(&version, &revision);
	if (checkKatanaType(400)) {
		motor = 0; // switch all motors with KatHD400
	}
	byte	p[32];		//packet
	byte	buf[256];	//readbuf
	byte	sz = 0;		//readbuf size
	p[0] = 'A';
	p[1] = motor;
	p[2] = 1;
	protocol->comm(p,buf,&sz);
}

void CKatBase::disableCrashLimits() {

	byte	p[32];		//packet
	byte	buf[256];	//readbuf
	byte	sz = 0;		//readbuf size
	p[0] = 'A';
	p[1] = 0;
	p[2] = 0;

	protocol->comm(p,buf,&sz);


}
//deprecated, use speed & position
void CKatBase::setCrashLimit(long idx, int limit) {

	byte	p[32];		//packet
	byte	buf[256];	//readbuf
	byte	sz = 0;		//readbuf size
	p[0] = 'S';
	p[1] = 5;			// subcommand 5 "Set Crashlimit"
	p[2] = (char)(limit >> 8);
	p[3] = (char)(limit);
	p[4] = 0;
	protocol->comm(p,buf,&sz);
}
////////////////////////////////////////////////////////////////
void CKatBase::setPositionCollisionLimit(long idx, int limit){
	mot.arr[idx].setPositionCollisionLimit(limit);
}
////////////////////////////////////////////////////////////////
void CKatBase::setSpeedCollisionLimit(long idx, int limit){
	mot.arr[idx].setSpeedCollisionLimit(limit);
}
////////////////////////////////////////////////////////////////

void CKatBase::unBlock() {

	for (int i=0; i < mot.cnt; ++i) {
		mot.arr[i].resetBlocked();
	}
}

void CKatBase::sendSLMP(byte* p) {
	// Set Linear movements parameters

	byte	buf[256];	//readbuf
	byte	sz = 0;		//readbuf size

	protocol->comm(p,buf,&sz);

}

void CKatBase::sendSLM(bool exactflag) {
	// Start Linear movement

	byte	p[3];		//packet
	byte	buf[256];	//readbuf
	byte	sz = 0;		//readbuf size

	p[0] = 204; // 'L'+128
	p[1] = 1;
	p[2] = exactflag ? 1 : 0;

	protocol->comm(p,buf,&sz);

}

void CKatBase::startSplineMovement(bool exactflag, int moreflag) {
	// Start Linear movement

	std::vector<byte> sendBuf(3), recvBuf(2, 0);
	byte readBytes;

	sendBuf[0] ='G'+128 ;
	sendBuf[1] = (byte) moreflag;
	sendBuf[2] = exactflag ? 1 : 0;

	protocol->comm(&sendBuf.front(), &recvBuf.front(), &readBytes);

}

void CKatBase::startFourSplinesMovement(bool exactflag) {
	// Start Linear movement

	std::vector<byte> sendBuf(3), recvBuf(2, 0);
	byte readBytes;

	sendBuf[0] ='L'+128 ;
	sendBuf[1] = 1;
	sendBuf[2] = exactflag ? 1 : 0;

	protocol->comm(&sendBuf.front(), &recvBuf.front(), &readBytes);
}

void CKatBase::recvMPS() {

	byte	p[32];		//packet
	byte	buf[256];	//readbuf
	byte	sz = 0;		//readbuf size
	p[0] = 'N';
	p[1] = 3;
	p[2] = 0;

	protocol->comm(p,buf,&sz);

	for (int i=0; i<mot.cnt; i++) {
		mot.arr[i].pvp.pos = (((short)buf[2*i+1]) <<8) | buf[2*i+2];
	}

}
int CKatBase::checkKatanaType(int type){
	recvMFW();
	if(type == 400){
		if(mfw.ver > K400_OLD_PROTOCOL_THRESHOLD){
			return -1;
		}
	}
	else if(type == 300){
		if(mfw.ver < K400_OLD_PROTOCOL_THRESHOLD){
			return -1;
		}
	}
	return 1;
}
