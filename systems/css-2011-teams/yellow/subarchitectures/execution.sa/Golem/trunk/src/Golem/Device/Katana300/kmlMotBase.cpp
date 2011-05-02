//
// C++ Implementation: kmlMotBase
//
// Description:
//
//
// Author: Tiziano Müller <tiziano.mueller@neuronics.ch>, (C) 2006
//
// Copyright: See COPYING file that comes with this distribution
//
//

#include <Golem/Device/Katana300/kmlMotBase.h>

#include <Golem/Device/Katana300/MathHelperFunctions.h>
#include <Golem/Device/Katana300/Timer.h>
#include <iostream>


bool CMotBase::init(CKatBase* _own, const TMotDesc _motDesc, CCplBase* _protocol) {
	gnl.own = _own;
	gnl.SID = _motDesc.slvID;
	protocol =  _protocol;
	return true;
}


void CMotBase::resetBlocked() {
	byte	p[32];		//packet
	byte	buf[256];	//readbuf
	byte	sz = 0;		//readbuf size

	recvPVP();

	p[0] = 'C';
	p[1] = gnl.SID;
	p[2] = MCF_FREEZE;			// Flag to freeze
	p[3] = (byte)(GetPVP()->pos >> 8);
	p[4] = (byte)(GetPVP()->pos);

	protocol->comm(p,buf,&sz);

	aps.mcfAPS = MCF_FREEZE;
}

void CMotBase::sendAPS(const TMotAPS* _aps) {
	byte	p[32];		//packet
	byte	buf[256];	//readbuf
	byte	sz = 0;		//readbuf size

	p[0] = 'C';
	p[1] = gnl.SID + 128;
	p[2] = _aps->mcfAPS;
	p[3] = (byte)(_aps->actpos >> 8);
	p[4] = (byte)(_aps->actpos);

	protocol->comm(p,buf,&sz);

	if (!buf[1])
		throw ParameterWritingException("APS");

	aps = *_aps;

}

void CMotBase::sendTPS(const TMotTPS* _tps) {
	byte	p[32];		//packet
	byte	buf[256];	//readbuf
	byte	sz = 0;		//readbuf size

	p[0] = 'C';
	p[1] = gnl.SID;
	p[2] = _tps->mcfTPS;
	p[3] = (byte)(_tps->tarpos >> 8);
	p[4] = (byte)(_tps->tarpos);

	protocol->comm(p,buf,&sz);
	if (!buf[1])
		throw ParameterWritingException("TPS");
	tps = *_tps;
}

void CMotBase::sendSCP(const TMotSCP* _scp) {

	Context c("CMotBase::sendSCP");

	const TMotSCP* P_SCP;
	const short NUMBER_OF_RETRIES = 3;

	byte	p[32];		//packet
	byte	buf[256];	//readbuf
	byte	sz = 0;		//readbuf size

	if (nmp) {

		// ----------------- New motor parameters ------------------ //
		for (short c = 0; c < NUMBER_OF_RETRIES; ++c) {

			p[0] = 'K';
			p[1] = gnl.SID;
			p[2] = _scp->maxppwm_nmp;
			p[3] = _scp->maxnpwm_nmp;
			p[4] = _scp->kspeed_nmp;
			p[5] = _scp->kpos_nmp;
			p[6] = _scp->kI_nmp;
			p[7] = _scp->crash_limit_nmp >> 8;
			p[8] = _scp->crash_limit_nmp;
			p[9] = _scp->crash_limit_lin_nmp >> 8;
			p[10]= _scp->crash_limit_lin_nmp;

			protocol->comm(p,buf,&sz);

			recvSCP();

			P_SCP = GetSCP();

			if (P_SCP->maxppwm_nmp ==_scp->maxppwm_nmp && P_SCP->maxnpwm_nmp ==_scp->maxnpwm_nmp &&
			        P_SCP->kspeed_nmp ==_scp->kspeed_nmp && P_SCP->kpos_nmp ==_scp->kpos_nmp && P_SCP->kI_nmp ==_scp->kI_nmp) {
				scp = *_scp;
				return;
			}

		} // end for

	} else {

		// ----------------- Old motor parameters ------------------ //
		for (short c = 0; c < NUMBER_OF_RETRIES; ++c) {

			p[0] = 'K';
			p[1] = gnl.SID;
			p[2] = _scp->maxppwm;
			p[3] = _scp->maxnpwm;
			p[4] = _scp->kP;
			p[5] = _scp->kI;
			p[6] = _scp->kD;
			p[7] = _scp->kARW;
			p[8] = _scp->kP_speed;
			p[9] = _scp->kI_speed;
			p[10]= _scp->kD_speed;

			protocol->comm(p,buf,&sz);

			recvSCP();

			P_SCP = GetSCP();

			// old firmwares loaded on PIC16 set by default maxppwm and maxnpwm to 70 regardless of the value
			// sent with the command 'k' (actually is set to 70 when this value is higher than 70).

			if (       (P_SCP->maxppwm ==_scp->maxppwm || P_SCP->maxppwm == 70)
			           && (P_SCP->maxnpwm ==_scp->maxnpwm || P_SCP->maxnpwm == 70)
			           && P_SCP->kP ==_scp->kP
			           && P_SCP->kI ==_scp->kI
			           && P_SCP->kD ==_scp->kD
			           && P_SCP->kP_speed ==_scp->kP_speed
			   ) {
				scp = *_scp;
				return;
			}

		} // end for

	} // end if

	throw WrongParameterException("SCP");
}

void CMotBase::recvSCP() {

	Context c("CMotBase::recvSCP");

	byte	p[32];		//packet
	byte	buf[256];	//readbuf
	byte	sz = 0;		//readbuf size

	p[0] = 'V';
	p[1] = gnl.SID;
	p[2] = 1;

	protocol->comm(p,buf,&sz);

	if (!buf[1])
		throw ParameterReadingException("SCP");

	if (nmp) {
		scp.maxppwm_nmp			= buf[3];
		scp.maxnpwm_nmp			= buf[4];
		scp.kspeed_nmp			= buf[5];
		scp.kpos_nmp			= buf[6];
		scp.kI_nmp				= buf[7];
		scp.crash_limit_nmp		= buf[8];
		scp.crash_limit_nmp		<<= 8;
		scp.crash_limit_nmp		+= buf[9];
		scp.crash_limit_lin_nmp	= buf[10];
		scp.crash_limit_lin_nmp	<<= 8;
		scp.crash_limit_lin_nmp	+= buf[11];

	} else {
		scp.maxppwm	= buf[3];
		scp.maxnpwm	= buf[4];
		scp.kP		= buf[5];
		scp.kI		= buf[6];
		scp.kD		= buf[7];
		scp.kARW	= buf[8];
		scp.kP_speed= buf[9];
		scp.kI_speed= buf[10];
		scp.kD_speed= buf[11];
	}

}

void CMotBase::sendDYL(const TMotDYL* _dyl) {

	// If the parameters are set wrong, this method retries "number_of_retries"
	// times to set them

	const TMotDYL* P_DYL;
	const short NUMBER_OF_RETRIES = 3;

	byte	p[32];		//packet
	byte	buf[256];	//readbuf
	byte	sz = 0;		//readbuf size


	if (nmp) {

		for ( short c = 0; c < NUMBER_OF_RETRIES; ++c) {

			p[0] = 'O';
			p[1] = gnl.SID;
			p[2] = _dyl->maxaccel_nmp;
			p[3] = 0;
			p[6] = (byte)(_dyl->maxpspeed_nmp >> 8);
			p[7] = (byte)(_dyl->maxpspeed_nmp);
			p[8] = (byte)(_dyl->maxnspeed_nmp >> 8);
			p[9] = (byte)(_dyl->maxnspeed_nmp);
			p[10] = (byte) 0;
			p[11] = (byte) _dyl->maxcurr_nmp;

			protocol->comm(p,buf,&sz);
			recvDYL();
			P_DYL = GetDYL();

			if (P_DYL->maxaccel_nmp ==_dyl->maxaccel_nmp && P_DYL->maxpspeed_nmp ==_dyl->maxpspeed_nmp &&
			        P_DYL->maxnspeed_nmp ==_dyl->maxnspeed_nmp) {
				dyl = *_dyl;
				return;
			}

		} // end for

	} else {

		for ( short c = 0; c < NUMBER_OF_RETRIES; ++c) {

			// Package format compatible since firmware version 40.2 for masterboard's PIC16
			// and version 13.5 for PIC16 microcontrolers within the slave boards
			// Compatible with all the firmwares for PIC18 with PID parameters

			p[0] = 'O';
			p[1] = gnl.SID;
			p[2] = _dyl->maxaccel;
			p[3] = _dyl->maxdecel;
			p[6] = (byte)(_dyl->maxpspeed >> 8);
			p[7] = (byte)(_dyl->maxpspeed);
			p[8] = (byte)(_dyl->maxnspeed >> 8);
			p[9] = (byte)(_dyl->maxnspeed);
			p[10] = (byte) 0;
			p[11] = (byte) _dyl->maxcurr;

			protocol->comm(p,buf,&sz);

			recvDYL();

			P_DYL = this->GetDYL();

			if (P_DYL->maxcurr ==_dyl->maxcurr && P_DYL->maxnspeed ==_dyl->maxnspeed &&
			        P_DYL->maxpspeed ==_dyl->maxpspeed) {
				dyl = *_dyl;
				return;
			}

		} // end for

	} // end if


	throw WrongParameterException("DYL");
}

void CMotBase::recvDYL() {
	byte	p[32];		//packet
	byte	buf[256];	//readbuf
	byte	sz = 0;		//readbuf size

	p[0] = 'V';
	p[1] = gnl.SID;
	p[2] = 2;

	protocol->comm(p,buf,&sz);

	if (!buf[1])
		throw ParameterReadingException("DYL");

	if (nmp) {

		dyl.maxaccel_nmp	= buf[3];
		dyl.maxpspeed_nmp	= (((short)buf[5]) <<8) | buf[6];
		dyl.maxnspeed_nmp	= (((short)buf[7]) <<8) | buf[8];
		dyl.maxcurr_nmp		= buf[9];

	} else {

		// Package format compatible since firmware version 40.2 for masterboard's PIC16
		// and version 13.5 for PIC16 microcontrolers within the slave boards
		// Compatible with all the firmwares for PIC18 with PID parameters

		dyl.maxaccel	= buf[3];
		dyl.maxdecel	= 0;
		dyl.minpos		= 0;
		dyl.maxpspeed	= (((short)buf[7]) <<8) | buf[8];
		dyl.maxnspeed	= (((short)buf[9]) <<8) | buf[10];
		dyl.maxcurr		= buf[11];
		dyl.actcurr		= 0;

	}
}

void CMotBase::recvPVP() {

	byte	p[32];		//packet
	byte	buf[256];	//readbuf
	byte	sz = 0;		//readbuf size

	p[0] = 'D';
	p[1] = gnl.SID;

	protocol->comm(p,buf,&sz);
	if (!buf[1])
		throw ParameterReadingException("PVP");
	pvp.msf		= (TMotStsFlg)buf[2];
	pvp.pos		= (((short)buf[3])<<8) | buf[4];
	pvp.vel		= (((short)buf[5])<<8) | buf[6];
	pvp.pwm		= buf[7];

}

void CMotBase::recvSFW() {
	byte	p[32];		//packet
	byte	buf[256];	//readbuf
	byte	sz = 0;		//readbuf size

	p[0] = 'V';
	p[1] = gnl.SID;
	p[2] = 32;

	protocol->comm(p,buf,&sz);
	if (!buf[1])
		throw ParameterReadingException("SFW");
	sfw.version	= buf[3];
	sfw.subversion	= buf[4];
	sfw.revision	= buf[5];
	sfw.type	= buf[6];
	sfw.subtype	= buf[7];

}


void CMotBase::setSpeedLimits(short positiveVelocity, short negativeVelocity) {

	byte p[32];		//packet
	byte buf[256];	//readbuf
	byte sz = 0;	//readbuf size
	p[0] = 'S';
	p[1] = gnl.SID;
	p[2] = 3;		// subcommand 3 "Set Speed Limits"
	p[3] = static_cast<byte>(positiveVelocity);
	p[4] = static_cast<byte>(negativeVelocity);
	p[5] = 0;

	protocol->comm(p,buf,&sz);

	dyl.maxnspeed = dyl.maxnspeed_nmp = negativeVelocity;
	dyl.maxpspeed = dyl.maxpspeed_nmp = positiveVelocity;

}

void CMotBase::setAccelerationLimit( short acceleration ) {

	byte p[32];		//packet
	byte buf[256];	//readbuf
	byte sz = 0;	//readbuf size
	p[0] = 'S';
	p[1] = gnl.SID;
	p[2] = 4;		// subcommand 4 "Set Acceleration Limit"
	p[3] = static_cast<byte>(acceleration);
	p[4] = 0;
	p[5] = 0;

	protocol->comm(p,buf,&sz);
	dyl.maxaccel_nmp = dyl.maxaccel = static_cast<byte>(acceleration);
}

void CMotBase::setPwmLimits(byte maxppwm, byte maxnpwm) {

	byte p[32];		//packet
	byte buf[256];	//readbuf
	byte sz = 0;	//readbuf size
	p[0] = 'S';
	p[1] = gnl.SID;
	p[2] = 2;		// subcommand 2 "Set PWM Limits"
	p[3] = maxppwm;
	p[4] = maxnpwm;
	p[5] = 0;

	protocol->comm(p,buf,&sz);
	scp.maxppwm_nmp = scp.maxppwm = maxppwm;
	scp.maxnpwm_nmp = scp.maxnpwm = maxnpwm;

	return;
}

void CMotBase::setControllerParameters(byte kSpeed, byte kPos, byte kI) {

	byte p[32];		//packet
	byte buf[256];	//readbuf
	byte sz = 0;	//readbuf size
	p[0] = 'S';
	p[1] = gnl.SID;
	p[2] = 1;		// subcommand 1 "Set Controller Parameters"
	p[3] = kSpeed;
	p[4] = kPos;
	p[5] = kI;

	protocol->comm(p,buf,&sz);
	scp.kspeed_nmp = scp.kP_speed = kSpeed;
	scp.kpos_nmp = scp.kP = kPos;
	scp.kI_nmp = kI; // no corresponding old motor parameter

	return;
}

void CMotBase::setCrashLimit(int limit) {

	byte p[32];		//packet
	byte buf[256];	//readbuf
	byte sz = 0;	//readbuf size
	p[0] = 'S';
	p[1] = gnl.SID;
	p[2] = 5;		// subcommand 5 "Set Crash Limit"
	p[3] = (byte) (limit >> 8);
	p[4] = (byte) limit;
	p[5] = 0;

	protocol->comm(p,buf,&sz);
	scp.crash_limit_nmp = limit;

	return;
}

void CMotBase::setCrashLimitLinear(int limit_lin) {

	byte p[32];		//packet
	byte buf[256];	//readbuf
	byte sz = 0;	//readbuf size
	p[0] = 'S';
	p[1] = gnl.SID;
	p[2] = 6;		// subcommand 6 "Set Crash Limit Linear"
	p[3] = (byte) (limit_lin >> 8);
	p[4] = (byte) limit_lin;
	p[5] = 0;

	protocol->comm(p,buf,&sz);
	scp.crash_limit_lin_nmp = limit_lin;

	return;
}
///////////////////////////////////////////////////////////////
//for Katana400s:
void CMotBase::setSpeedCollisionLimit(int limit){
	byte p[32];		
	byte buf[256];	
	byte sz = 0;	
	p[0] = 'S';
	p[1] = gnl.SID;
	p[2] = 7;		
	p[3] = (byte) limit; //for both the linear and non-linear the same:
	p[4] = (byte) limit;
	p[5] = 0;
	protocol->comm(p,buf,&sz);
	scp.crash_limit_nmp = limit;
	return;
}
///////////////////////////////////////////////////////////////
//for Katana400s:
void CMotBase::setPositionCollisionLimit(int limit){
	byte p[32];		
	byte buf[256];	
	byte sz = 0;	
	p[0] = 'S';
	p[1] = gnl.SID;
	p[2] = 5;		
	p[3] = (byte) (limit >> 8);
	p[4] = (byte) limit;
	p[5] = 0;
	protocol->comm(p,buf,&sz);
	scp.crash_limit_nmp = limit;
	return;
}

void CMotBase::getParameterOrLimit(int subcommand, byte* R1, byte* R2, byte* R3) {
	// illegal subcommand
	if ((subcommand > 255) || ((subcommand < 249) && (subcommand != 245))) {
		*R1 = 0;
		*R2 = 0;
		*R3 = 0;
		return;

	}

	byte p[32];		//packet
	byte buf[256];	//readbuf
	byte sz = 0;	//readbuf size
	p[0] = 'S';
	p[1] = gnl.SID;
	p[2] = (byte) subcommand;
	p[3] = 0;
	p[4] = 0;
	p[5] = 0;

	protocol->comm(p,buf,&sz);

	*R1 = buf[3];
	*R2 = buf[4];
	*R3 = buf[5];

	return;
}

void CMotBase::sendSpline(short targetPosition, short duration, short p1, short p2, short p3, short p4) {
    std::vector<byte> sendBuf(14), recvBuf(2, 0);
	byte readBytes = 0;

	sendBuf[0] = 'G';
	sendBuf[1] = gnl.SID;
    sendBuf[2] = static_cast<byte>(targetPosition >> 8);
    sendBuf[3] = static_cast<byte>(targetPosition);
    sendBuf[4] = static_cast<byte>(duration >> 8);
    sendBuf[5] = static_cast<byte>(duration);

    sendBuf[6] = static_cast<byte>(p1 >> 8);
    sendBuf[7] = static_cast<byte>(p1);
    sendBuf[8] = static_cast<byte>(p2 >> 8);
    sendBuf[9] = static_cast<byte>(p2);
    sendBuf[10] = static_cast<byte>(p3 >> 8);
    sendBuf[11] = static_cast<byte>(p3);
    sendBuf[12] = static_cast<byte>(p4 >> 8);
    sendBuf[13] = static_cast<byte>(p4);

	protocol->comm(&sendBuf.front(), &recvBuf.front(), &readBytes);
}


void CMotBase::sendFourSplines(short targetPosition, short duration, std::vector<short>& coefficients) {
	std::vector<byte> sendBuf(38), recvBuf(2, 0);
	byte readBytes = 0;

	assert( (coefficients.size() == 16) || "Function takes exactly 4x4 coefficients!" );

	sendBuf[0] = 'L';
	sendBuf[1] = gnl.SID;
    sendBuf[2] = static_cast<byte>(targetPosition >> 8);
    sendBuf[3] = static_cast<byte>(targetPosition);
    sendBuf[4] = static_cast<byte>(duration >> 8);
    sendBuf[5] = static_cast<byte>(duration);
	int n = 6;
    for(unsigned int i = 0; i < coefficients.size(); ++i) {
        sendBuf[n] = static_cast<byte>(coefficients[i] >> 8);
        sendBuf[n+1] = static_cast<byte>(coefficients[i]);
		n = n + 2;
    }
	protocol->comm(&sendBuf.front(), &recvBuf.front(), &readBytes);
}

void CMotBase::setTPSPDegrees(double tar) {
	int enc = KNI_MHF::rad2enc( KNI_MHF::deg2rad(tar), _initialParameters.angleOffset, _initialParameters.encodersPerCycle, _initialParameters.encoderOffset,  _initialParameters.rotationDirection);
	setTPSP(enc);
}

void CMotBase::setTPSP(int tar) {
	tps.tarpos = tar;
	freedom = true;
}

void CMotBase::resetTPSP() {
	freedom = false;
}

void
CMotBase::setInitialParameters(double angleOffset, double angleRange, int encodersPerCycle, int encoderOffset, int rotationDirection) {

	_initialParameters.angleOffset = angleOffset;
	_initialParameters.angleRange = angleRange;
	_initialParameters.encoderOffset = encoderOffset;
	_initialParameters.encodersPerCycle = encodersPerCycle;
	_initialParameters.rotationDirection = rotationDirection;

	_initialParameters.angleStop = angleOffset + angleRange;

	int encoderStop = encoderOffset - rotationDirection*static_cast<int>(encodersPerCycle*(angleRange/(2.0*M_PI)));

	_encoderLimits.enc_minpos = (encoderOffset > encoderStop) ? encoderStop : encoderOffset;
	_encoderLimits.enc_maxpos = (encoderOffset < encoderStop) ? encoderStop : encoderOffset;
	_encoderLimits.enc_per_cycle = encodersPerCycle;
	_encoderLimits.enc_range = ::abs(_encoderLimits.enc_minpos - _encoderLimits.enc_maxpos);
}

void
CMotBase::setCalibrationParameters(bool doCalibration, short order, TSearchDir direction, TMotCmdFlg motorFlagAfter, int encoderPositionAfter) {
	_calibrationParameters.enable = doCalibration;
	_calibrationParameters.order  = order;
	_calibrationParameters.dir    = direction;
	_calibrationParameters.mcf    = motorFlagAfter;
	_calibrationParameters.encoderPositionAfter = encoderPositionAfter;
	_calibrationParameters.isCalibrated = false;
}

void
CMotBase::setCalibrated(bool calibrated) {
	_calibrationParameters.isCalibrated = calibrated;
}

void
CMotBase::setTolerance(int tolerance) {
	_encoderLimits.enc_tolerance = tolerance;
}

bool CMotBase::checkAngleInRange(double angle) {
	return (angle >= _initialParameters.angleOffset) && (angle <= _initialParameters.angleStop);
}
bool CMotBase::checkEncoderInRange(int encoder) {
	return (encoder >= _encoderLimits.enc_minpos) && (encoder <= _encoderLimits.enc_maxpos);
}


void CMotBase::inc(int dif, bool wait, int tolerance, long timeout) {
	recvPVP();
	mov( GetPVP()->pos + dif, wait, tolerance, timeout);
}

void CMotBase::dec(int dif, bool wait, int tolerance, long timeout) {
	recvPVP();
	mov(GetPVP()->pos - dif, wait, tolerance, timeout);
}

void CMotBase::mov(int tar, bool wait, int tolerance, long timeout) {

	if (!checkEncoderInRange(tar))
		throw MotorOutOfRangeException();

	tps.mcfTPS = MCF_ON;
	tps.tarpos = tar;

	sendTPS(&tps);

	if (wait)
		waitForMotor(tar,tolerance,0,timeout);
	else
		return;
}

void CMotBase::waitForMotor(int target, int encTolerance, short mode,
		int waitTimeout) {
	const long POLLFREQUENCY = 200;
	KNI::Timer t(waitTimeout), poll_t(POLLFREQUENCY);
	t.Start();
	while (true) {
		if (t.Elapsed())
			throw MotorTimeoutException();
		poll_t.Start();
		recvPVP();
		if (GetPVP()->msf == 40)
			throw MotorCrashException();
		switch(mode)
		{
			case 0:
				if (::abs(target - GetPVP()->pos) < encTolerance)
					return; // position reached
				break;
			case 1:
				if (GetPVP()->msf == MSF_NLINMOV)
					return; // non-linear movement reached
				break;
			case 2:
				if (GetPVP()->msf != MSF_LINMOV)
					return; // linear movement reached
				break;
		}
		poll_t.WaitUntilElapsed();
	}	
}

void CMotBase::incDegrees(double dif, bool wait, int tolerance, long timeout) {
	int enc = KNI_MHF::rad2enc( KNI_MHF::deg2rad(dif), _initialParameters.angleOffset, _initialParameters.encodersPerCycle, _initialParameters.encoderOffset, _initialParameters.rotationDirection);
	inc(enc, wait, tolerance, timeout);
}

void CMotBase::decDegrees(double dif, bool wait, int tolerance, long timeout) {
	int enc = KNI_MHF::rad2enc( KNI_MHF::deg2rad(dif), _initialParameters.angleOffset, _initialParameters.encodersPerCycle, _initialParameters.encoderOffset, _initialParameters.rotationDirection);
	dec(enc, wait, tolerance, timeout);
}

void CMotBase::movDegrees(double tar, bool wait, int tolerance, long timeout) {
	int enc = KNI_MHF::rad2enc( KNI_MHF::deg2rad(tar), _initialParameters.angleOffset, _initialParameters.encodersPerCycle, _initialParameters.encoderOffset, _initialParameters.rotationDirection);
	mov(enc, wait, tolerance, timeout);
}
