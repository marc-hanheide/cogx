/** @file AxNI.cpp
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * Katana 450 testing program. Run ONLY if you know what it does!!!!!!!!!!!!
 * While running the program always hold main power switch in your hand.
 *
 * @version 1.0
 *
 */

//------------------------------------------------------------------------------

// AXNI_POLYNOM_TEST_ -> 0: Calibration of each joint separately + zero pose move using p2p movements
// AXNI_POLYNOM_TEST_ -> 1: Calibration of all joints + zero pose move using polynomials

//#define AXNI_POLYNOM_TEST_

//------------------------------------------------------------------------------

#include <Golem/Device/Katana450/Katana450.h>
#include <Golem/Device/Katana450/AxNI_base.h>
#include <Golem/Tools/Data.h>
#include <Golem/Demo/Common/Tools.h>
#include <iostream>

using namespace golem;

//------------------------------------------------------------------------------

#ifdef AXNI_POLYNOM_TEST_
void moveTrajectory(Katana450Joint* pKatana450Joint, const GenCoord& begin, const GenCoord& end, U32 segments, SecTmReal dt) {
	MessageStream* str = pKatana450Joint->getContext().getMessageStream();
	const PerfTimer& tm = pKatana450Joint->getContext().getTimer();

	const GenCoord min = pKatana450Joint->getMin();
	const GenCoord max = pKatana450Joint->getMax();
	if (begin.pos < min.pos || begin.pos > max.pos || end.pos < min.pos || end.pos > max.pos)
		throw Message("trajectory not in range!");

	const SecTmReal duration = dt*segments; // overall duration
	GenCoordTrj trajectory(SEC_TM_REAL_ZERO, duration, begin, end); // 3rd degree trajectory

	for (U32 i = 0; i < segments; ++i) {
		const bool bSendPrev = i > 0;
		const bool bSendNext = i < segments - 1;

		// wait for empty buffer (when trajectory finishes)
		PerfTimer tt;
		for (;;) {
			const U32 size = U32(pKatana450Joint->pAxNI_base->getMoveBufferSize((U8)(pKatana450Joint->getIndex() + 1)));
			if (size >= 12) {
				str->write("SYNC (#%i, @%f), size=%i", i, tm.elapsed(), size);
				break;
			}
			tt.reset();
		}
//		while (!pKatana450Joint->sysSync()) {
//			PerfTimer::sleep(AxNI_base::CANopenInhibitTime);
//		}

		// sample trajectory
		const SecTmReal t = dt*i;
		const GenCoord prev = trajectory.get(t);
		const GenCoord next = trajectory.get(t + dt);

		// send segment #i
//str->write("SENDING (#%i, @%f): {(%f, %f), (%f, %f), prev=%i, next=%i}", i, tm.elapsed(), prev.pos, prev.vel, next.pos, next.vel, int(bSendPrev), int(bSendNext));
		pKatana450Joint->sysSend(prev, next, bSendPrev, bSendNext, dt);
		PerfTimer::sleep(AxNI_base::CANopenCycleTime);
		
		// initialise movement
		if (!bSendPrev) {
			pKatana450Joint->sysStart();
			// wait for CAN bus state change
			//PerfTimer::sleep(AxNI_base::CANopenInhibitTime);
		}
		//PerfTimer::sleep(AxNI_base::CANopenCycleTime);
		PerfTimer::sleep(AxNI_base::CANopenInhibitTime);

str->write("dt=%f", tt.elapsed());

//U32 encPos = pKatana450Joint->pAxNI_base->getPosition(U8(pKatana450Joint->getIndex() + 1));
//Real pos = pKatana450Joint->posFromEnc(encPos);
//str->write("READING (#%i, @%f): enc=%i, pos=%f", i, tm.elapsed(), encPos, pos);
	}
}

void test(Joint* pJoint) {
	MessageStream* str = pJoint->getContext().getMessageStream();
	const PerfTimer& tm = pJoint->getContext().getTimer();

	Katana450Joint* pKatana450Joint = dynamic_cast<Katana450Joint*>(pJoint);
	if (!pKatana450Joint)
		throw Message("Katana450Joint interface required!");

	const GenCoord min = pKatana450Joint->getMin();
	const GenCoord max = pKatana450Joint->getMax();
	GenCoord begin, end;
	
	const U32 segments = 10; // num of trajectory segments must not be smaller than 1
	const SecTmReal dt = SecTmReal(0.5); // single segment duration

	pKatana450Joint->sysRecv(begin);
	str->write("\nAXIS #%d: pos = {%f, %f, %f}, min = {%f, %f, %f}, max = {%f, %f, %f}",
		U32(pKatana450Joint->getIndex() + 1), begin.pos, begin.vel, begin.acc, min.pos, min.vel, min.acc, max.pos, max.vel, max.acc
	);
	end = GenCoord(Real(0.0), REAL_ZERO, REAL_ZERO); // movement target

	PerfTimer::sleep(0.1);

	// solution #0
	moveTrajectory(pKatana450Joint, begin, end, segments, dt);	
	PerfTimer::sleep(2.0);
	//moveTrajectory(pKatana450Joint, end, begin, segments, dt);

	//SecTmReal tEnd = tm.elapsed() + 5.0;
	//for (;;) {
	//	SecTmReal t = tm.elapsed();
	//	U32 encPos = pKatana450Joint->pAxNI_base->getPosition(U8(pKatana450Joint->getIndex() + 1));
	//	Real pos = pKatana450Joint->posFromEnc(encPos);
	//	str->write("READING (@%f): enc=%i, pos=%f", t, encPos, pos);
	//	PerfTimer::sleep(0.1);
	//	if (t > tEnd) break;
	//}
}

void test(Arm* pArm) {
	Katana450Arm* pKatana450Arm = dynamic_cast<Katana450Arm*>(pArm);
	if (!pKatana450Arm)
		throw Message("Katana450Arm interface required!");

	// Axis #5
	//test(pKatana450Arm->getJoints()[4]);
	// Axis #4
	//test(pKatana450Arm->getJoints()[3]);
	// Axis #3
	//test(pKatana450Arm->getJoints()[2]);
	// Axis #1
	//test(pKatana450Arm->getJoints()[0]);
	// Axis #2
	//test(pKatana450Arm->getJoints()[1]);
}

#else
Real posFromEnc(int pos, Real angleOffset, int encoderOffset, int encodersPerCycle, int rotationDirection, Real offset, Real gain) {
	return offset + gain*(angleOffset - REAL_2_PI*(pos - encoderOffset)/Real(encodersPerCycle*rotationDirection));
}
int posToEnc(Real pos, Real angleOffset, int encoderOffset, int encodersPerCycle, int rotationDirection, Real offset, Real gain) {
	return (int)Math::round(Real(encoderOffset) + (angleOffset - (pos - offset)/gain)*Real(encodersPerCycle*rotationDirection)/REAL_2_PI);
}

void test(AxNI_base* axni, U32 node, Real angleOffset, Real angleRange, int encoderOffset, int encodersPerCycle, int rotationDirection, int encoderPositionAfter, Real offset, Real gain) {
	printf("\nAXIS #%d:\nangleOffset=%f, angleRange=%f, encoderOffset=%i, encodersPerCycle=%i, rotationDir=%i, encoderPositionAfter=%i, offset=%f, gain=%f\n",
		node, angleOffset, angleRange, encoderOffset, encodersPerCycle, rotationDirection, encoderPositionAfter, offset, gain);
	printf("angle (encoder min) = %f\n",
		posFromEnc(encoderOffset, angleOffset*REAL_2_PI/360.0, encoderOffset, encodersPerCycle, rotationDirection, offset, gain));
	printf("angle (encoder init) = %f\n", 
		posFromEnc(encoderPositionAfter, angleOffset*REAL_2_PI/360.0, encoderOffset, encodersPerCycle, rotationDirection, offset, gain));

	Real max = offset + gain*(angleOffset + angleRange)*REAL_2_PI/360.0;
	Real min = offset + gain*angleOffset*REAL_2_PI/360.0;
	if (max < min)
		std::swap(max, min);
	//printf("min = %f, max = %f\n", min, max);

	int encMin = posToEnc(min, angleOffset*REAL_2_PI/360.0, encoderOffset, encodersPerCycle, rotationDirection, offset, gain);
	Real angMin = posFromEnc(encMin, angleOffset*REAL_2_PI/360.0, encoderOffset, encodersPerCycle, rotationDirection, offset, gain);
	int encMax = posToEnc(max, angleOffset*REAL_2_PI/360.0, encoderOffset, encodersPerCycle, rotationDirection, offset, gain);
	Real angMax = posFromEnc(encMax, angleOffset*REAL_2_PI/360.0, encoderOffset, encodersPerCycle, rotationDirection, offset, gain);
	
	printf("angle (min, max) = (%f, %f)\n", angMin, angMax);
	printf("encoder (min, max) = (%i, %i)\n", encMin, encMax);

	// AxNI
	axni->activate((U8)node);
	AxNI_base::CalibrationParameter calParam;
	calParam.encOffset = (U32)encoderOffset;
	calParam.encPosAfter = (U32)encoderPositionAfter;
	calParam.encPerCycle = (U32)encodersPerCycle;
	AxNI_base::ControllerParameter ctrlParam;
	AxNI_base::CollisionParameter collParam;
	axni->calibrate(	(U8)node, calParam, ctrlParam, collParam);

	// Current angle reading test 
	U32 pos = axni->getPosition((U8)node);
	printf("AxNI: angle (encoder init) = {enc=%i, ang=%f}\n", pos, posFromEnc(pos, angleOffset*REAL_2_PI/360.0, encoderOffset, encodersPerCycle, rotationDirection, offset, gain));

	// Zero angle move test (p2p)
	pos = posToEnc(0.0, angleOffset*REAL_2_PI/360.0, encoderOffset, encodersPerCycle, rotationDirection, offset, gain);
	axni->moveP2Pwait((U8)node, pos, 10, 1, 50);
	pos = axni->getPosition((U8)node);
	printf("AxNI: angle (zero) = {enc=%i, ang=%f}\n", pos, posFromEnc(pos, angleOffset*REAL_2_PI/360.0, encoderOffset, encodersPerCycle, rotationDirection, offset, gain));
}

#endif

//------------------------------------------------------------------------------

int main(int argc, char *argv[]) {
	try {
		// Determine configuration file name
		std::string cfg;
		if (argc == 1) {
			// default configuration file name
			cfg.assign(argv[0]);
#ifdef WIN32
			size_t pos = cfg.rfind(".exe"); // Windows only
			if (pos != std::string::npos) cfg.erase(pos);
#endif
			cfg.append(".xml");
		}
		else
			cfg.assign(argv[1]);

		// Create XML parser and load configuration file
		XMLParser::Desc parserDesc;
		XMLParser::Ptr pParser = parserDesc.create();
		try {
			FileReadStream fs(cfg.c_str());
			pParser->load(fs);
		}
		catch (const Message& msg) {
			std::cerr << msg.str() << std::endl;
			std::cout << "Usage: " << argv[0] << " <configuration_file>" << std::endl;
			return 1;
		}

		// Find program XML root context
		XMLContext* pXMLContext = pParser->getContextRoot()->getContextFirst("golem");
		if (pXMLContext == NULL)
			throw Message(Message::LEVEL_CRIT, "Unknown configuration file: %s", cfg.c_str());

		// Create program context
		golem::Context::Desc contextDesc;
		XMLData(contextDesc, pXMLContext);
		golem::Context::Ptr pContext = contextDesc.create(); // throws

		//-----------------------------------------------------------------------------

#ifdef AXNI_POLYNOM_TEST_
		// Load Katana 450 driver
		Arm::Desc::Ptr pDesc = Arm::Desc::load(*pContext, "GolemDeviceKatana450");

		// Create arm controller
		pContext->getMessageStream()->write(Message::LEVEL_INFO, "Initialising %s...", pDesc->name.c_str());
		Arm::Ptr pArm = pDesc->create(*pContext);
		
		// Display arm information
		armInfo(*pArm);

		//test(&*pArm);
#else
		// Init CAN bus
		AxNI_base axni;
		axni.start("/dev/pcanusb0");

		// uncomment only one arm joint at time!!!!!

		// joint #1
		//test(&axni,	1,	  6.65,	 339.0,	100000,	51200,	+1,	99000,	-REAL_PI,	+1.0);//new
		
		// joint #2
		// "stretch" joint #3 by hand before moving joint #2!!!!!
		//test(&axni,	2,	124.25,	-132.0,	100000,	94976,	-1,	99000,	     0.0,	-1.0);//new
		
		// joint #3
		//test(&axni,	3,	 52.7 ,	 127.4,	100000,	94976,	+1,	99000,	 REAL_PI,	-1.0);
		
		// joint #4
		//test(&axni,	4,	 63.5 ,	 224.0,	100000,	51200,	-1,	101000,	-REAL_PI,	+1.0);
		
		// joint #5
		//test(&axni,	5,	  8.5 ,	 336.0,	100000,	51200,	+1,	99000,	 REAL_PI,	-1.0);
#endif

		PerfTimer::sleep(10.0);
	}
	catch (const Message& msg) {
		std::cerr << msg.str() << std::endl;
	}
	catch (const std::exception &ex) {
		std::cerr << Message(Message::LEVEL_CRIT, "C++ exception: %s", ex.what()).str() << std::endl;
	}

	return 0;
}
