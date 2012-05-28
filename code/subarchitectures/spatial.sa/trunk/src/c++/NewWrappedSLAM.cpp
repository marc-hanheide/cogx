//
// = LIBRARY
//
// = FILENAME
//    NewWrappedSLAM.cc
//
// = FUNCTION
//
// = AUTHOR(S)
//    John Folkesson
//    Patric Jensfelt 
//
// = DESCRIPTION
//    
// = COPYRIGHT
//    Copyright (c) 2005 John Folkesson 
//
/*----------------------------------------------------------------------*/

#include "NewWrappedSLAM.hpp"

#include <Utils/CureDebug.hh>

#include <AddressBank/RLDisplaySICK.hh>
#include <Filters/SimpleOdoModel.hh>
#include <Filters/GyroModel.hh>
#include <Filters/GyroAdjOdometry.hh>
#include <AddressBank/ConfigFileReader.hh>

#ifndef DEPEND
#endif

namespace Cure {

NewWrappedSLAM::NewWrappedSLAM(short numberOfSensors, bool usefuse) :
	DataFilter(4, 0, false), m_MapBank(10), m_Map(&m_MapBank), m_Dead(200),
			m_Hough(), m_SlotScans(10, SICKSCAN_TYPE), m_SlotMeas(10,
					MEASUREMENTSET_TYPE), m_ScanDisp(0) {
	m_NumberOfSensors = numberOfSensors;
	constructorInit();
	m_ConfigModels = 1;
	if (usefuse) {
		m_ConfigModels += 2;
	}
}

NewWrappedSLAM::NewWrappedSLAM(PoseErrorModel *modelA, Timestamp & period,
		short numberOfSensors, PoseErrorModel *modelB, FuseFilter *fuser,
		bool userotate, long inputBuffer, long outputbuffer)

:
	DataFilter(4, 0, false), m_MapBank(10), m_Map(&m_MapBank), m_Dead(modelA,
			period, modelB, fuser, userotate, inputBuffer, outputbuffer), m_Hough(),
			m_SlotScans(10, SICKSCAN_TYPE), m_SlotMeas(10, MEASUREMENTSET_TYPE),
			m_ScanDisp(0) {
	m_NumberOfSensors = numberOfSensors;
	constructorInit();
	m_ConfigModels = 0;
}

void NewWrappedSLAM::constructorInit() {
	m_SLAM = new EKFSLAM(&m_Map, 0, m_NumberOfSensors, 100);
	m_OffsetSlot = 0;
	useScanlines = true;
	m_SlotScans.setName("NewWrappedSLAM.SlotScans");
	m_SlotMeas.setName("NewWrappedSLAM.SlotMeas");
	FilterName = "NewWrappedSLAM";
	m_ModelA = 0;
	m_ModelB = 0;
	m_Fuser = 0;
	m_SlotMeas.push(m_Dead.trigger());
	m_SlotScans.push(m_Dead.trigger());
	if (m_NumberOfSensors > 0)
		m_OffsetSlot = new DataSlotAddress[m_NumberOfSensors];
	for (short i = 0; i < m_NumberOfSensors; i++) {
		m_OffsetSlot[i].Slot.setup(10, POSE3D_TYPE);
	}

	m_walkclocktimefile.open("slaminputdata.txt", std::ios::out);
}

NewWrappedSLAM::~NewWrappedSLAM() {
	if (m_OffsetSlot)
		delete[] m_OffsetSlot;
	m_OffsetSlot = 0;
	if (m_ScanDisp)
		delete m_ScanDisp;
	if (m_ModelA)
		delete m_ModelA;
	if (m_ModelB)
		delete m_ModelB;
	if (m_Fuser)
		delete m_Fuser;
	m_ModelB = 0;
	m_ModelA = 0;
	m_Fuser = 0;
	m_ScanDisp = 0;

	if (m_SLAM) {
		delete m_SLAM;
		m_SLAM = 0;
	}
}

int NewWrappedSLAM::resetRobotPose(const Pose3D &p, bool useCov) {
	CureCERR(30) << "resetRobotPose not implemented for NewWrappedSLAM\n";
	return 1;
}

Cure::Pose3D NewWrappedSLAM::getPosePrediction() {
	return PoseProvider::getPredictedWorldPose(m_SLAM->PoseOut, m_Dead.m_SlotInA);
}

int NewWrappedSLAM::config(const std::string &cfgFile) {
	ConfigFileReader cfg;
	if (cfg.init(cfgFile.c_str()))
		return -1;

	if (m_DontDisplay == false) {
		int displayMode = 0;
		cfg.getDisplayMode(displayMode); // If this call fails we dont display
		if (displayMode == 1) {
			std::string rlHost = "localhost";
			cfg.getRoboLookHost(rlHost); // Fall back to localhost if not host found
			std::string robot;
			if (cfg.getRobotName(robot))
				return -1;
			if (m_Rlph.connect(rlHost, robot) == 0) {
				double x, y, theta;
				if (cfg.getMapDisplayOffset(x, y, theta) == 0)
					m_Rlph.setEstEnvPose(x, y, theta);
			} else {
				m_Rlph.m_Rlp = 0;
			}
		}
	}

	if (m_Map.config(cfg, m_SLAM->MSet))
		return -1;
	if (m_ConfigModels & 1) {
		int odType;
		std::string odParams;
		if (cfg.getOdomModel(odType, odParams))
			return -1;
		if (odType == 0) {
			m_ModelA = new SimpleOdoModel();
		} else {
			CureCERR(20) << "WARNING odType=" << odType << " not handled\n";
			return -1;
		}
		m_Dead.setErrorModel(m_ModelA, 0);
		if (m_ModelA->config(odParams))
			return -1;
	}
	if (m_ConfigModels & 2) {
		int gyroType;
		std::string gyroParams;
		if (cfg.getInertialModel(gyroType, gyroParams)) {
			CureCERR(20) << "WARNING inertialType=" << gyroType << " not handled\n";
		} else {
			m_Fuser = new GyroAdjOdometry();
			if (gyroType == 0) {
				m_ModelB = new GyroModel();
			} else {
				CureCERR(20) << "WARNING inertialType=" << gyroType << " not handled\n";
				return -1;
			}
			if (m_ModelB->config(gyroParams))
				return -1;
			m_Dead.setErrorModel(m_ModelB, 1);
			m_Dead.setFuser(m_Fuser, true);
		}
	}
	if (m_SLAM->setup(cfg))
		return -1;
	if (useScanlines) {
		for (int rhci = 1; rhci <= 2; rhci++) {
			std::string rhCfg;
			if (cfg.getRangeHoughConfig(rhci, rhCfg) == 0) {
				if (m_Hough.config(rhCfg)) {
					CureCERR(20) << "ERROR: failed with subconfig " << rhci
							<< " for RangeHough\n";
					return -1;
				}
			} else {
				CureCERR(30)
						<< "WARNING: Using default values for RangeHough subconfig "
						<< rhci << std::endl;
			}
		} CureDO(30) {
			m_Hough.printConfiguration();
		}
		m_SlotScans.push(m_Hough.in(0));
		m_Hough.out(0)->push(&m_SlotMeas);
		m_SlotMeas.push(m_SLAM->in(1));
	}
	if (m_DontDisplay == false) {
		m_SLAM->setRoboLookProxy(m_Rlph.m_Rlp);
		if (m_Rlph.m_Rlp) {
			m_ScanDisp = new RLDisplaySICK(m_Rlph.m_Rlp);
			m_SlotScans.push(m_ScanDisp->in(0));
		}
	}
	m_SLAM->in(0)->pull(m_Dead.out(3));
	//  m_SLAM->in(1)->pull(m_Dead.out(1));
	// m_SLAM->in(2)->pull(m_Dead.out(2));
	return 0;
}

void NewWrappedSLAM::loosenMatching(double factor, int type) {
	m_SLAM->loosenMatching(factor, type);
}

void NewWrappedSLAM::setGain(double gain) {
	m_SLAM->setGain(gain);
}

void NewWrappedSLAM::increaseMinCovariance(double factor) {
	if (useScanlines)
		m_Hough.increaseMinCovariance(factor);
}
void NewWrappedSLAM::increaseLineQuality(double factor) {
	if (useScanlines)
		m_Hough.increaseLineQuality(factor);
}
void NewWrappedSLAM::removeClutter(double factor) {
	if (useScanlines)
		m_Hough.removeClutter(factor);
	m_SLAM->removeClutter(factor);
}

int NewWrappedSLAM::addOdometry(const Pose3D &odo) {
	struct timeval tv;
	gettimeofday(&tv, NULL);
	char buf[128];
	sprintf(buf, "1 0 %ld.%06ld %ld.%06ld", odo.Time.Seconds, odo.Time.Microsec,
			tv.tv_sec, tv.tv_usec);
	m_walkclocktimefile << buf << std::endl;

	Pose3D o(odo); // Since write does not allow const
	return m_Dead.in(0)->write(o);
}
int NewWrappedSLAM::addInertial(const Pose3D &inert) {
	Pose3D i(inert); // Since write does not allow const
	return m_Dead.in(1)->write(i);
}

int NewWrappedSLAM::addScan(const SICKScan &scan) {
	SICKScan s(scan); // Since write does not allow const
	int err = m_SlotScans.write(s);
	if (err)
		return -err;

	bool ret = (m_SLAM->PoseOut.getDoubleTime()
			!= m_LastTimePoseUpdated.getDouble());
	m_LastTimePoseUpdated = m_SLAM->PoseOut.Time;
	return int(ret);
}

int NewWrappedSLAM::addMeasurementSet(MeasurementSet &measSet) {
	struct timeval tv;
	gettimeofday(&tv, NULL);
	char buf[128];

	int err = m_SlotMeas.write(measSet);
	sprintf(buf, "2 %d %ld.%06ld %ld.%06ld", err, measSet.Time.Seconds,
			measSet.Time.Microsec, tv.tv_sec, tv.tv_usec);
	m_walkclocktimefile << buf << std::endl;
	if (err) {
		return -err;
	}
	bool ret = (m_SLAM->PoseOut.getDoubleTime()
			!= m_LastTimePoseUpdated.getDouble());
	m_LastTimePoseUpdated = m_SLAM->PoseOut.Time;
	return int(ret);
}

unsigned short NewWrappedSLAM::write(Cure::TimestampedData& p, const int port) {
	if (port == 0) {
		Pose3D *odo = p.narrowPose3D();
		if (odo) {
			return m_Dead.in(0)->write(p);
		} else {
			CureCERR(20) << "Only allows odometry to be written to port 0 for \""
					<< FilterName << "\"\n";
			return TYPE_ERROR;
		}
	} else if (port == 1) {
		SICKScan *scan = p.narrowSICKScan();
		if (scan) {
			return m_SlotScans.write(p);
		} else {
			CureCERR(20) << "Only allows scans to be written to port 1 for \""
					<< FilterName << "\"\n";
			return TYPE_ERROR;
		}
	} else if (port == 2) {
		Pose3D *inert = p.narrowPose3D();
		if (inert) {
			return m_Dead.in(1)->write(p);
		} else {
			CureCERR(20) << "Only allows inertial to be written to port 2 for \""
					<< FilterName << "\"\n";
			return TYPE_ERROR;
		}
	} else if (port == 3) {
		m_Dead.trigger()->write(p);
		return m_SLAM->in(1)->write(p);
	} else {
		CureCERR(20) << "Can only write to port 0,1  and 2 for \"" << FilterName
				<< "\"\n";
		return ADDRESS_INVALID;
	}
	return 0;
}
int NewWrappedSLAM::saveMap(const std::string &filename) {
	CureCERR(30) << "Saving map \"" << filename << "\"\n";
	std::fstream fs;
	fs.open(filename.c_str(), std::ios::out);
	if (!fs.good()) {
		CureCERR(20) << "Failed to save map to file \"" << filename << "\"\n";
		return 1;
	}
	return m_Map.saveMap(fs);
}

int NewWrappedSLAM::saveTrackKeys(const std::string &filename) {
	CureCERR(30) << "Saving TrackKeys \"" << filename << "\"\n";
	std::fstream fs;
	fs.open(filename.c_str(), std::ios::out);
	if (!fs.good()) {
		CureCERR(20) << "Failed to save hash tables to file \"" << filename
				<< "\"\n";
		return 1;
	}
	return m_Map.saveTrackKeys(fs);
}

} // namespace Cure
