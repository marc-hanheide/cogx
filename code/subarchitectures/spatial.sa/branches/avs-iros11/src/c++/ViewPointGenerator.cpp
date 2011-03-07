/*
 * ViewPointGenerator.cpp
 *
 *  Created on: Mar 4, 2011
 *      Author: alper
 */

#include "ViewPointGenerator.h"
#include "AVS_ContinualPlanner.h"

namespace spatial {
ViewPointGenerator::~ViewPointGenerator() {
	// TODO Auto-generated constructor stub

}


ViewPointGenerator::ViewPointGenerator(AVS_ContinualPlanner* component, CureObstMap* plgm, BloxelMap* bloxelmap, int samplesize,
		double sampleawayfromobs, double conedepth, double horizangle, double vertangle, double minDistance, double pdfsum, double pdfthreshold, double robotx, double roboty)
{
	// TODO Auto-generated destructor stub
	m_component = component;
	lgm = plgm;
	bloxelmap= bloxelmap;
	m_samplesize = samplesize;
	m_sampleawayfromobs = sampleawayfromobs;
	m_conedepth = conedepth;
	  m_horizangle= horizangle;
	  m_vertangle = vertangle;
	  m_minDistance = minDistance;
	  m_bloxelmapPDFsum = pdfsum ;
	  m_pdfthreshold = pdfthreshold;
	m_robotx = robotx;
	  m_roboty= roboty;
	m_best3DConeRatio = 0.1;
    m_tiltinterval = 0.1;
    m_component->log("ViewPointGenerator parameters: m_samplesize: %d, m_sampleawayfromobs: %f, m_conedepth: %f, m_horizangle: %f, m_vertangle: %f, m_minDistance: %f, m_bloxelmapPDFsum: %f , m_pdfthreshold: %f",
    		m_samplesize, m_sampleawayfromobs, m_conedepth,m_horizangle,m_vertangle,m_minDistance, m_bloxelmapPDFsum, m_pdfthreshold);
    m_component->log("BloxelMap size: %d, %d CureMap size: %d", bloxelmap->getMapSize().first,bloxelmap->getMapSize().second, lgm->getSize());
}


vector<ViewPointGenerator::SensingAction> ViewPointGenerator::getBest3DViewCones(){

	m_component->log("ViewPointGenerator::GetBest3DViewCones");

	vector<pair<unsigned int, double> > ordered2DVClist = get2DCandidateViewCones();
	vector<SensingAction> unordered3DVCList, ordered3DVCList, tmp;

	SensingAction sample;
		std::vector<SensingAction> samplepoints;

		std::vector<double> angles;
		for (double rad = -30 * M_PI / 180; rad < 30 * M_PI / 180; rad = rad
				+ m_tiltinterval) {
			angles.push_back(rad);
		}

		// We have the VC candidate list ordered according to their 2D pdf sums
		// now for the top X candidate get a bunch of tilt angles and calculate the 3D cone sums
		double xW, yW;
		for (unsigned int j = 0; j < ordered2DVClist.size() * m_best3DConeRatio; j++) {
			lgm->index2WorldCoords(m_samples2D[ordered2DVClist[j].first].getX(),
					m_samples2D[ordered2DVClist[j].first].getY(), xW, yW);

			for (unsigned int i = 0; i < angles.size(); i++) {
				std::vector<double> coord;
				// coord.push_back(m_SlamRobotPose.getX());
				// coord.push_back(m_SlamRobotPose.getY());
				coord.push_back(xW);
				coord.push_back(yW);
				coord.push_back(1.4);
				SensingAction sample;
				sample.pos = coord;
				sample.pan = m_samples2D[ordered2DVClist[j].first].getTheta();
				// sample.pan = m_SlamRobotPose.getTheta();
				sample.tilt = angles[i];
				samplepoints.push_back(sample);
			}
		}

		// turn
		unordered3DVCList = getViewConeSums(samplepoints);
		for (unsigned int i =0; i< unordered3DVCList.size(); i++){
			if (ordered3DVCList.size() == 0) {
				ordered3DVCList.push_back(unordered3DVCList[i]);
			} else {
				bool inserted = false;
				tmp = ordered3DVCList;
				for (unsigned int j = 0; j < tmp.size(); j++) {
					if (unordered3DVCList[i].totalprob >= ordered3DVCList[j].totalprob) {
						inserted = true;
						ordered3DVCList.insert(ordered3DVCList.begin() + j, unordered3DVCList[i]);
						break;
					}
				}
				if (!inserted) {
					//	  log("pushing back");
					ordered3DVCList.push_back(unordered3DVCList[i]);
				}
			}
		}
		m_component->log("Ordered 3D view cones");
		//Loop over the ordered list and return cones that covers up to a threshold
		vector<SensingAction> result3DVCList;

		double pdfsumsofar =0;
		for (unsigned int i =0; i < ordered3DVCList.size(); i++){
			double tmp = pdfsumsofar;
			if ( ((tmp + ordered3DVCList[i].totalprob)*100) / m_bloxelmapPDFsum < m_pdfthreshold){
				result3DVCList.push_back(ordered3DVCList[i]);
			}
		}
		m_component->log("Returning %d 3D viewcones", result3DVCList.size());
		return result3DVCList;
}

vector<ViewPointGenerator::SensingAction> ViewPointGenerator::getViewConeSums(std::vector<SensingAction> &samplepoints) {

	SpatialGridMap::GDProbSum sumcells;
	SpatialGridMap::GDIsObstacle isobstacle;
	m_component->log("ViewPointGenerator::getViewConeSums");
	m_component->log("Got %d 3D ViewCones", samplepoints.size());

	try {
		for (unsigned int i = 0; i < samplepoints.size(); i++) {
			SensingAction viewpoint = samplepoints[i];
			/*m_map->coneModifier(samplepoints[i].pos[0],samplepoints[i].pos[1],samplepoints[i].pos[2], samplepoints[i].pan,samplepoints[i].tilt, m_horizangle, m_vertangle, m_conedepth, 10, 10, isobstacle, makeobstacle,makeobstacle);*/
			bloxelmap->coneQuery(samplepoints[i].pos[0], samplepoints[i].pos[1],
					samplepoints[i].pos[2], samplepoints[i].pan,
					samplepoints[i].tilt, m_horizangle, m_vertangle,
					m_conedepth, 10, 10, isobstacle, sumcells, sumcells,
					m_minDistance);
			m_component->log("cone query done.");
			cout << "cone #" << i << " " << viewpoint.pos[0] << " "
					<< viewpoint.pos[1] << " " << viewpoint.pos[2] << " "
					<< viewpoint.pan << " " << viewpoint.tilt
					<< " pdfsum of cone: " << sumcells.getResult() << endl;

			//    /* Show view cone on a temporary map, display the map and wipe it*/

			samplepoints[i].totalprob = sumcells.getResult();
			sumcells.reset();
		}
	} catch (std::exception &e) {
		printf("Caught exception %s: \n", e.what());
	}

	return samplepoints;
}

vector<pair<unsigned int, double> > ViewPointGenerator::get2DCandidateViewCones(){

	m_component->log("ViewPointGenerator::get2DCandidateViewCones");
	std::vector<std::vector<pair<int, int> > > VCones;
	m_samples2D = sample2DGrid();
	VCones = calculate3DViewConesFrom2D();
	CurePDFMap* lgmpdf  = new CurePDFMap(lgm->getSize(),lgm->getCellSize() ,0, CureObstMap::MAP1,lgm->getCentXW(), lgm->getCentYW());

	for (int x = -lgmpdf->getSize(); x <= lgmpdf->getSize(); x++) {
	  for (int y = -lgmpdf->getSize(); y <= lgmpdf->getSize(); y++) {
	    int bloxelX = x + lgmpdf->getSize();
	    int bloxelY = y + lgmpdf->getSize();
	    if ((*lgm)(x,y) != '2'){
	      double bloxel_floor = 0;
	      (*lgmpdf)(x,y) = 0;
	      for (unsigned int i = 0; i < (*bloxelmap)(bloxelX,bloxelY).size();i++){
		(*lgmpdf)(x,y) += (*bloxelmap)(bloxelX, bloxelY)[i].data.pdf * ((*bloxelmap)(bloxelX,bloxelY)[i].celing - bloxel_floor);
		bloxel_floor = (*bloxelmap)(bloxelX,bloxelY)[i].celing;
	      }
	    }
	  }
	}

	double sum;
		int x, y;
		vector<pair<unsigned int, double> > orderedVClist, tmp;
		vector<unsigned int>::iterator it;
		for (unsigned int i = 0; i < VCones.size(); i++) {
			sum = 0;
			for (unsigned int j = 0; j < VCones[i].size(); j++) {
				x = VCones[i][j].first;
				y = VCones[i][j].second;
				if ((x > -lgmpdf->getSize() && x < lgmpdf->getSize()) && (y
						> -lgmpdf->getSize() && y < lgmpdf->getSize()))
					sum += (*lgmpdf)(x, y);

			}
			if (orderedVClist.size() == 0) {
				orderedVClist.push_back(make_pair(i, sum));
			} else {
				bool inserted = false;
				tmp = orderedVClist;
				for (unsigned int j = 0; j < tmp.size(); j++) {
					if (sum >= orderedVClist[j].second) {
						inserted = true;
						orderedVClist.insert(orderedVClist.begin() + j, make_pair(
								i, sum));
						break;
					}
				}
				if (!inserted) {
					m_component->log("pushing 2D cone back");
					orderedVClist.push_back(make_pair(i, sum));
				}
			}
		}
		m_component->log("Ordered 2D VC list has %d cones", orderedVClist.size());
		for(unsigned int i=0; i < orderedVClist.size(); i++){
			m_component->log("Sum of VC #%d is %d", i, orderedVClist[i].second);
		}
		return orderedVClist;
}



bool pathgridseen = false;
double ViewPointGenerator::getPathLength(Cure::Pose3D start,
		Cure::Pose3D destination, CureObstMap* lgm) {

	/*Checking if a point in x,y is reachable */

	/// Binary grid that is 0 for foree space and 1 for occupied
	Cure::BinaryMatrix m_NonFreeSpace;

	// We make the number of columns of the BinaryMatrix a multiple

	// of 32 so that we get the benefit of the representation.
	// Here m_LGMap is assumed to be the LocalGridMap
	int rows = 2 * lgm->getSize() + 1;
	int cols = ((2 * lgm->getSize() + 1) / 32 + 1) * 32;
	m_NonFreeSpace.reallocate(rows, cols);
	m_NonFreeSpace = 0; // Set all cells to zero

	// First we create a binary matrix where all cells that
	// corresponds to known obstacles are set to "1".
	for (int x = -lgm->getSize(); x <= lgm->getSize(); x++) {
		for (int y = -lgm->getSize(); y <= lgm->getSize(); y++) {
			if ((*lgm)(x, y) == '1') { // occupied OR Plane
				m_NonFreeSpace.setBit(x + lgm->getSize(), y + lgm->getSize(),
						true);
			}
		}
	}

	// Create an istance of BinaryMatrx which will hold the result of
	// expanding the obstacles
	Cure::BinaryMatrix m_PathGrid;
	m_PathGrid = 0;

	// Grow each occupied cell to account for the size of the
	// robot. We put the result in another binary matrix, m_PathGrid
	m_NonFreeSpace.growInto(m_PathGrid, 0.5 * 0.01 / lgm->getCellSize(), true);
	// 0.45 is robot width hard coded here.
	// We treat all unknown cells as occupied so that the robot only
	// uses paths that it knowns to be free. Note that we perfom this
	// operation directly on the m_PathGrid, i.e. the grid with the
	// expanded obstacle. The reasoning behind this is that we do not
	// want the unknown cells to be expanded as well as we would have
	// to recalculate the position of the frontiers otherwise, else
	// they might end up inside an obstacle (could happen now as well
	// from expanding the occupied cell but then it is known not to be
	// reachable).
	for (int x = -lgm->getSize(); x <= lgm->getSize(); x++) {
		for (int y = -lgm->getSize(); y <= lgm->getSize(); y++) {
			if ((*lgm)(x, y) == '2') {
				m_PathGrid.setBit(x + lgm->getSize(), y + lgm->getSize(), true);
			}
		}
	}
	// if(!pathgridseen)
	//  pbVis->Display2DBinaryMap(m_PathGrid, lgm);

	//	pathgridseen = true;
	/*Checking if a point in x,y is reachable */

	// Get the indices of the destination coordinates
	//		log("point reachable");
	int rS, cS, rE, cE;
	//log("robotpose: %f,%f", lastRobotPose->x,lastRobotPose->y);
	//log("1");
	if (lgm->worldCoords2Index(start.getX(), start.getY(), rS, cS) == 0
			&& lgm->worldCoords2Index(destination.getX(), destination.getY(),
					rE, cE) == 0) {
		// Compensate for the fact that the PathGrid is just a normal matrix where the cells are numbers from the corner
		cS += lgm->getSize();
		rS += lgm->getSize();
		cE += lgm->getSize();

		rE += lgm->getSize();

		Cure::ShortMatrix path;
		double d = (m_PathGrid.path(rS, cS, rE, cE, path, 100 * lgm->getSize())
				* lgm->getCellSize());
		return d;
	} else {
		return -1;
	}

}

std::vector<Cure::Pose3D> ViewPointGenerator::sample2DGrid() {
	srand(time(NULL));
	std::vector<double> angles;

	std::vector<Cure::Pose3D> samples;
	/*Sampling free space BEGIN*/
	m_component->log("ViewPointGenerator::sample2DGrid");
	for (double rad = 0; rad < M_PI * 2; rad = rad + M_PI / 18) {
		angles.push_back(rad);
	}
	int i = 0;
	int randx, randy;
	double xW, yW, angle;
	m_component->log("Sampling %d points ", m_samplesize);
	bool haspoint;
	Cure::Pose3D singlesample;
	int giveup = 0;
	while (i < m_samplesize) {
		giveup++;
		haspoint = false;
		randx = (rand() % (2 * lgm->getSize())) - lgm->getSize();
		randy = (rand() % (2 * lgm->getSize())) - lgm->getSize();
		int the = (int) (rand() % angles.size());
		angle = angles[the];
		//if we have that point already, skip.
		/* for (int j = 0; j < i; j++) {
		 if (samples[2 * j] == randx && m_samples[2 * j + 1] == randy
		 && m_samplestheta[j] == angle) {
		 //log("we already have this point.");
		 haspoint = true;
		 break;
		 }
		 }*/
		lgm->index2WorldCoords(randx, randy, xW, yW);
		std::pair<int, int> sample;
		sample.first = randx;
		sample.second = randy;

		if (!haspoint && (*lgm)(randx, randy) == '0' && isCircleFree2D(*lgm,
				xW, yW, m_sampleawayfromobs)) {
			Cure::Pose3D start, dest;
			start.setX(m_robotx);
			start.setY(m_roboty);
			dest.setX(xW);
			dest.setX(yW);

			double d = getPathLength(start, dest, lgm);
			m_component->log("path to here: %3.2f", d);
			// There is a path to this destination
			//	    log("there's a path to this destination");
			if (d > 0 || true) {
				singlesample.setX(randx);
				singlesample.setY(randy);
				singlesample.setTheta(angle);
				samples.push_back(singlesample);
				i++;
			}

		} else {
			//m_component->log("point either non free space or seen.");
		}
		//if (giveup > m_samplesize*5){
			//m_component->log("Tried to much giving up, anything can happen after this.");
			//break;
		//}
	}
	m_component->log("Got %d 2D samples", samples.size());

	return samples;
}
std::vector<std::vector<pair<int, int> > > ViewPointGenerator::calculate3DViewConesFrom2D() {
	m_component->log("Calculating 3D view cones for generated 2D samples");
	Cure::Pose3D candidatePose;
	XVector3D a;
	std::vector<pair<int, int> > tpoints;
	std::vector<std::vector<pair<int, int> > > ViewConePts;

	for (unsigned int y = 0; y < m_samples2D.size(); y++) { //calc. view cone for each sample

		lgm->index2WorldCoords(m_samples2D[y].getX(), m_samples2D[y].getY(), a.x,
				a.y);
		a.theta = m_samples2D[y].getTheta();
		tpoints = getInside2DViewCone(lgm, a, true);
		ViewConePts.push_back(tpoints);
	}
	return ViewConePts;
}

std::vector<pair<int, int> > ViewPointGenerator::getInside2DViewCone(
		CureObstMap* lgm, XVector3D &a, bool addall) {
	//m_component->log("get inside view cone");
	std::vector<pair<int, int> > tpoints;
	XVector3D b, c, p;
	XVector3D m_a, m_b, m_c;
	int* rectangle = new int[4];
	int h, k;
	get2DViewConeCorners(a, a.theta, m_conedepth, m_horizangle, b, c);

	lgm->worldCoords2Index(a.x, a.y, h, k);
	m_a.x = h;
	m_a.y = k;
	lgm->worldCoords2Index(b.x, b.y, h, k);
	m_b.x = h;
	m_b.y = k;
	lgm->worldCoords2Index(c.x, c.y, h, k);
	m_c.x = h;
	m_c.y = k;
	//  log("Got Map triangle coordinates: A:(%f,%f),B:(%f,%f),C:(%f,%f) \n", m_a.x,m_a.y,m_b.x,m_b.y,m_c.x,m_c.y);

	findBoundingRectangle(m_a, m_b, m_c, rectangle);
	//log("XRectangle coordinates: Min: (%i,%i), Max:(%i,%i)\n",rectangle[0],rectangle[2]
	//,rectangle[1], rectangle[3]);
	for (int x = rectangle[0]; x < rectangle[1]; x++) // rectangle bounding triangle
	{
		for (int y = rectangle[2]; y < rectangle[3]; y++) {
			p.x = x;
			p.y = y;
			if (isPointInsideTriangle(p, m_a, m_b, m_c)) {
				tpoints.push_back(make_pair(x, y));
			}

		}
	}
	vector<pair<int, int> >::iterator theIterator = tpoints.begin();
	tpoints.insert(theIterator, 1, make_pair(a.x, a.y));
	delete rectangle;
	return tpoints;

}
void ViewPointGenerator::get2DViewConeCorners(XVector3D a, double direction,
		double range, double fov, XVector3D &b, XVector3D &c) {
	//log("Direction: %f, FOV:%f, range: %f", direction, fov, range);
	float angle1 = direction + fov / 2;
	float angle2 = direction - fov / 2;
	b.x = cos(angle1) * range + a.x;
	c.x = cos(angle2) * range + a.x;
	b.y = sin(angle1) * range + a.y;
	c.y = sin(angle2) * range + a.y;
	//log("Got triangle coordinates: A:(%f,%f),B:(%f,%f),C:(%f,%f) \n", a.x,a.y,b.x,b.y,c.x,c.y);
}

void ViewPointGenerator::findBoundingRectangle(XVector3D a, XVector3D b,
		XVector3D c, int* rectangle) {
	int maxx, maxy, minx, miny;
	maxy = max(max(a.y, b.y), c.y);
	maxx = max(max(a.x, b.x), c.x);
	miny = min(min(a.y, b.y), c.y);
	minx = min(min(a.x, b.x), c.x);
	rectangle[0] = minx;
	rectangle[1] = maxx;
	rectangle[2] = miny;
	rectangle[3] = maxy;
	//log("Rectangle coordinates: Min: (%i,%i), Max:(%i,%i)\n",minx,miny,maxx, maxy);
}

bool ViewPointGenerator::isPointInsideTriangle(XVector3D p, XVector3D a,
		XVector3D b, XVector3D c) { //the first one is the point the rest triangle

	if (isPointSameSide(p, a, b, c) && isPointSameSide(p, b, a, c)
			&& isPointSameSide(p, c, a, b)) {
		return true;
	} else {
		return false;
	}

}
bool ViewPointGenerator::isPointSameSide(XVector3D p1, XVector3D p2,
		XVector3D a, XVector3D b) {
	XVector3D cp1 = (b - a).crossVector3D((p1 - a));
	XVector3D cp2 = (b - a).crossVector3D((p2 - a));
	if (cp1.dotVector3D(cp2) >= 0) {
		return true;
	} else {
		return false;
	}

}

bool ViewPointGenerator::isCircleFree2D(const CureObstMap &map, double xW,
		double yW, double rad) {
	int xiC, yiC;
	if (map.worldCoords2Index(xW, yW, xiC, yiC) != 0) {
		CureCERR(30) << "Querying area outside the map (xW=" << xW << ", yW="
					<< yW << ")\n";
		return true;
	}

	double w = rad / map.getCellSize();
	int wi = int(w + 0.5);
	int size = map.getSize();

	for (int x = xiC - wi; x <= xiC + wi; x++) {
		for (int y = yiC - wi; y <= yiC + wi; y++) {
			if (x >= -size && x <= size && y >= -size && y <= size) {
				if (hypot(x - xiC, y - yiC) < w) {
					if (map(x, y) != '0')
						return false;
				}
			}
		}
	}

	return true;
}

}
