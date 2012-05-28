/*
 * ViewPointGenerator.cpp
 *
 *  Created on: Mar 4, 2011
 *      Author: alper
 */

#include "ViewPointGenerator.h"
#include "AVS_ContinualPlanner.h"
#include "TimeLogger.hpp"
#define SCOPED_TIME_LOG TimeLogger logger(m_component, __FILE__, __LINE__);

using namespace SpatialGridMap;
namespace spatial {
ViewPointGenerator::~ViewPointGenerator() {
	// TODO Auto-generated constructor stub

}

ViewPointGenerator::ViewPointGenerator(AVS_ContinualPlanner* component,
		CureObstMap* plgm, BloxelMap* pbloxelmap, int samplesize,
		double sampleawayfromobs, double conedepth, double tiltstep,
		double panstep, double horizangle, double vertangle, double minDistance,
		double pdfsum, double pdfthreshold, double robotx, double roboty) {
	// TODO Auto-generated destructor stub

	m_component = component;
	lgm = new CureObstMap(*plgm);
	bloxelmap = new BloxelMap(*pbloxelmap);
	m_samplesize = samplesize;
	m_sampleawayfromobs = sampleawayfromobs;
	m_conedepth = conedepth;
	m_horizangle = horizangle;
	m_vertangle = vertangle;
	m_minDistance = minDistance;
	m_bloxelmapPDFsum = pdfsum;
	m_pdfthreshold = pdfthreshold;
	m_robotx = robotx;
	m_roboty = roboty;
	m_best3DConeRatio = 0.1;
	m_tiltstep = 0.1;
	m_component->log(
			"ViewPointGenerator parameters: m_samplesize: %d, m_sampleawayfromobs: %f, m_conedepth: %f, m_horizangle: %f, m_vertangle: %f, m_minDistance: %f, m_bloxelmapPDFsum: %f , m_pdfthreshold: %f, robotx %f, roboty %f",
			m_samplesize, m_sampleawayfromobs, m_conedepth, m_horizangle,
			m_vertangle, m_minDistance, m_bloxelmapPDFsum, m_pdfthreshold, m_robotx,
			m_roboty);
	m_component->log("BloxelMap size: %d, %d CureMap size: %d",
			bloxelmap->getMapSize().first, bloxelmap->getMapSize().second,
			lgm->getSize());
	m_sensingProb = 1;
	m_panstep = panstep * M_PI / 180;
	m_tiltstep = tiltstep * M_PI / 180;
}

vector<ViewPointGenerator::SensingAction> ViewPointGenerator::getBest3DViewCones(
		vector<NavData::FNodePtr> &nodes) {

	m_component->log("ViewPointGenerator::GetBest3DViewCones");
	m_lastMapPDFSum = -1;
	std::vector<double> angles;
	for (double rad = -30 * M_PI / 180; rad < -29 * M_PI / 180; rad = rad
			+ m_tiltstep) {
		angles.push_back(rad);
	}

	double totalprobsum = 0;
	double lastConePDFSum = 1;
	vector<SensingAction> result3DVCList;
	{
		SCOPED_TIME_LOG;
		int test_num = 0;
		while ((totalprobsum < m_bloxelmapPDFsum * m_pdfthreshold)
				&& (result3DVCList.size() < 20) && (test_num < 10)) {
			SCOPED_TIME_LOG;
			vector<SensingAction> unordered3DVCList, ordered3DVCList, tmp;
			SensingAction sample;
			std::vector<SensingAction> samplepoints;
			{
				SCOPED_TIME_LOG;
				vector<pair<unsigned int, double> > ordered2DVClist =
						getOrdered2DCandidateViewCones(nodes);

				if (ordered2DVClist.size() == 0) {
					return result3DVCList;
				}

				// We have the VC candidate list ordered according to their 2D pdf sums
				// now for the top X candidate get a bunch of tilt angles and calculate the 3D cone sums
				double xW, yW;
				size_t maxiterations = 1 < ordered2DVClist.size() ? 1
						: ordered2DVClist.size();
				for (size_t j = 0; j < maxiterations; j++) {
					//	for (unsigned int j = 0; j < ordered2DVClist.size() * m_best3DConeRatio; j++) 
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
						sample.totalprob = ordered2DVClist[j].second;
						samplepoints.push_back(sample);
					}
				}

				unordered3DVCList = samplepoints;

				// turn
			}
			int bestindex = -1;
			GDIsObstacle isobstacle;
			{
				SCOPED_TIME_LOG;
				//		unordered3DVCList = getViewConeSums(samplepoints);
				double maxpdf = -1;

				for (unsigned int i = 0; i < unordered3DVCList.size(); i++) {
					if (unordered3DVCList[i].totalprob > maxpdf) {
						maxpdf = unordered3DVCList[i].totalprob;
						bestindex = i;
					}
				}
			}
			{
				SCOPED_TIME_LOG;
				GDProbSum sumcells;
				GDProbScale scalefunctor(1 - m_sensingProb);
				double initialMapPDFSum = 0;
				{
					SCOPED_TIME_LOG;
					if (m_lastMapPDFSum < 0) {
						bloxelmap->universalQuery(sumcells);
						initialMapPDFSum = sumcells.getResult();
						sumcells.reset();
					} else {
						initialMapPDFSum = m_lastMapPDFSum;
					}
					m_component->log(
							"getBest3DViewCones: Before whole map PDF sums to: %f",
							initialMapPDFSum);

					// Got the best cone for this map, now change the map and get a new sum of remaining 3D cones
					bloxelmap->coneModifier(unordered3DVCList[bestindex].pos[0],
							unordered3DVCList[bestindex].pos[1],
							unordered3DVCList[bestindex].pos[2],
							unordered3DVCList[bestindex].pan,
							unordered3DVCList[bestindex].tilt, m_horizangle, m_vertangle,
							m_conedepth, 5, 5, isobstacle, scalefunctor, scalefunctor,
							m_minDistance);
					if (m_component->m_usePeekabot) {
						m_component->displayPDF(*bloxelmap);
					}
				}
				{
					SCOPED_TIME_LOG;

					bloxelmap->universalQuery(sumcells);
					double postMapPDFSum = sumcells.getResult();

					m_component->log(
							"getBest3DViewCones: After whole map PDF sums to: %f",
							postMapPDFSum);

					m_component->log("Best index %d", bestindex);
					//lastConePDFSum = unordered3DVCList[bestindex].totalprob;
					lastConePDFSum = initialMapPDFSum - postMapPDFSum;
					if (lastConePDFSum < 0.01) {
						m_component->log("Best cone's prob. sum. is less than 1%, skip");
						test_num++;
						continue;
					}
					m_lastMapPDFSum = postMapPDFSum;

					totalprobsum += lastConePDFSum;
					unordered3DVCList[bestindex].totalprob = lastConePDFSum;
					result3DVCList.push_back(unordered3DVCList[bestindex]);

					m_component->log(
							"Added new 3DCone to result set with prob: %f, total so far: %f",
							unordered3DVCList[bestindex].totalprob, totalprobsum);
				}
			} //SCOPE
		}
	}
	m_component->log("Returning %d 3D viewcones \n", result3DVCList.size());
	for (unsigned int i = 0; i < result3DVCList.size(); i++) {
		m_component->log("3DCone #%d, sum: %f", i, result3DVCList[i].totalprob);
	}
	return result3DVCList;
}

vector<ViewPointGenerator::SensingAction> ViewPointGenerator::getViewConeSums(
		std::vector<SensingAction> &samplepoints) {

	SpatialGridMap::GDProbSum sumcells;
	SpatialGridMap::GDIsObstacle isobstacle;
	m_component->log("ViewPointGenerator::getViewConeSums");
	m_component->log("Got %d 3D ViewCones", samplepoints.size());

	try {
		for (unsigned int i = 0; i < samplepoints.size(); i++) {
			SensingAction viewpoint = samplepoints[i];
			/*m_map->coneModifier(samplepoints[i].pos[0],samplepoints[i].pos[1],samplepoints[i].pos[2], samplepoints[i].pan,samplepoints[i].tilt, m_horizangle, m_vertangle, m_conedepth, 10, 10, isobstacle, makeobstacle,makeobstacle);*/
			bloxelmap->coneQuery(samplepoints[i].pos[0], samplepoints[i].pos[1],
					samplepoints[i].pos[2], samplepoints[i].pan, samplepoints[i].tilt,
					m_horizangle, m_vertangle, m_conedepth, 5, 5, isobstacle, sumcells,
					sumcells, m_minDistance);
			//	m_component->log("PDFSum of cone: %f.", sumcells.getResult());

			cout << "cone #" << i << " " << viewpoint.pos[0] << " "
					<< viewpoint.pos[1] << " " << viewpoint.pos[2] << " "
					<< viewpoint.pan << " " << viewpoint.tilt << " pdfsum of cone: "
					<< sumcells.getResult() << endl;

			//    /* Show view cone on a temporary map, display the map and wipe it*/
			samplepoints[i].totalprob = sumcells.getResult();
			samplepoints[i].conedepth = m_conedepth;
			samplepoints[i].vertangle = m_vertangle;
			samplepoints[i].horizangle = m_horizangle;
			samplepoints[i].minDistance = m_minDistance;
			sumcells.reset();
		}
	} catch (std::exception &e) {
		printf("Caught exception %s: \n", e.what());
	}

	return samplepoints;
}

vector<pair<unsigned int, double> > ViewPointGenerator::getOrdered2DCandidateViewCones(
		vector<NavData::FNodePtr> &nodes) {

	m_component->log("ViewPointGenerator::getOrdered3DCandidateViewCones");
	std::vector<std::vector<pair<int, int> > > VCones;

	if (m_component->m_sampleRandomPoints)
		m_samples2D = sample2DGrid();
	else
		m_samples2D = sample2DGridFromNodes(nodes);

	vector<pair<unsigned int, double> > orderedVClist, tmp;
	if (m_samples2D.size() == 0) {
		return orderedVClist;
	}
	VCones = calculate2DConesRegion();

	CurePDFMap* lgmpdf = new CurePDFMap(lgm->getSize(), lgm->getCellSize(), 0,
			CureObstMap::MAP1, lgm->getCentXW(), lgm->getCentYW());
	m_component->log("BloxelMap size: %d, %d LGMPDF size: %d",
			bloxelmap->getMapSize().first, bloxelmap->getMapSize().second,
			lgmpdf->getSize());

	// project 3D PDF map to 2D

	for (int x = -lgmpdf->getSize(); x <= lgmpdf->getSize(); x++) {
		for (int y = -lgmpdf->getSize(); y <= lgmpdf->getSize(); y++) {
			int bloxelX = x + lgmpdf->getSize();
			int bloxelY = y + lgmpdf->getSize();
			if ((*lgm)(x, y) != '2') {
				double bloxel_floor = 0;
				(*lgmpdf)(x, y) = 0;
				for (unsigned int i = 0; i < (*bloxelmap)(bloxelX, bloxelY).size(); i++) {
					(*lgmpdf)(x, y) += (*bloxelmap)(bloxelX, bloxelY)[i].data.pdf
							* ((*bloxelmap)(bloxelX, bloxelY)[i].celing - bloxel_floor);
					bloxel_floor = (*bloxelmap)(bloxelX, bloxelY)[i].celing;
				}
			}
		}
	}

	// get sums for each 2D cone

	double sum;
	int x, y;
	vector<unsigned int>::iterator it;
	for (unsigned int i = 0; i < VCones.size(); i++) {
		sum = 0;
		for (unsigned int j = 0; j < VCones[i].size(); j++) {
			x = VCones[i][j].first;
			y = VCones[i][j].second;
			if ((x > -lgmpdf->getSize() && x < lgmpdf->getSize()) && (y
					> -lgmpdf->getSize() && y < lgmpdf->getSize())) {
				sum += (*lgmpdf)(x, y);

			}

		}
		if (orderedVClist.size() == 0) {
			orderedVClist.push_back(make_pair(i, sum));
		} else {
			bool inserted = false;
			tmp = orderedVClist;
			for (unsigned int j = 0; j < tmp.size(); j++) {
				if (sum >= orderedVClist[j].second) {
					inserted = true;
					orderedVClist.insert(orderedVClist.begin() + j, make_pair(i, sum));
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
	//	for(unsigned int i=0; i < orderedVClist.size(); i++){
	//	m_component->log("Sum of VC #%d is %f", i, orderedVClist[i].second);
	//		}
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
				m_NonFreeSpace.setBit(x + lgm->getSize(), y + lgm->getSize(), true);
			}
		}
	}

	// Create an istance of BinaryMatrx which will hold the result of
	// expanding the obstacles
	Cure::BinaryMatrix m_PathGrid;
	m_PathGrid = 0;

	// Grow each occupied cell to account for the size of the
	// robot. We put the result in another binary matrix, m_PathGrid
	m_NonFreeSpace.growInto(m_PathGrid, 4, true);
	// 0.45 is robot width hard coded here.
	// We treat all unknown cells as occupied so that the robot only
	// uses paths that it knowns to be free. Note that we perfom this
	// operation directly on the m_PathGrid, i.e. the grid with the
	// expanded obstacle. The reasoning behind this is that we do not
	// want the unknown cells to be expanded a"s well as we would have
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
			&& lgm->worldCoords2Index(destination.getX(), destination.getY(), rE, cE)
					== 0) {
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
std::vector<Cure::Pose3D> ViewPointGenerator::sample2DGridFromNodes(vector<
		NavData::FNodePtr> &nodes) {
	m_component->log("ViewPointGenerator::sample2DGridFromNodes2DGrid");
	srand( time(NULL));
	std::vector<Cure::Pose3D> samples;
	/*Sampling free space BEGIN*/
	double dist = 0.2;

	for (size_t i = 0; i < nodes.size(); i++) {
		double theta = (rand() % 360) * M_PI / 180;
		for (int j = 0; j < 10; j++) {
			double angle = theta + j / 10 * 2 * M_PI;
			while (angle > M_PI)
				angle -= 2 * M_PI;
			while (angle < -M_PI)
				angle += 2 * M_PI;

			double x = nodes[i]->x + dist * cos(angle);
			double y = nodes[i]->y + dist * sin(angle);
			int cx, cy;
			if (lgm->worldCoords2Index(x, y, cx, cy) == 0) {
				Cure::Pose3D singlesample;
				singlesample.setX(cx);
				singlesample.setY(cy);
				singlesample.setTheta(angle);
				samples.push_back(singlesample);
			}
		}
	}

	m_component->log("Got %d 2D samples", samples.size());

	return samples;
}

std::vector<Cure::Pose3D> ViewPointGenerator::sample2DGrid() {
	srand( time(NULL));
	std::vector<double> angles;

	std::vector<Cure::Pose3D> samples;
	/*Sampling free space BEGIN*/
	m_component->log("ViewPointGenerator::sample2DGrid");
	for (double rad = 0; rad < M_PI * 2; rad = rad + m_panstep) {
		angles.push_back(rad);
	}
	int i = 0;
	int randx, randy;
	double xW, yW, angle;
	cout << "sampling 2D points" << endl;
	m_component->log("Sampling %d points ", m_samplesize);
	bool haspoint;
	Cure::Pose3D singlesample;
	int giveup = 0;
	while (i < m_samplesize && giveup < m_samplesize * 20) {
		giveup++;
		haspoint = false;
		randx = (rand() % (2 * lgm->getSize())) - lgm->getSize();
		randy = (rand() % (2 * lgm->getSize())) - lgm->getSize();
		int the = (int) (rand() % angles.size());
		angle = angles[the];
		//if we have that point already, skip.
		for (int j = 0; j < i; j++) {
			if (samples[j].getX() == randx && samples[j].getY() == randy
					&& samples[j].getTheta() == angle) {
				//log("we already have this point.");
				haspoint = true;
				break;
			} else if ((pow((randx - samples[j].getX()), 2) + (pow((randy
					- samples[j].getY()), 2))) < 0.4) {
				haspoint = true;
				break;
			}
		}
		lgm->index2WorldCoords(randx, randy, xW, yW);
		std::pair<int, int> sample;
		sample.first = randx;
		sample.second = randy;

		if (!haspoint && (*lgm)(randx, randy) == '0' && isCircleFree2D(*lgm, xW,
				yW, m_sampleawayfromobs)) {
			Cure::Pose3D start, dest;
			start.setX(m_robotx);
			start.setY(m_roboty);
			dest.setX(xW);
			dest.setY(yW);
			m_component->log("from: %f, %f", xW, yW);

			double d = getPathLength(start, dest, lgm);
			m_component->log("path to here: %3.2f", d);
			// There is a path to this destination
			//	    log("there's a path to this destination");
			if (d > 0) {
				singlesample.setX(randx);
				singlesample.setY(randy);
				singlesample.setTheta(angle);
				samples.push_back(singlesample);
				i++;
			}
			//			else {
			// cout << "no path to here" << endl;
			//			}

		} else {
			//	printf("point either non free space or seen.");
			//}
			//if (giveup > m_samplesize*5){
			//m_component->log("Tried to much giving up, anything can happen after this.");
			//break;
		}
	}
	m_component->log("Got %d 2D samples", samples.size());

	return samples;
}

/* Check whether P and Q lie on the same side of line AB */
float ViewPointGenerator::Side(XVector3D p, XVector3D q, XVector3D a,
		XVector3D b) {
	float z1 = (b.x - a.x) * (p.y - a.y) - (p.x - a.x) * (b.y - a.y);
	float z2 = (b.x - a.x) * (q.y - a.y) - (q.x - a.x) * (b.y - a.y);
	return z1 * z2;
}

/* Check whether segment P0P1 intersects with triangle t0t1t2 */
int ViewPointGenerator::Intersecting(XVector3D p0, XVector3D p1, XVector3D t0,
		XVector3D t1, XVector3D t2) {
	/* Check whether segment is outside one of the three half-planes
	 * delimited by the triangle. */
	float f1 = Side(p0, t2, t0, t1), f2 = Side(p1, t2, t0, t1);
	float f3 = Side(p0, t0, t1, t2), f4 = Side(p1, t0, t1, t2);
	float f5 = Side(p0, t1, t2, t0), f6 = Side(p1, t1, t2, t0);
	/* Check whether triangle is totally inside one of the two half-planes
	 * delimited by the segment. */
	float f7 = Side(t0, t1, p0, p1);
	float f8 = Side(t1, t2, p0, p1);

	/* If segment is strictly outside triangle, or triangle is strictly
	 * apart from the line, we're not intersecting */
	if ((f1 < 0 && f2 < 0) || (f3 < 0 && f4 < 0) || (f5 < 0 && f6 < 0) || (f7 > 0
			&& f8 > 0))
		return 0;

	/* If segment is aligned with one of the edges, we're overlapping */
	if ((f1 == 0 && f2 == 0) || (f3 == 0 && f4 == 0) || (f5 == 0 && f6 == 0))
		return 0; //OVERLAPPING;

	/* If segment is outside but not strictly, or triangle is apart but
	 * not strictly, we're touching */
	if ((f1 <= 0 && f2 <= 0) || (f3 <= 0 && f4 <= 0) || (f5 <= 0 && f6 <= 0)
			|| (f7 >= 0 && f8 >= 0))
		return 0; //TOUCHING;

	/* If both segment points are strictly inside the triangle, we
	 * are not intersecting either */
	if (f1 > 0 && f2 > 0 && f3 > 0 && f4 > 0 && f5 > 0 && f6 > 0)
		return 0; //NOT_INTERSECTING;

	/* Otherwise we're intersecting with at least one edge */
	return 1;
}

int ViewPointGenerator::TrianglesIntersecting(XVector3D p0, XVector3D p1,
		XVector3D p2, XVector3D t0, XVector3D t1, XVector3D t2) {
	if (!Intersecting(p0, p1, t0, t1, t2) && !Intersecting(p0, p2, t0, t1, t2)
			&& !Intersecting(p1, p2, t0, t1, t2))
		return 0;
	return 1;
}

void ViewPointGenerator::findIntersectingCones2D() {
	for (int y = 0; y < m_samples2D.size(); y++) { //calc. view cone for each sample
		XVector3D a;
		lgm->index2WorldCoords(m_samples2D[y].getX(), m_samples2D[y].getY(), a.x,
				a.y);
		a.theta = m_samples2D[y].getTheta();
		XVector3D b, c;
		XVector3D m_a, m_b, m_c;
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
		for (int z = 0; z < m_samples2D.size(); z++) { //calc. view cone for each sample
			if (y != z) {
				XVector3D a1;
				lgm->index2WorldCoords(m_samples2D[z].getX(), m_samples2D[z].getY(),
						a1.x, a1.y);
				a1.theta = m_samples2D[z].getTheta();
				XVector3D b1, c1;
				XVector3D m_a1, m_b1, m_c1;
				get2DViewConeCorners(a1, a1.theta, m_conedepth, m_horizangle, b1, c1);

				lgm->worldCoords2Index(a1.x, a1.y, h, k);
				m_a1.x = h;
				m_a1.y = k;
				lgm->worldCoords2Index(b1.x, b1.y, h, k);
				m_b1.x = h;
				m_b1.y = k;
				lgm->worldCoords2Index(c1.x, c1.y, h, k);
				m_c1.x = h;
				m_c1.y = k;
				if (ViewPointGenerator::TrianglesIntersecting(m_a1, m_b1, m_c1, m_a,
						m_b, m_c)) {
					m_viewconesIntersections[y].push_back(z);
					m_viewconesIntersections[z].push_back(y);
				}
			}
		}
	}
}

std::vector<std::vector<pair<int, int> > > ViewPointGenerator::calculate2DConesRegion() {
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
		CureCERR(30) << "Querying area outside the map (xW=" << xW << ", yW=" << yW
				<< ")\n";
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
