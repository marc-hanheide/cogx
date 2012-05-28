// = FILENAME
//    ObjectRelationTester.cpp
//
// = FUNCTION
//    Testing and visualization of code relating to object relation computations
//
// = AUTHOR(S)
//    Kristoffer Sjöö
//
// = COPYRIGHT
//    Copyright (c) 2010 Kristoffer Sjöö
//
/*----------------------------------------------------------------------*/
#include "ObjectRelationTester.hpp"
#include <AddressBank/ConfigFileReader.hh>
//#include "PBVisualization.hh"

//bool
//inferRelationsThreeObjects(vector<double> &ret, double BOnA, double AOnB, double BOnT,
//    double AOnT, double BInA, double AInB, double BInT, double AInT);

using namespace cast;
#include <Pose3.h>

using namespace std;
using namespace cast;
using namespace boost;
using namespace spatial;
using namespace cogx::Math;

/**
 * The function called to create a new instance of our component.
 *
 * Taken from zwork
 */
extern "C" {
cast::interfaces::CASTComponentPtr newComponent() {
	return new ObjectRelationTester();
}
}

ObjectRelationTester::ObjectRelationTester() :
	m_sampler(&m_evaluator) {
}

ObjectRelationTester::~ObjectRelationTester() {
}

void ObjectRelationTester::configure(const map<string, string>& _config) {
	m_bTestOnness = false;
	m_bSampleOnness = false;

	map<string, string>::const_iterator it = _config.find("--test-onness");
	if (it != _config.end()) {
		m_bTestOnness = true;
		it = _config.find("--sample-onness");
		if (it != _config.end()) {
			m_bSampleOnness = true;
		}
	}

	m_bTestInness = false;
	m_bSampleInness = false;
	it = _config.find("--test-inness");
	if (it != _config.end()) {
		m_bTestInness = true;
		it = _config.find("--sample-inness");
		if (it != _config.end()) {
			m_bSampleInness = true;
		}
	}

	m_bDemoSampling = false;
	it = _config.find("--demo-sampling");
	if (it != _config.end()) {
		m_bDemoSampling = true;
	}

	m_bDemoSampling = false;
	it = _config.find("--show-poses");
	if (it != _config.end()) {
		m_bShowPoses = true;
	}

	m_RetryDelay = 1000;
	if (_config.find("--retry-interval") != _config.end()) {
		std::istringstream str(_config.find("--retry-interval")->second);
		str >> m_RetryDelay;
	}

	//  m_bTestInference = false;
	//  it = _config.find("--test-inference");
	//  if (it != _config.end()) {
	//    m_bTestInference = true;
	//  }

	m_PbPort = 5050;
	m_PbHost = "localhost";

	Cure::ConfigFileReader *cfg = 0;
	it = _config.find("-c");

	if (it != _config.end()) {
		cfg = new Cure::ConfigFileReader;
		log("About to try to open the config file");
		if (cfg->init(it->second) != 0) {
			delete cfg;
			cfg = 0;
			log("Could not init Cure::ConfigFileReader with -c argument");
		} else {
			log("Managed to open the Cure config file");
		}
	}

	if (cfg) {
		std::string usedCfgFile, tmp;
		if (cfg && cfg->getString("PEEKABOT_HOST", true, tmp, usedCfgFile) == 0) {
			m_PbHost = tmp;
		}
	}

	connectPeekabot();
}

void ObjectRelationTester::start() {
	if (m_bTestOnness || m_bTestInness) {// || m_bTestInference) {
		while (!m_PeekabotClient.is_connected() && (m_RetryDelay > -1)) {
			connectPeekabot();
			sleepComponent( m_RetryDelay);
		}

		if (m_bTestOnness || m_bTestInness) {
			m_relationTester.add(m_PeekabotClient, "relation_tester",
					peekabot::REPLACE_ON_CONFLICT);
		}
		//    else if (m_bTestInference) {
		//      m_relationTester.add(m_PeekabotClient, "inference_tester", peekabot::REPLACE_ON_CONFLICT);
		//    }
		println("Connected to peekabot, ready to go");
	}
}

void ObjectRelationTester::runComponent() {
	log("I am running!");

	peekabot::SphereProxy sqdp;
	peekabot::SphereProxy scwp;
	peekabot::SphereProxy bcwp;
	peekabot::SphereProxy op;
	peekabot::CubeProxy csp;
	peekabot::CubeProxy cop;
	peekabot::CubeProxy bp;
	peekabot::CubeProxy bp2;
	peekabot::CubeProxy bp3;

	peekabot::SphereProxy sp;
	peekabot::SphereProxy spm;
	peekabot::SphereProxy sp2;
	peekabot::SphereProxy spm2;

	if (m_bTestOnness || m_bTestInness) {// || m_bTestInference) {
		BoxObject *table1 = new BoxObject;
		m_testObjects.push_back(table1);

		table1->type = OBJECT_BOX;

		//    Matrix33 rotation;
		//    double rotAngle = 0.0;
		//    fromAngleAxis(rotation, rotAngle, vector3(0.0, 0.0, 1.0));
		//    table1->pose = pose3(vector3(0.0, 0.0, 0.5), rotation);

		//    table1->shape = PLANE_OBJECT_RECTANGLE;
		table1->radius1 = 0.5;
		table1->radius2 = 0.5;
		table1->radius3 = 0.5;
		setIdentity(table1->pose);
		table1->pose.pos = vector3(0, 0, 0);

		addProxy(table1, "table");

		//    bp3.add(m_relationTester, "table", peekabot::REPLACE_ON_CONFLICT);

		//    peekabot::VertexSet polyVerts;

		//    pp.add(m_relationTester, "table", peekabot::REPLACE_ON_CONFLICT);
		//    pp.set_color(1.0, 1.0, 0);
		//    polyVerts.add(table1->radius1, table1->radius2, 0);
		//    polyVerts.add(-table1->radius1, table1->radius2, 0);
		//    polyVerts.add(-table1->radius1, -table1->radius2, 0);
		//    polyVerts.add(table1->radius1, -table1->radius2, 0);

		//    pp.add_vertices(polyVerts);

		//    bp3.translate(table1.pose.pos.x, table1.pose.pos.y, table1.pose.pos.z);
		//    bp3.rotate(rotAngle, 0.0, 0.0, 1.0);

		spatial::BoxObject *krispies = new spatial::BoxObject;
		krispies->type = spatial::OBJECT_BOX;
		krispies->radius1 = 0.095;
		krispies->radius2 = 0.045;
		krispies->radius3 = 0.145;
		setIdentity(krispies->pose);
		krispies->pose.pos.x = table1->pose.pos.x;
		krispies->pose.pos.y = table1->pose.pos.y;
		krispies->pose.pos.z = table1->pose.pos.z + 0.5 + 0.26 + 0.145;
		m_testObjects.push_back(krispies);
		addProxy(krispies, "krispies");
		//
		//    bp.add(m_relationTester, "krispies", peekabot::REPLACE_ON_CONFLICT);
		//    bp.translate(table1.pose.pos.x, table1.pose.pos.y, table1.pose.pos.z+0.5 + 0.26+0.145);
		//    bp.set_scale(0.19, 0.09, 0.29);
		//    bp.set_color(0.0, 0.0, 1.0);
		//    bp.set_opacity(0.5);

		spatial::BoxObject* joystick = new spatial::BoxObject;
		joystick->type = spatial::OBJECT_BOX;
		joystick->radius1 = 0.115;
		joystick->radius2 = 0.105;
		joystick->radius3 = 0.130;
		setIdentity(joystick->pose);
		joystick->pose.pos.x = table1->pose.pos.x;
		joystick->pose.pos.y = table1->pose.pos.y;
		joystick->pose.pos.z = table1->pose.pos.z + 0.5 + 0.13;
		m_testObjects.push_back(joystick);
		addProxy(joystick, "joystick");

		spatial::HollowBoxObject *redbox = new spatial::HollowBoxObject;
		redbox-> type = spatial::OBJECT_HOLLOW_BOX;
		redbox->radius1 = 0.27;
		redbox->radius2 = 0.165;
		redbox->radius3 = 0.135;
		redbox->thickness = 0.051;
		redbox->sideOpen = 5;
		setIdentity(redbox->pose);
		redbox->pose.pos.x = table1->pose.pos.x + 0.3;
		redbox->pose.pos.y = table1->pose.pos.y;
		redbox->pose.pos.z = table1->pose.pos.z + 0.5 + 0.135;
		fromAngleAxis(redbox->pose.rot, -0.5 * M_PI, vector3(0.0, 0.0, 1.0));
		m_testObjects.push_back(redbox);
		addProxy(redbox, "redbox");

		//    bp2.add(m_relationTester, "joystick", peekabot::REPLACE_ON_CONFLICT);
		//    bp2.translate(table1.pose.pos.x, table1.pose.pos.y, table1.pose.pos.z+0.5 + 0.13);
		//    bp2.set_scale(0.23, 0.21, 0.26);
		//    bp2.set_color(1.0, 0.0, 0.0);
		//    bp2.set_opacity(0.5);

		if (m_bTestOnness) {
			peekabot::GroupProxy sliders;
			sliders.add(m_relationTester, "weights", peekabot::REPLACE_ON_CONFLICT);

			sqdp.add(sliders, "squareDistanceOutside", peekabot::REPLACE_ON_CONFLICT);
			sqdp.translate(-1.0, 6.0, 10 * m_evaluator.distanceFalloffOutside);
			sqdp.set_scale(0.1);
			scwp.add(sliders, "squareDistanceInside", peekabot::REPLACE_ON_CONFLICT);
			scwp.translate(0.0, 6.0, 10 * m_evaluator.distanceFalloffInside);
			scwp.set_scale(0.1);
			bcwp.add(sliders, "patchThreshold", peekabot::REPLACE_ON_CONFLICT);
			bcwp.translate(1.0, 6.0, 10 * m_evaluator.patchThreshold);
			bcwp.set_scale(0.1);
			csp.add(sliders, "containmentSteepness", peekabot::REPLACE_ON_CONFLICT);
			csp.translate(0.0, 6.0, 10 * m_evaluator.supportCOMContainmentSteepness);
			csp.set_scale(0.1);
			cop.add(sliders, "containmentOffset", peekabot::REPLACE_ON_CONFLICT);
			cop.translate(0.0, 6.0, m_evaluator.supportCOMContainmentOffset);
			cop.set_scale(0.1);

			sp.add(m_relationTester, "Onness", peekabot::REPLACE_ON_CONFLICT);
			sp.translate(0.0, 3.0, 1.0);
			spm.add(m_relationTester, "Onness-max", peekabot::REPLACE_ON_CONFLICT);
			spm.translate(0.0, 3.0, 1.0);
			spm.set_opacity(0.3);
			sp2.add(m_relationTester, "Onness2", peekabot::REPLACE_ON_CONFLICT);
			sp2.translate(2.0, 3.0, 1.0);
			spm2.add(m_relationTester, "Onness-max2", peekabot::REPLACE_ON_CONFLICT);
			spm2.translate(2.0, 3.0, 1.0);
			spm2.set_opacity(0.3);
		}
		if (m_bTestInness) {
			sp.add(m_relationTester, "Inness", peekabot::REPLACE_ON_CONFLICT);
			sp.translate(0.0, 3.0, 1.0);
			spm.add(m_relationTester, "Inness-max", peekabot::REPLACE_ON_CONFLICT);
			spm.translate(0.0, 3.0, 1.0);
			spm.set_opacity(0.3);
			sp2.add(m_relationTester, "Inness2", peekabot::REPLACE_ON_CONFLICT);
			sp2.translate(2.0, 3.0, 1.0);
			spm2.add(m_relationTester, "Inness-max2", peekabot::REPLACE_ON_CONFLICT);
			spm2.translate(2.0, 3.0, 1.0);
			spm2.set_opacity(0.3);
		}
	}

	vector<Vector3> points;
	int nPoints = 0;
	int maxPoints = 2500;

	peekabot::PointCloudProxy pcloud;
	peekabot::GroupProxy poses;
	if (!m_bShowPoses) {
		if (m_bSampleOnness) {
			pcloud.add(m_relationTester, "onpoints", peekabot::REPLACE_ON_CONFLICT);
			points.reserve(maxPoints);
		} else if (m_bSampleInness) {
			pcloud.add(m_relationTester, "inpoints", peekabot::REPLACE_ON_CONFLICT);
			points.reserve(maxPoints);
		}
	} else {
		if (m_bSampleOnness) {
			poses.add(m_relationTester, "onpoints", peekabot::REPLACE_ON_CONFLICT);
		} else if (m_bSampleInness) {
			poses.add(m_relationTester, "inpoints", peekabot::REPLACE_ON_CONFLICT);
		}
	}

	while (isRunning()) {
		if (m_bTestOnness || m_bTestInness) {// || m_bTestInference) {
			peekabot::Result<peekabot::Transformation> r;
			updatePosesFromPB();

			//      if (m_bTestInference) {
			//	double BOnA = m_evaluator.evaluateOnness(m_testObjects[1], m_testObjects[2]);
			//	double AOnB = m_evaluator.evaluateOnness(m_testObjects[2], m_testObjects[1]);
			//	double AOnT = m_evaluator.evaluateOnness(m_testObjects[0], m_testObjects[1]);
			//	double BOnT = m_evaluator.evaluateOnness(m_testObjects[0], m_testObjects[2]);
			//	double BInA = m_evaluator.evaluateInness(m_testObjects[1], m_testObjects[2]);
			//	double AInB = m_evaluator.evaluateInness(m_testObjects[2], m_testObjects[1]);
			//	double AInT = m_evaluator.evaluateInness(m_testObjects[0], m_testObjects[1]);
			//	double BInT = m_evaluator.evaluateInness(m_testObjects[0], m_testObjects[2]);
			//	vector<double> inferred;
			//	if (inferRelationsThreeObjects(inferred,
			//	      BOnA, AOnB, BOnT, AOnT, BInA, AInB, BInT, AInT)) {
			//	  log("B on A: %f", inferred[11]);
			//	  log("B on table: %f", inferred[5]);
			//	  log("B on_t A: %f", inferred[10]);
			//	  log("B on_t table: %f", inferred[4]);
			//	  log("B in A: %f", inferred[9]);
			//	  log("B in table: %f", inferred[3]);
			//	  log("A on B: %f", inferred[8]);
			//	  log("A on table: %f", inferred[2]);
			//	  log("A on_t B: %f", inferred[7]);
			//	  log("A on_t table: %f", inferred[1]);
			//	  log("A in B: %f", inferred[6]);
			//	  log("A in table: %f", inferred[0]);
			//	}
			//	else {
			//	  log("Error! inferRelationsThreeObjects failed!");
			//	}
			//      }

			if (m_bTestOnness) {
				peekabot::Result<peekabot::Transformation> vr;
				vr = sqdp.get_transformation();
				if (vr.succeeded())
					m_evaluator.distanceFalloffOutside = 0.1 * vr.get_result().pos.z;
				vr = scwp.get_transformation();
				if (vr.succeeded())
					m_evaluator.distanceFalloffInside = 0.1 * vr.get_result().pos.z;
				vr = bcwp.get_transformation();
				if (vr.succeeded())
					m_evaluator.patchThreshold = 0.1 * vr.get_result().pos.z;
				vr = csp.get_transformation();
				if (vr.succeeded()) {
					m_evaluator.supportCOMContainmentSteepness = 0.1
							* vr.get_result().pos.z;
				}
				vr = cop.get_transformation();
				if (vr.succeeded()) {
					m_evaluator.supportCOMContainmentOffset = vr.get_result().pos.z;
				}

				double scale1 = m_evaluator.evaluateOnness(m_testObjects[0],
						m_testObjects[1]);
				double scale2 = m_evaluator.evaluateOnness(m_testObjects[2],
						m_testObjects[1]);
				Witness w1 = m_evaluator.m_lastWitness;
				m_evaluator.evaluateOnness(m_testObjects[1], m_testObjects[2]);
				Witness w2 = m_evaluator.m_lastWitness;
				if (w1.distance - w2.distance > 0.0001 || w1.distance - w2.distance
						< -0.0001) {
					log("Error!");
				}
				//	log("%f, %f", scale1, scale2);
				sp.set_scale(scale1);
				sp2.set_scale(scale2);

				if (m_testObjects[0]->type == spatial::OBJECT_BOX
						&& m_testObjects[1]->type == spatial::OBJECT_BOX) {
					const spatial::BoxObject &box1 =
							*static_cast<spatial::BoxObject*> (m_testObjects[1]);
					const spatial::BoxObject &box2 =
							*static_cast<spatial::BoxObject*> (m_testObjects[2]);
					vector<Vector3> patch;
					Witness witness = m_evaluator.findContactPatch(box2, box1, &patch);
					if (patch.size() > 2) {
						peekabot::PolygonProxy patchp;
						peekabot::VertexSet polyVerts;
						patchp.add(m_relationTester, "Patch", peekabot::REPLACE_ON_CONFLICT);
						patchp.set_color(1, 0, 0);
						for (vector<Vector3>::iterator it = patch.begin(); it
								!= patch.end(); it++) {
							polyVerts.add(it->x, it->y, it->z);
						}
						patchp.add_vertices(polyVerts);

						Vector3 avs = m_evaluator.computeAttentionVectorSumForSolid(
								m_testObjects[2], witness.point1, box1.pose.pos, 0.5);
						//	      Vector3 avs = m_evaluator.computeAttentionVectorSumForPatch(patch, witness.point1, box1.pose.pos,
						//		  0.1);
						normalise(avs);
						peekabot::PolylineProxy plp;
						peekabot::VertexSet plps;
						plps.add(box1.pose.pos.x, box1.pose.pos.y, box1.pose.pos.z);
						Vector3 tmp = box1.pose.pos - avs;
						plps.add(tmp.x, tmp.y, tmp.z);
						plp.add(m_relationTester, "AVS", peekabot::REPLACE_ON_CONFLICT);
						plp.add_vertices(plps);
					}
					//	  peekabot::CylinderProxy normp;
					//	  normp.add(m_relationTester, "Normal", peekabot::REPLACE_ON_CONFLICT);
					//	  normp.set_color(0,0,1);
					//	  normp.set_scale(0.005, 0.005, 0.1);
					//	  normp.translate(0.05,0,0);
					//	  normp.set_orientation(witness.normal.x, witness.normal.y, witness.normal.z);
					//	  normp.rotate(M_PI/2, 0, 1, 0);
					//	  normp.translate(witness.point1.x, witness.point1.y, witness.point1.z,
					//	      peekabot::PARENT_COORDINATES);


					peekabot::SphereProxy witp1;
					witp1.add(m_relationTester, "Witness 1",
							peekabot::REPLACE_ON_CONFLICT);
					witp1.translate(witness.point1.x, witness.point1.y, witness.point1.z);
					witp1.set_scale(0.01);
					peekabot::SphereProxy witp2;
					witp2.add(m_relationTester, "Witness 2",
							peekabot::REPLACE_ON_CONFLICT);
					witp2.translate(witness.point2.x, witness.point2.y, witness.point2.z);
					witp2.set_scale(0.01);
				}

				if (m_bSampleOnness) {
					if (m_testObjects.size() > 2) {
						if (nPoints < maxPoints) {
							vector<spatial::Object*> testObjects;
							vector<spatial::SpatialRelationType> types;

							// HERE: Select trajector and any intermediates

							//Trajector
							testObjects.push_back(m_testObjects[1]);
							types.push_back(spatial::RELATION_ON);

							//	      //Intermediate
							//	      testObjects.push_back(m_testObjects[2]);//
							//	      types.push_back(spatial::RELATION_ON);

							vector<vector<Pose3> > points;
							points.reserve(500);

							sampleRecursively(types, testObjects, 100, 500, points,

							// HERE: Select base object

									// A on Table
									m_testObjects[0]);
							//		  // A on B
							//		  m_testObjects[2]);

							log("Found %i points", points.size());

							if (m_bShowPoses) {
								for (vector<vector<Pose3> >::iterator it = points.begin(); it
										!= points.end(); it++) {

									for (unsigned int level = 0; level < it->size(); level++) {

										BoxObject *boxobject;
										if (m_testObjects[level]->type == OBJECT_BOX
												|| m_testObjects[level]->type == OBJECT_HOLLOW_BOX) {
											boxobject = static_cast<BoxObject*> (testObjects[level]);
										}

										peekabot::CubeProxy dummy;
										dummy.add(poses, "pt", peekabot::AUTO_ENUMERATE_ON_CONFLICT);
										float m[16];
										getRow44((*it)[level], m);
										dummy.set_transformation(m, true);
										dummy.set_scale(boxobject->radius1 * 2, boxobject->radius2
												* 2, boxobject->radius3 * 2);
										dummy.set_visibility(false);
										//points.push_back(box1.pose.pos);
									}
									nPoints++;
								}
							} else {
								peekabot::VertexSet vs;
								for (vector<vector<Pose3> >::iterator it = points.begin(); it
										!= points.end(); it++) {
									//  if (evaluateOnness(&box2, &box1) > ((double)rand())/RAND_MAX) 
									//    if (nPoints > 500) 
									const Pose3 &pose = (*it)[0];
									vs.add(pose.pos.x, pose.pos.y, pose.pos.z);
									//points.push_back(box1.pose.pos);
									nPoints++;
								}
								pcloud.add_vertices(vs);
							}
						}
					}
				}
			} // if (m_bTestOnness)

			if (m_bTestInness) {
				sp.set_scale(m_evaluator.evaluateInness(m_testObjects[0],
						m_testObjects[1]));
				sp2.set_scale(m_evaluator.evaluateInness(m_testObjects[3],
						m_testObjects[1]));

				if (m_bSampleInness) {
					if (m_testObjects.size() > 3) {
						if (nPoints < maxPoints) {
							vector<spatial::Object*> testObjects;
							vector<vector<Pose3> > points;
							points.reserve(500);
							testObjects.push_back(m_testObjects[1]);
							vector<spatial::SpatialRelationType> types;
							types.push_back(spatial::RELATION_IN);

							sampleRecursively(types, testObjects, 100, 500, points,
									m_testObjects[3]);
							log("Found %i points", points.size());

							if (m_bShowPoses) {
								for (vector<vector<Pose3> >::iterator it = points.begin(); it
										!= points.end(); it++) {
									for (unsigned int level = 0; level < it->size(); level++) {

										BoxObject *boxobject;
										if (m_testObjects[level]->type == OBJECT_BOX
												|| m_testObjects[level]->type == OBJECT_HOLLOW_BOX) {
											boxobject = static_cast<BoxObject*> (testObjects[level]);
										}

										peekabot::CubeProxy dummy;
										dummy.add(poses, "pt", peekabot::AUTO_ENUMERATE_ON_CONFLICT);
										float m[16];
										getRow44((*it)[level], m);
										dummy.set_transformation(m, true);
										dummy.set_scale(boxobject->radius1 * 2, boxobject->radius2
												* 2, boxobject->radius3 * 2);
										dummy.set_visibility(false);
										//points.push_back(box1.pose.pos);
									}
								}
								nPoints++;
							} else {
								peekabot::VertexSet vs;
								for (vector<vector<Pose3> >::iterator it = points.begin(); it
										!= points.end(); it++) {
									//  if (evaluateOnness(&box2, &box1) > ((double)rand())/RAND_MAX) 
									//    if (nPoints > 500) 
									const Pose3 &pose = (*it)[0];
									vs.add(pose.pos.x, pose.pos.y, pose.pos.z);
									//points.push_back(box1.pose.pos);
									nPoints++;
								}
								pcloud.add_vertices(vs);
							}
						}
					}
				}
			} // if (m_bTestInness)
		}

		sleepComponent(500);
	}
}

void ObjectRelationTester::addProxy(const spatial::Object* obj,
		const string &label) {
	peekabot::GroupProxy group;
	ostringstream ostr;

	group.add(m_relationTester, label.c_str(), peekabot::REPLACE_ON_CONFLICT);
	m_testObjectProxies.push_back(group);

	if (obj->type == spatial::OBJECT_BOX) {
		peekabot::CubeProxy cube;
		const spatial::BoxObject* bo = static_cast<const spatial::BoxObject*> (obj);
		cube.add(group, "Shape", peekabot::REPLACE_ON_CONFLICT);
		cube.set_scale(2 * bo->radius1, 2 * bo->radius2, 2 * bo->radius3);
	} else if (obj->type == spatial::OBJECT_HOLLOW_BOX) {
		const spatial::HollowBoxObject* hbo =
				static_cast<const spatial::HollowBoxObject*> (obj);
		if (hbo->sideOpen != 1) {
			peekabot::CubeProxy cube1;
			cube1.add(group, "Side1", peekabot::REPLACE_ON_CONFLICT);
			cube1.translate(hbo->radius1 - hbo->thickness / 2, 0, 0);
			cube1.set_scale(hbo->thickness, 2 * hbo->radius2, 2 * hbo->radius3);
		}
		if (hbo->sideOpen != 2) {
			peekabot::CubeProxy cube2;
			cube2.add(group, "Side2", peekabot::REPLACE_ON_CONFLICT);
			cube2.translate(-hbo->radius1 + hbo->thickness / 2, 0, 0);
			cube2.set_scale(hbo->thickness, 2 * hbo->radius2, 2 * hbo->radius3);
		}
		if (hbo->sideOpen != 3) {
			peekabot::CubeProxy cube3;
			cube3.add(group, "Side3", peekabot::REPLACE_ON_CONFLICT);
			cube3.translate(0, hbo->radius2 - hbo->thickness / 2, 0);
			cube3.set_scale(2 * hbo->radius1, hbo->thickness, 2 * hbo->radius3);
		}
		if (hbo->sideOpen != 4) {
			peekabot::CubeProxy cube4;
			cube4.add(group, "Side4", peekabot::REPLACE_ON_CONFLICT);
			cube4.translate(0, -hbo->radius2 + hbo->thickness / 2, 0);
			cube4.set_scale(2 * hbo->radius1, hbo->thickness, 2 * hbo->radius3);
		}
		if (hbo->sideOpen != 5) {
			peekabot::CubeProxy cube5;
			cube5.add(group, "Side5", peekabot::REPLACE_ON_CONFLICT);
			cube5.translate(0, 0, hbo->radius3 - hbo->thickness / 2);
			cube5.set_scale(2 * hbo->radius1, 2 * hbo->radius2, hbo->thickness);
		}
		if (hbo->sideOpen != 6) {
			peekabot::CubeProxy cube6;
			cube6.add(group, "Side6", peekabot::REPLACE_ON_CONFLICT);
			cube6.translate(0, 0, -hbo->radius3 + hbo->thickness / 2);
			cube6.set_scale(2 * hbo->radius1, 2 * hbo->radius2, hbo->thickness);
		}
	}
	float m[16];
	getRow44(obj->pose, m);
	group.set_transformation(m, true);

	//  group.set_position(obj->pose.pos.x, obj->pose.pos.y, obj->pose.pos.z);

	//  log("set pos %f %f %f", obj->pose.pos.x, obj->pose.pos.y, obj->pose.pos.z);
}

void ObjectRelationTester::updatePosesFromPB() {
	if (m_testObjectProxies.size() != m_testObjects.size()) {
		cerr << "Error on " << __FILE__ << ": " << __LINE__ << "\n";
		exit(1);
	}

	for (unsigned int i = 0; i < m_testObjectProxies.size(); i++) {
		peekabot::Result<peekabot::Transformation> r =
				m_testObjectProxies[i].get_transformation(peekabot::WORLD_COORDINATES);
		if (r.succeeded()) {
			double m[16];

			m[0] = r.get_result().x.x;
			m[1] = r.get_result().y.x;
			m[2] = r.get_result().z.x;
			m[3] = r.get_result().pos.x;
			m[4] = r.get_result().x.y;
			m[5] = r.get_result().y.y;
			m[6] = r.get_result().z.y;
			m[7] = r.get_result().pos.y;
			m[8] = r.get_result().x.z;
			m[9] = r.get_result().y.z;
			m[10] = r.get_result().z.z;
			m[11] = r.get_result().pos.z;
			m[12] = 0;
			m[13] = 0;
			m[14] = 0;
			m[15] = 0;

			setRow44(m_testObjects[i]->pose, m);
			log("Read %f %f %f", m_testObjects[i]->pose.pos.x,
					m_testObjects[i]->pose.pos.y, m_testObjects[i]->pose.pos.z);
		}
	}
}

void ObjectRelationTester::connectPeekabot() {
	try {
		log("Trying to connect to Peekabot (again) on host %s and port %d",
				m_PbHost.c_str(), m_PbPort);

		m_PeekabotClient.connect(m_PbHost, m_PbPort);

	} catch (std::exception &e) {
		log("Caught exception when connecting to peekabot (%s)", e.what());
		return;
	}
}

void ObjectRelationTester::sampleOnnessForObject(
		const spatial::Object *objectS, spatial::Object *objectO) {
	//  ASSERT_TYPE(objectS);
	//  ASSERT_TYPE(objectO);

	Pose3 oldPose = objectO->pose;
	vector<Vector3> points;

	BoxObject *supportBox = (BoxObject *) objectS;

	double frameRadius =
			supportBox->radius1 > supportBox->radius2 ? supportBox->radius1
					: supportBox->radius2;
	frameRadius = frameRadius > supportBox->radius3 ? frameRadius
			: supportBox->radius3;
	m_evaluator.sampleOnnessDistribution(objectS, objectO, points,
			objectS->pose.pos.x - frameRadius, objectS->pose.pos.x + frameRadius,
			objectS->pose.pos.y - frameRadius, objectS->pose.pos.y + frameRadius,
			objectS->pose.pos.z - frameRadius, objectS->pose.pos.z + frameRadius * 4,
			0.04, 0.02);

	objectO->pose = oldPose;

	peekabot::PointCloudProxy pcloud;
	peekabot::VertexSet cloudPoints;
	pcloud.add(m_relationTester, "onpoints", peekabot::REPLACE_ON_CONFLICT);

	for (vector<Vector3>::iterator it = points.begin(); it != points.end(); it++) {
		cloudPoints.add(it->x, it->y, it->z);
	}
	pcloud.add_vertices(cloudPoints);
}

void ObjectRelationTester::sampleRecursively(const vector<
		spatial::SpatialRelationType> &relationTypes,
		const vector<spatial::Object*> &objects, unsigned int nSamplesPerStep,
		unsigned int nMaxSamples, vector<vector<Pose3> > &outPoints,
		spatial::Object *supportObject, unsigned int currentLevel
//    , const vector<Vector3> &triangle
) {
	if (currentLevel == UINT_MAX)
		currentLevel = objects.size() - 1;

	if (supportObject->pose.pos.x == -FLT_MAX) {
		log("Error! Support object pose uninitialized!");
		return;
	}
	spatial::Object *trajector = objects[currentLevel];

	Pose3 oldPose = trajector->pose;

	double frameRadius;
	if (supportObject->type == spatial::OBJECT_PLANE) {
		spatial::PlaneObject &table1 = (spatial::PlaneObject &) (*supportObject);
		if (table1.shape == spatial::PLANE_OBJECT_RECTANGLE) {
			frameRadius = table1.radius1 > table1.radius2 ? table1.radius1
					: table1.radius2;
		} else {
			log("Unsupported object type!");
			return;
		}
	} else if (supportObject->type == spatial::OBJECT_BOX || supportObject->type
			== spatial::OBJECT_HOLLOW_BOX) {
		spatial::BoxObject &box1 = (spatial::BoxObject &) (*supportObject);
		frameRadius = box1.radius1 > box1.radius2 ? box1.radius1 : box1.radius2;
		frameRadius = frameRadius > box1.radius3 ? frameRadius : box1.radius3;
	} else {
		log("Unsupported object type!");
	}

	double maxLateral, minVertical, maxVertical;
	if (relationTypes[currentLevel] == RELATION_ON) {
		maxLateral = -frameRadius * 1.5;
		minVertical = -frameRadius * 1.5;
		maxVertical = frameRadius * 3;
	} else {
		maxLateral = -frameRadius;
		minVertical = -frameRadius;
		maxVertical = frameRadius;
	}

	unsigned int pointsFound = 0;
	unsigned int iterations = 0;
	while (pointsFound < nSamplesPerStep && outPoints.size() < nMaxSamples
			&& iterations < 10000) {
		iterations++;
		trajector->pose.pos.x = (((double) rand()) / RAND_MAX) * (2 * maxLateral)
				- maxLateral + supportObject->pose.pos.x;
		trajector->pose.pos.y = (((double) rand()) / RAND_MAX) * (2 * maxLateral)
				- maxLateral + supportObject->pose.pos.y;

		//    if (triangle.size() > 0 && currentLevel == 0 &&
		//	!isInTriangle(trajector->pose.pos.x, trajector->pose.pos.y, triangle))
		//	continue;

		trajector->pose.pos.z = (((double) rand()) / RAND_MAX) * (maxVertical
				- minVertical) + minVertical + supportObject->pose.pos.z;

		randomizeOrientation(trajector->pose);
		//currentLevel is 0 for the trajector, 1 for whatever it's on/in and so on
		double
				value =
						relationTypes[currentLevel] == RELATION_ON ? m_evaluator.evaluateOnness(
								supportObject, trajector)
								: m_evaluator.evaluateInness(supportObject, trajector);
		if (value > 0.5) {
			pointsFound++;

			if (currentLevel == objects.size() - 1) {
				// This is the first level
				outPoints.push_back(vector<Pose3> (objects.size()));
			}

			if (currentLevel == 0) {
				// This is the top-level trajector itself
				outPoints.back()[currentLevel] = trajector->pose;
			} else {
				// Sample and recurse
				outPoints.back()[currentLevel] = trajector->pose;
				sampleRecursively(relationTypes, objects, nSamplesPerStep, nMaxSamples,
						outPoints, trajector, currentLevel - 1);
			}
		}
		//    if (iterations % 100 == 0) {
		//      log("iterations: %i, points: %i", iterations, pointsFound);
		//    }
	}
	trajector->pose = oldPose;
}
