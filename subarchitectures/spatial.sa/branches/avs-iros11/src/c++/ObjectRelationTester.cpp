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

ObjectRelationTester::ObjectRelationTester() : m_sampler(&m_evaluator)
{
}

ObjectRelationTester::~ObjectRelationTester() 
{ 
}

void ObjectRelationTester::configure(const map<string,string>& _config) 
{
  m_bTestOnness = false;
  m_bSampleOnness = false;

  map<string,string>::const_iterator it = _config.find("--test-onness");
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
}

void ObjectRelationTester::start() 
{
  if (m_bTestOnness || m_bTestInness) {
    while(!m_PeekabotClient.is_connected() && (m_RetryDelay > -1)){
      connectPeekabot();
      sleep(m_RetryDelay);
    }

    if (m_bTestOnness) {
      m_relationTester.add(m_PeekabotClient, "on-ness_tester", peekabot::REPLACE_ON_CONFLICT);
    }
    else if (m_bTestInness) {
      m_relationTester.add(m_PeekabotClient, "in-ness_tester", peekabot::REPLACE_ON_CONFLICT);
    }
    println("Connected to peekabot, ready to go");
  }
}

void ObjectRelationTester::runComponent() 
{
  log("I am running!");

  peekabot::SphereProxy sqdp;
  peekabot::SphereProxy scwp;
  peekabot::SphereProxy bcwp;
  peekabot::SphereProxy op;
  peekabot::CubeProxy csp;
  peekabot::CubeProxy cop;
  peekabot::PolygonProxy pp;
  peekabot::CubeProxy bp;
  peekabot::CubeProxy bp2;
  PlaneObject table1;

  peekabot::SphereProxy sp;
  peekabot::SphereProxy spm;
  peekabot::SphereProxy sp2;
  peekabot::SphereProxy spm2;

  if (m_bTestOnness || m_bTestInness) {
    table1.type = OBJECT_PLANE;

    Matrix33 rotation;
    double rotAngle = 0.0;
    fromAngleAxis(rotation, rotAngle, vector3(0.0, 0.0, 1.0));
    table1.pose = pose3(vector3(0.0, 0.0, 1.0), rotation);

    table1.shape = PLANE_OBJECT_RECTANGLE;
    table1.radius1 = 0.5;
    table1.radius2 = 0.5;

    peekabot::VertexSet polyVerts;

    pp.add(m_relationTester, "table", peekabot::REPLACE_ON_CONFLICT);
    pp.set_color(1.0, 1.0, 0);
    polyVerts.add(table1.radius1, table1.radius2, 0);
    polyVerts.add(-table1.radius1, table1.radius2, 0);
    polyVerts.add(-table1.radius1, -table1.radius2, 0);
    polyVerts.add(table1.radius1, -table1.radius2, 0);

    pp.add_vertices(polyVerts);
    pp.translate(table1.pose.pos.x, table1.pose.pos.y, table1.pose.pos.z);
    pp.rotate(rotAngle, 0.0, 0.0, 1.0);


    bp.add(m_relationTester, "krispies", peekabot::REPLACE_ON_CONFLICT);
    bp.translate(table1.pose.pos.x, table1.pose.pos.y, table1.pose.pos.z + 0.26+0.145);
    bp.set_scale(0.19, 0.09, 0.29);
    bp.set_color(0.0, 0.0, 1.0);
    bp.set_opacity(0.5);

    bp2.add(m_relationTester, "joystick", peekabot::REPLACE_ON_CONFLICT);
    bp2.translate(table1.pose.pos.x, table1.pose.pos.y, table1.pose.pos.z + 0.13);
    bp2.set_scale(0.23, 0.21, 0.26);
    bp2.set_color(1.0, 0.0, 0.0);
    bp2.set_opacity(0.5);

    if (m_bTestOnness) {
      peekabot::GroupProxy sliders;
      sliders.add(m_relationTester, "weights", peekabot::REPLACE_ON_CONFLICT);

      sqdp.add(sliders, "squareDistanceOutside", peekabot::REPLACE_ON_CONFLICT);
      sqdp.translate(-1.0, 6.0, 10*m_evaluator.distanceFalloffOutside);
      sqdp.set_scale(0.1);
      scwp.add(sliders, "squareDistanceInside", peekabot::REPLACE_ON_CONFLICT);
      scwp.translate(0.0, 6.0, 10*m_evaluator.distanceFalloffInside);
      scwp.set_scale(0.1);
      bcwp.add(sliders, "patchThreshold", peekabot::REPLACE_ON_CONFLICT);
      bcwp.translate(1.0, 6.0, 10*m_evaluator.patchThreshold);
      bcwp.set_scale(0.1);
      csp.add(sliders, "containmentSteepness", peekabot::REPLACE_ON_CONFLICT);
      csp.translate(0.0, 6.0, 10*m_evaluator.supportCOMContainmentSteepness);
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

  while (isRunning()) {
    if (m_bTestOnness || m_bTestInness) {
    
      
      peekabot::Result<peekabot::Transformation> r;

      r = bp.get_transformation(peekabot::WORLD_COORDINATES);
      if (r.succeeded()) {
	Pose3 boxPose;
	setIdentity(boxPose);
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

	setRow44(boxPose, m);

	BoxObject box1;

	box1.type = OBJECT_BOX;
	box1.pose = boxPose;
	box1.radius1 = 0.095;
	box1.radius2 = 0.045;
	box1.radius3 = 0.145;

	r = bp2.get_transformation(peekabot::WORLD_COORDINATES);
	if (r.succeeded()) {
	  Pose3 boxPose;
	  setIdentity(boxPose);
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

	  
	 /* m[0] = r.get_result()(0,0);
	  m[1] = r.get_result()(0,1);
	  m[2] = r.get_result()(0,2);
	  m[3] = r.get_result()(0,3);
	  m[4] = r.get_result()(1,0);
	  m[5] = r.get_result()(1,1);
	  m[6] = r.get_result()(1,2);
	  m[7] = r.get_result()(1,3);
	  m[8] = r.get_result()(2,0);
	  m[9] = r.get_result()(2,1);
	  m[10] = r.get_result()(2,2);
	  m[11] = r.get_result()(2,3);
	  m[12] = r.get_result()(3,0);
	  m[13] = r.get_result()(3,1);
	  m[14] = r.get_result()(3,2);
	  m[15] = r.get_result()(3,3);*/

	  setRow44(boxPose, m);

	  HollowBoxObject box2;

//	  box2.type = OBJECT_HOLLOW_BOX;
	  box2.type = OBJECT_BOX;
	  box2.pose = boxPose;
	  box2.radius1 = 0.115;
	  box2.radius2 = 0.105;
	  box2.radius3 = 0.13;
//	  box2.thickness = 0.02;


	  if (m_bTestOnness) {
	    peekabot::Result<peekabot::Transformation> vr;
	    vr = sqdp.get_transformation();
//	    if (vr.succeeded()) distanceFalloffOutside= 0.1*vr.get_result().pos.z;
	    vr = scwp.get_transformation();
//	    if (vr.succeeded()) distanceFalloffInside = 0.1*vr.get_result().pos.z;
	    vr = bcwp.get_transformation();
//	    if (vr.succeeded()) patchThreshold = 0.1*vr.get_result().pos.z;
	    vr = csp.get_transformation();
	    if (vr.succeeded()) {
	      //	bottomCOMContainmentSteepness = 
//	      supportCOMContainmentSteepness = 0.1*vr.get_result().pos.z;
	    }
	    vr = cop.get_transformation();
	    if (vr.succeeded()) {
	      //	bottomCOMContainmentOffset = 
//	      supportCOMContainmentOffset = vr.get_result().pos.z;
	    }

	    double scale1 = m_evaluator.evaluateOnness(&table1, &box1);
	    double scale2 = m_evaluator.evaluateOnness(&box2, &box1);
	    Witness w1 = m_evaluator.m_lastWitness;
	    m_evaluator.evaluateOnness(&box1, &box2);
	    Witness w2 = m_evaluator.m_lastWitness;
	    if (w1.distance - w2.distance > 0.0001 ||
		w1.distance - w2.distance < -0.0001) {
	      log("Error!");
	    }
	    log("%f, %f", scale1, scale2);
	    sp.set_scale(scale1);
	    sp2.set_scale(scale2);

	    vector<Vector3> patch;
	    Witness witness = m_evaluator.findContactPatch(box2, box1, &patch);
	    if (patch.size() > 2) {
	      peekabot::PolygonProxy patchp;
	      peekabot::VertexSet polyVerts;
	      patchp.add(m_relationTester, "Patch", peekabot::REPLACE_ON_CONFLICT);
	      patchp.set_color(1,0,0);
	      for (vector<Vector3>::iterator it = patch.begin(); it != patch.end();it++){
		polyVerts.add(it->x, it->y, it->z);
	      }
	      patchp.add_vertices(polyVerts);


	      Vector3 avs = m_evaluator.computeAttentionVectorSumForSolid(&box2, witness.point1, box1.pose.pos,
		  0.5);
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
	    witp1.add(m_relationTester, "Witness 1", peekabot::REPLACE_ON_CONFLICT);
	    witp1.translate(witness.point1.x, witness.point1.y, witness.point1.z);
	    witp1.set_scale(0.01);
	    peekabot::SphereProxy witp2;
	    witp2.add(m_relationTester, "Witness 2", peekabot::REPLACE_ON_CONFLICT);
	    witp2.translate(witness.point2.x, witness.point2.y, witness.point2.z);
	    witp2.set_scale(0.01);

	    if (m_bSampleOnness) {
	      static bool sampleTable = false;


//	      Cure::LocalGridMap<double> pdf(50, 0.05, 0.0, 
//		  Cure::LocalGridMap<double>::MAP1, 0, 0);
	      vector<spatial::Object *>objects;
	      vector<string> objectLabels;
	      vector<spatial::SpatialRelationType> relations;

	      if (sampleTable) {
		objects.push_back(&box2);
		objectLabels.push_back("box2");
//		objects.push_back(&box1);
//		objectLabels.push_back("box1");
//		relations.push_back(RELATION_ON);
		objects.push_back(&table1);
		objectLabels.push_back("table1");
		relations.push_back(RELATION_ON);
	      }
	      else {
		objects.push_back(&box1);
		objectLabels.push_back("box1");
//		objects.push_back(&box2);
//		objectLabels.push_back("box2");
//		relations.push_back(RELATION_ON);
		objects.push_back(&table1);
		objectLabels.push_back("table1");
		relations.push_back(RELATION_ON);
	      }

	      //	    objects.push_back(&box2);
	      //	    objectLabels.push_back("box2");
	      //	    relations.push_back(RELATION_ON);

//	      SampleCloud testCloud;
//	      m_sampler.
//		sampleBinaryRelationSystematically(relations, objects, 
//		    objectLabels, pdfMap.getCellSize(),
//		    testCloud);

	      //  log("Writing into 2D grid");
//	      testCloud.KernelDensityEstimation2D(pdf, 
//		  objects.back()->pose.pos, m_sampler.getKernelWidthFactor(), 
//		  total, 1.0);

//	      testCloud.compact();
//	      Vector3 center;
//	      double interval;
//	      int xExt, yExt, zExt;
//	      vector<double> weights;
//	      testCloud.makePointCloud(center, interval, xExt, yExt, zExt, weights);
//
//	      center += objects.back()->pose.pos;
//	      vector<Vector3>centers;
//	      centers.push_back(center);
//
//	      m_sampler.kernelDensityEstimation3D(pdfMap,
//		  centers, interval, xExt, yExt, zExt, weights, 1.0, 1.0);
//
//	      sampleTable = !sampleTable;
//	      
//	      //visualPB.DisplayMap(pdfMap);
//	      visualPB->AddPDF(pdfMap);

//	      peekabot::LineCloudProxy linecloudp;
//
//	      linecloudp.add(m_PeekabotClient, "root.distribution",
//		  peekabot::REPLACE_ON_CONFLICT);
//	      linecloudp.clear_vertices();
//	      linecloudp.set_color(0.5, 0, 0.5);
//
//	      double maxPDFValue = 0.0;
//	      for (int x = -pdf.getSize(); x <= pdf.getSize(); x++) {
//		for (int y = -pdf.getSize(); y <= pdf.getSize(); y++) {
//		  if (pdf(x,y) > maxPDFValue) {
//		    maxPDFValue = pdf(x,y);
//		  }
//		}
//	      }
//
//	      for (int x = -pdf.getSize(); x < pdf.getSize(); x++) {
//		for (int y = -pdf.getSize(); y <= pdf.getSize(); y++) {
//		  if (pdf(x, y) == 0)
//		    continue;
//		  double xW2, yW2;
//		  double xW3, yW3;
//		  pdf.index2WorldCoords(x, y, xW2, yW2);
//		  pdf.index2WorldCoords(x+1, y, xW3, yW3);
//		  linecloudp.add_line(xW2, yW2, pdf(x, y)/maxPDFValue,
//		      xW3, yW3, pdf(x+1, y)/maxPDFValue);
//		}
//	      }
//	      for (int x = -pdf.getSize(); x <= pdf.getSize(); x++) {
//		for (int y = -pdf.getSize(); y < pdf.getSize(); y++) {
//		  if (pdf(x, y) == 0)
//		    continue;
//		  double xW2, yW2;
//		  double xW3, yW3;
//		  pdf.index2WorldCoords(x, y, xW2, yW2);
//		  pdf.index2WorldCoords(x, y+1, xW3, yW3);
//		  linecloudp.add_line(xW2, yW2, pdf(x, y)/maxPDFValue,
//		      xW3, yW3, pdf(x, y+1)/maxPDFValue);
//		}
//	      }
//
	    }

	  } // if (m_bTestOnness)

	  if (m_bTestInness) {
	    sp.set_scale(m_evaluator.evaluateInness(&table1, &box1));
	    sp2.set_scale(m_evaluator.evaluateInness(&box2, &box1));

	    if (m_bSampleInness) {
	      Cure::LocalGridMap<double> pdf(25, 0.05, 0.0, 
		  Cure::LocalGridMap<double>::MAP1, 0, 0);
	      vector<spatial::Object *>objects;
	      vector<string>objectLabels;
	      objects.push_back(&box1);
	      objectLabels.push_back("box1");
	      objects.push_back(&box2);
	      objectLabels.push_back("box2");
	      //	    objects.push_back(&box2);
	      vector<spatial::SpatialRelationType> relations;
	      relations.push_back(RELATION_IN);
	      //	    relations.push_back(RELATION_IN);

	      double total;
	      m_sampler.
		sampleBinaryRelationRecursively(relations, objects, objects.size()-2, pdf,
		  total);
	      peekabot::LineCloudProxy linecloudp;

	      linecloudp.add(m_PeekabotClient, "Scene.distribution",
		  peekabot::REPLACE_ON_CONFLICT);
	      linecloudp.clear_vertices();
	      linecloudp.set_color(0.5, 0, 0.5);

	      double maxPDFValue = 0.0;
	      for (int x = -pdf.getSize(); x <= pdf.getSize(); x++) {
		for (int y = -pdf.getSize(); y <= pdf.getSize(); y++) {
		  if (pdf(x,y) > maxPDFValue) {
		    maxPDFValue = pdf(x,y);
		  }
		}
	      }

	      for (int x = -pdf.getSize(); x < pdf.getSize(); x++) {
		for (int y = -pdf.getSize(); y <= pdf.getSize(); y++) {
		  if (pdf(x, y) == 0)
		    continue;
		  double xW2, yW2;
		  double xW3, yW3;
		  pdf.index2WorldCoords(x, y, xW2, yW2);
		  pdf.index2WorldCoords(x+1, y, xW3, yW3);
		  peekabot::VertexSet pdfs;
		  pdfs.add(xW2, yW2, pdf(x, y)/maxPDFValue);
		  pdfs.add(xW3, yW3, pdf(x+1, y)/maxPDFValue);
		  linecloudp.add_vertices(pdfs);
		}
	      }
	      for (int x = -pdf.getSize(); x <= pdf.getSize(); x++) {
		for (int y = -pdf.getSize(); y < pdf.getSize(); y++) {
		  if (pdf(x, y) == 0)
		    continue;
		  double xW2, yW2;
		  double xW3, yW3;
		  pdf.index2WorldCoords(x, y, xW2, yW2);
		  pdf.index2WorldCoords(x, y+1, xW3, yW3);
		  peekabot::VertexSet pdfs;
		  pdfs.add(xW2, yW2, pdf(x, y)/maxPDFValue);
		  pdfs.add(xW3, yW3, pdf(x, y+1)/maxPDFValue);
		  linecloudp.add_vertices(pdfs);
		}
	      }
	    }
	  } // if (m_bTestInness)
	}
      }
    }

    sleepComponent(500);
  }
}

void 
ObjectRelationTester::connectPeekabot()
{
  try {
    log("Trying to connect to Peekabot (again) on host %s and port %d",
        m_PbHost.c_str(), m_PbPort);

    m_PeekabotClient.connect(m_PbHost, m_PbPort);

  } catch(std::exception &e) {
    log("Caught exception when connecting to peekabot (%s)",
        e.what());
    return;
  }
}
