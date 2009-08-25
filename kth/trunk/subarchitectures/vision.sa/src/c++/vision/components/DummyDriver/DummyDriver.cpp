/**
 * @author Michael Zillich
 * @date February 2009
 */

#include <cast/architecture/ChangeFilterFactory.hpp>
#include "DummyDriver.h"
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/vector.hpp>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>

using namespace boost;

/**
 * The function called to create a new instance of our component.
 */
extern "C"
{
  cast::CASTComponentPtr newComponent()
  {
    return new cast::DummyDriver();
  }
}

namespace cast
{

using namespace std;

void DummyDriver::configure(const map<string,string> & _config)
{
  map<string,string>::const_iterator it;

  if((it = _config.find("--labels")) != _config.end())
  {
    istringstream istr(it->second);
    string label;
    while(istr >> label)
      labels.push_back(label);

    ostringstream ostr;
    for(size_t i = 0; i < labels.size(); i++)
      ostr << " '" << labels[i] << "'";
    log("detecting objects: %s", ostr.str().c_str());
  }
  
  robotx = 0;
  roboty = -1;
  runonce = true;
  count = 0;
  for (int t = 0 ; t < 360; t = t + 1){
     	prior1.push_back(1);
     	prior2.push_back(1);
     }
}

void DummyDriver::start()
{
  // we want to receive detected objects
  addChangeFilter(createLocalTypeFilter<VisionData::VisualObject>(cdl::ADD),
      new MemberFunctionChangeReceiver<DummyDriver>(this,
        &DummyDriver::receiveVisualObject));
  // .. and when they change
  addChangeFilter(createLocalTypeFilter<VisionData::VisualObject>(cdl::OVERWRITE),
      new MemberFunctionChangeReceiver<DummyDriver>(this,
        &DummyDriver::receiveVisualObject));
}

void DummyDriver::runComponent()
{
  sleepProcess(1000);  // HACK: the nav visualisation might crash if we send it
                       // object observations too soon.
  // and initiate detection
  if (true){
	  VisionData::DetectionCommandPtr cmd = new VisionData::DetectionCommand;
  cmd->labels = labels;
  addToWorkingMemory(newDataID(), cmd);
  runonce = false;
	  
  }
  
}





void DummyDriver::receiveVisualObject(const cdl::WorkingMemoryChange & _wmc)
{
  VisionData::VisualObjectPtr obj =
    getMemoryEntry<VisionData::VisualObject>(_wmc.address);
  if(obj->detectionConfidence >= 0.5){
    log("ok, detected '%s' at count: %d", obj->label.c_str(), count);
  }
  else{
  	log("no object");
  	 VisionData::DetectionCommandPtr cmd = new VisionData::DetectionCommand;
  cmd->labels.push_back(obj->label);
  addToWorkingMemory(newDataID(), cmd);
  	return;
}
    
    
    std::vector<double> like1;
    std::vector<double> like2;
    std::vector<double> like12;
    std::vector<double> like21;
     numeric::ublas::vector<double> q1(2);
    numeric::ublas::vector<double> q2(2);
    boost::numeric::ublas::vector<double> robot (2);
    robot(0) = robotx;
    robot(1) = roboty;
    int anglestep = 1;
     for (int t = 0 ; t < 360; t = t + anglestep){
     	like1.push_back(1);
     	like2.push_back(1);
     	like12.push_back(1);
     	like21.push_back(1);
     	
     }
  
     q1(0) = 0;
    q1(1) = -1;
     q2(0) = 0;
    q2(1) = 1;
    
    numeric::ublas::matrix<double> cov1 (2, 2);
    numeric::ublas::matrix<double> invertedcov1 (2, 2);
    numeric::ublas::matrix<double> cov2 (2, 2);
    numeric::ublas::matrix<double> cov12 (2, 2);
    numeric::ublas::matrix<double> cov21 (2, 2);
    
    
    cov1(0,0) = 0.1; cov1(0,1) = 0;
    cov1(1,0) = 0; cov1(1,1) = 0.1;
    
    invertedcov1(0,0) = 10; invertedcov1(0,1) = 0;
    invertedcov1(1,0) = 0; invertedcov1(1,1) = 10;
    
    //invertedcov =  numeric::ublas::InvertMatrix(cov1, invertedcov);
    numeric::ublas::matrix<double> rotation (2, 2);
    double det = 0.01;
    double constant1 = 1 / ( sqrt(det)* 2*M_PI );
    double constant2 = 1 / ( sqrt(det)* 2*M_PI );
    double constant12 = 1 / ( sqrt(det)* 2*M_PI );
    double constant21 = 1 / ( sqrt(det)* 2*M_PI );
    
    numeric::ublas::matrix<double> invertedcov;
    
    int highest = 0;
    int highi = -1;
    int n = 0;
    if(obj->label == "ricek_front" || obj->label == "ricek_back"){
    	if(obj->label == "ricek_back"){
    	robot(0) = robotx;
    	robot(1) = 1;
    	}
    	
    log("first loop");
    for (double t = 0 ; t < 360; t = t + anglestep){
    	
    	double i = t*M_PI/180;
    	rotation(0,0) = cos(i); rotation(0,1) = -sin(i);
    	rotation(1,0) = sin(i); rotation(1,1) = cos(i);
    	
    	numeric::ublas::vector<double> sub1(2);
    	numeric::ublas::vector<double> q1rotation(2);
    	numeric::ublas::vector<double> sub2(2);
    	numeric::ublas::vector<double> q2rotation(2);
    	double temp1,temp2;
    	
    	q1rotation = prod(rotation,q1);
    	sub1 = robot - q1rotation;
    	numeric::ublas::vector<double> intermediate1(2);
    	intermediate1 = prod(trans(sub1),invertedcov1);
    	double res1 = inner_prod(intermediate1,sub1);
    	temp1 = 0.5*constant1 * exp(-0.5*res1);
    	
    	
    	q2rotation = prod(rotation,q2);
    	sub2 = robot - q2rotation;
    	numeric::ublas::vector<double> intermediate2(2);
    	intermediate2 = prod(trans(sub2),invertedcov1);
    	double res2 = inner_prod(intermediate2,sub2);
    	temp2 = 0.5*constant1 * exp(-0.5*res2);
    	double temp = temp1+temp2;
    	
    	like1[n] = temp;
    	like12[n] = temp1;  	
    	n++;
    	}
    }
   n = 0;
   if(obj->label == "ricek_front" || obj->label == "ricek1_back"){
   	if(obj->label == "ricek1_back"){
    	robot(0) = robotx;
    	robot(1) = -roboty;
    	}
   	
    log("second loop");
    for (double t = 0 ; t < 360; t = t + 1){
    	double i = t*M_PI/180;
    	rotation(0,0) = cos(i); rotation(0,1) = -sin(i);
    	rotation(1,0) = sin(i); rotation(1,1) = cos(i);
    	
    	numeric::ublas::vector<double> sub1(2);
    	numeric::ublas::vector<double> q1rotation(2);
    	numeric::ublas::vector<double> sub2(2);
    	numeric::ublas::vector<double> q2rotation(2);
    	double temp1,temp2;
    	
    	q1rotation = prod(rotation,q1);
    	sub1 = robot - q1rotation;
    	
    	numeric::ublas::vector<double> intermediate1(2);
    	intermediate1 = prod(trans(sub1),invertedcov1);
    	double res1 = inner_prod(intermediate1,sub1);
    	temp1 = 0.5*constant1 * exp(-0.5*res1);
    	
    	
    	q2rotation = prod(rotation,q2);
    	sub2 = robot - q2rotation;
    	numeric::ublas::vector<double> intermediate2(2);
    	intermediate2 = prod(trans(sub2),invertedcov1);
    	double res2 = inner_prod(intermediate2,sub2);
    	temp2 = 0.5*constant1 * exp(-0.5*res2);
    	
    	double temp = temp1+temp2;
    	
    	like2[n] = temp;
    	
    	like21[n] = temp1;
    	n++;
    	}
    }
    
  robotx = robotx-0.1;
  roboty = -1;
  sleep(1);
    
  char buffer1[32];
  char buffer2[32];
  count++;

  sprintf (buffer1, "temp1_%d",count );
  sprintf (buffer2, "temp2_%d",count );
  FILE *fp1;
  
  FILE *fp2;
	 fp1=fopen(buffer1, "w");
  	fp2=fopen(buffer2, "w");
  	
  	
  for (unsigned int i = 0; i < like1.size() ; i++)
  {
  	double post1 = like1[i]*like21[i]*prior1[i];
  	double post2 = like2[i]*like12[i]*prior2[i];
  	
  	prior1[i] = post1;
  	prior2[i] = post2;
  	
  	fprintf(fp1, "%f\n", post1);
  	fprintf(fp2, "%f\n", post2);
  	
  }
  fclose(fp1);
  fclose(fp2);
  
  
  VisionData::DetectionCommandPtr cmd = new VisionData::DetectionCommand;
  cmd->labels.push_back(obj->label);
  addToWorkingMemory(newDataID(), cmd);
}





}

