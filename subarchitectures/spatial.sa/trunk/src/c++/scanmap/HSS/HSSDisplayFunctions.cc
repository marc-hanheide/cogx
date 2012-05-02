//
// = FILENAME
//    HSSDisplayFunctions.cc
//
// = FUNCTION
//
// = AUTHOR(S)
//    Patric Jensfelt
//
// = COPYRIGHT
//    Copyright (c) 2010 Patric Jensfelt
//
/*----------------------------------------------------------------------*/

#include "HSSDisplayFunctions.hh"

namespace HSS {

void displayScan(peekabot::PointCloudProxy scanPts,
                 HSS::Scan2D &scan, 
                 const Eigen::VectorXd &X, const Eigen::Vector3d &xsR,
                 float r, float g, float b)
{
  Eigen::VectorXd xsW(HSS::compound(X,xsR));
  peekabot::VertexSet vs;

  for (unsigned int i = 0; i < scan.range.size(); i++) {
    if (!scan.valid[i]) continue;
    vs.add_vertex(xsW[0]+scan.range[i]*cos(xsW[2]+scan.theta[i]),
                  xsW[1]+scan.range[i]*sin(xsW[2]+scan.theta[i]),
                  -0.001);      
  }
  
  scanPts.set_vertices(vs);
  scanPts.set_color(r,g,b);
}

void displayUncertainty(peekabot::GroupProxy &root,
                        const Eigen::Vector3d &X, const Eigen::Matrix3d &P,
                        double nsigma, float r, float g, float b)
{
  peekabot::LineCloudProxy ell;
  ell.add(root, "ellipse");
  {
    // Ellipse drawing code is taken from Kai Arras SLAM Toolbox
    double sxx = P(0,0);
    double syy = P(1,1);
    double sxy = P(0,1);
    // Major and minor axes
    double a = sqrt(0.5*(sxx+syy+sqrt(HSS::sqr(sxx-syy)+4*HSS::sqr(sxy)))); // always greater
    double b = sqrt(0.5*(sxx+syy-sqrt(HSS::sqr(sxx-syy)+4*HSS::sqr(sxy)))); // always smaller
    
    // Scaling in order to reflect specified probability
    //a *= sqrt(chi2invtable(alpha,2));
    //b *= sqrt(chi2invtable(alpha,2));
    
    a *= nsigma;
    b *= nsigma;

    // Look where the greater half axis belongs to
    if (sxx < syy) {
      double swap = a; 
      a = b; 
      b = swap; 
    }
    
    // Calculate inclination (numerically stable)
    double angle = 0;
    if (sxx != syy) {
      angle = 0.5*atan(2*sxy/(sxx-syy));        
    } else if (sxy == 0) {
      angle = 0;     // angle doesn't matter 
    } else if (sxy > 0) {
      angle =  M_PI_4;
    } else if (sxy < 0) {
      angle = -M_PI_4;
    }
    
    int N = 50;
    Eigen::MatrixXd xy(2,N);
    int i = 0;
    for (double ang = 0; ang < 2.0*M_PI; ang += 2.0*M_PI/N) {
      xy(0,i) = a*cos(ang);
      xy(1,i) = b*sin(ang);
      i++;
    }
    
    Eigen::MatrixXd R(2,2);
    R(0,0) = R(1,1) = cos(angle);
    R(0,1) = -sin(angle);
    R(1,0) = -R(0,1);
    xy = R*xy;
 
    peekabot::VertexSet vs;
    for (int i = 0; i < N; i++) {
      vs.add_vertex(X[0]+xy(0,i), X[1]+xy(1,i), 0);
      vs.add_vertex(X[0]+xy(0,(i+1)%N), X[1]+xy(1,(i+1)%N), 0);
    }
    ell.set_vertices(vs);
  }
  ell.set_color(r,g,b);

  peekabot::LineCloudProxy wedge;
  wedge.add(root, "wedge");
  {
    peekabot::VertexSet vs;
    double unc = sqrt(P(2,2));
    double llen = 0.5;
    vs.add_vertex(X[0], X[1], 0);
    vs.add_vertex(X[0] + llen * cos(X[2]),
                  X[1] + llen * sin(X[2]),
                  0);
    vs.add_vertex(X[0], X[1], 0);
    vs.add_vertex(X[0] + llen * cos(X[2] + nsigma*unc),
                  X[1] + llen * sin(X[2] + nsigma*unc),
                  0);
    wedge.set_vertices(vs);
  }
  wedge.set_color(r,g,b);
}

void addDoorPost(peekabot::GroupProxy &door, double width, 
                 float r, float g, float b)
{             
  peekabot::CubeProxy cpR;
  cpR.add(door, "doorpostright");
  cpR.set_scale(0.1, 0.1, 2.0);
  cpR.set_position(-0.5*width, 0, 1.0);
  cpR.set_color(r,g,b);
  
  peekabot::CubeProxy cpL;
  cpL.add(door, "doorpostleft");
  cpL.set_scale(0.1, 0.1, 2.0);
  cpL.set_position(+0.5*width, 0, 1.0);
  cpL.set_color(r,g,b);

  peekabot::CubeProxy cpT;
  cpT.add(door, "doorposttop");
  cpT.set_scale(width+0.1, 0.1, 0.1);
  cpT.set_position(0, 0, 2.0);
  cpT.set_color(r,g,b);
}

void displayDoorMeas(peekabot::GroupProxy &root,
                     const Eigen::VectorXd &X, const Eigen::Vector3d &xsR,
                     HSS::DoorExtractor &doorExtractor)
{
  peekabot::GroupProxy doormeas;
  doormeas.add(root, "doormeas", peekabot::AUTO_ENUMERATE_ON_CONFLICT);
  
  Eigen::Vector3d xsW(HSS::compound(X, xsR));

  for (std::list<HSS::RedundantLine2DRep>::iterator i = 
         doorExtractor.doors().begin();
       i != doorExtractor.doors().end(); i++) {
    
    // Pose of the door in the sensor frame
    Eigen::Vector3d xdS;
    xdS[0] = i->xC();
    xdS[1] = i->yC();
    xdS[2] = i->theta();

    // Calculate the pose of the door in the world frame
    Eigen::Vector3d xdW(HSS::compound(xsW, xdS));
    double width = 2.0*i->h();

    peekabot::GroupProxy door;
    door.add(doormeas, "door");
    door.set_pose(xdW[0], xdW[1], 0 , xdW[2], 0, 0);
    float col[] = {1.0, 0.817, 0.269};
    HSS::addDoorPost(door, width, col[0],col[1],col[2]);
  }
}

void displayStateWithUnc(peekabot::GroupProxy &refroot,
                         const Eigen::VectorXd &X, const Eigen::MatrixXd &P,
                         int statepos, HSS::Scan2D *scan, 
                         double nsigma,
                         float r, float g, float b)
{
  Eigen::Vector3d xW;
  xW[0] = X[statepos+0];
  xW[1] = X[statepos+1];
  xW[2] = X[statepos+2];
  
  // Display the coordinate system
  peekabot::LineCloudProxy coord;
  coord.add(refroot, "coord");
  {
    peekabot::VertexSet vsax;
    double axlen = 0.5;
    vsax.add_vertex(xW[0]+axlen*cos(xW[2]+M_PI_2),
                    xW[1]+axlen*sin(xW[2]+M_PI_2),
                    0);
    vsax.add_vertex(xW[0],
                    xW[1],
                    0);
    vsax.add_vertex(xW[0],
                    xW[1],
                    0);
    vsax.add_vertex(xW[0]+axlen*cos(xW[2]),
                    xW[1]+axlen*sin(xW[2]),
                    0);
    coord.set_vertices(vsax);
  }
  coord.set_line_width(2);
  coord.set_color(r,g,b);
  
  {
    Eigen::Matrix3d Pxya;
    Pxya.setZero();
    Pxya(0,0) = P(statepos,statepos);
    Pxya(0,1) = P(statepos,statepos+1);
    Pxya(0,2) = P(statepos,statepos+2);
    Pxya(1,0) = P(statepos+1,statepos);
    Pxya(1,1) = P(statepos+1,statepos+1);
    Pxya(1,2) = P(statepos+1,statepos+2);
    Pxya(2,0) = P(statepos+2,statepos);
    Pxya(2,1) = P(statepos+2,statepos+1);
    Pxya(2,2) = P(statepos+2,statepos+2);
    
    HSS::displayUncertainty(refroot, xW, Pxya, nsigma, r,g,b);
  }

  if (scan) {
    // Display the scan points in the reference scans
    peekabot::PointCloudProxy scanPts;
    scanPts.add(refroot, "scan", peekabot::REPLACE_ON_CONFLICT);
    peekabot::VertexSet vs;
    for (unsigned int j = 0; j < scan->range.size(); j++) {
      if (!scan->valid[j]) continue;
      vs.add_vertex(xW[0]+scan->range[j]*cos(xW[2]+scan->theta[j]),
                    xW[1]+scan->range[j]*sin(xW[2]+scan->theta[j]),
                    0);      
    }
    scanPts.set_vertices(vs);
    scanPts.set_color(r,g,b);
  }
}

void
displayPolygon(peekabot::ObjectProxyBase &root,
               Eigen::Vector3d pose,
               Eigen::MatrixXd polygon,
               float r, float g, float b)
{
  if (polygon.cols() == 0) {
    std::cerr << "No polygon to display\n";
    return;
  }

  peekabot::LineCloudProxy pp;
  pp.add(root, "polygon", peekabot::REPLACE_ON_CONFLICT);

  peekabot::VertexSet vs;
  
  Eigen::MatrixXd R(2,2);
  R(0,0) = cos(pose[2]);
  R(0,1) =-sin(pose[2]);
  R(1,1) = R(0,0);
  R(1,0) =-R(0,1);

  Eigen::MatrixXd xy(R * polygon);
  
  for (int i = 0; i < polygon.cols(); i++) {
    vs.add_vertex(pose[0] + xy(0,i),
                  pose[1] + xy(1,i),
                  0);
    vs.add_vertex(pose[0] + xy(0,(i+1)%polygon.cols()),
                  pose[1] + xy(1,(i+1)%polygon.cols()),
                  0);
  }
  
  pp.set_vertices(vs);
  pp.set_color(r,g,b);
}

}; // namespace HSS
