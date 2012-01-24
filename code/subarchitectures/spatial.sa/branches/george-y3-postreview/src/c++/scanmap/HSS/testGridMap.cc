//
// = LIBRARY
//
// = FILENAME
//    testGridMap.cc
//
// = FUNCTION
//
// = AUTHOR(S)
//    Patric Jensfelt
//
// = DESCRIPTION
//    
// = COPYRIGHT
//    Copyright (c) 2010 Patric Jensfelt
//
/*----------------------------------------------------------------------*/

#ifndef DEPEND
#include <Eigen/Core>
#include <cstdio>
#endif

#include <peekabot.hh>
#include <peekabot/Types.hh>

#include "HSSutils.hh"
#include "HSSScan2D.hh"
#include "HSSVirtualScan2DFactory.hh"
#include "HSSGridMap2D.hh"
#include "HSSGridMapFunctor.hh"
#include "HSSGridLineRayTracer.hh"
#include "HSSDisplayFunctions.hh"

int main(int argc, char * argv[])
{
  const char *optstring = "h";
  const char *args = "[-h help]";
  std::string pbHost = "localhost";
  char o = getopt(argc, argv, optstring);
  while (o != -1) {
    switch (o) {
    case 'h':
    case '?':
      std::cerr << "Usage: " << argv[0] << " " << args << std::endl;
      return -1;
    }
    o = getopt(argc, argv, optstring);
  }

  peekabot::PeekabotClient peekabotClient;
  peekabot::GroupProxy peekabotRoot;
  
  int pbPort = 5050;

  try {
    printf("Trying to connect to Peekabot on host %s and port %d\n",
           pbHost.c_str(), pbPort);

    peekabotClient.connect(pbHost, pbPort);
    peekabotRoot.add(peekabotClient, "HSS", peekabot::REPLACE_ON_CONFLICT);    

  } catch(std::exception &e) {
    fprintf(stderr, "Caught exception when connecting to peekabot (%s)\n",
            e.what());
    return -1;
  }

  HSS::CharGridMapFunctor gridfunc;
  HSS::GridMap2D<char> grid(gridfunc, 50, 0.1, 0,0);
  HSS::GridLineRayTracer<char> raytracer(grid, gridfunc);

  HSS::Scan2D scan;
  scan.alloc(681);
  scan.min_theta = -HSS::deg2rad(120);
  scan.max_theta = HSS::deg2rad(120);
  scan.min_range = 0.1;  
  scan.max_range = 5.55;
  scan.range_sigma = 0.05;
  double da = (scan.max_theta - scan.min_theta) / (scan.range.size() - 1);
  for (unsigned int i = 0; i < scan.range.size(); i++) {
    scan.range[i] = 3;
    scan.theta[i] = scan.min_theta + da * i;
    scan.valid[i] = 1;
  }
  
  Eigen::Vector3d xs;
  xs[0] = 0; xs[1] = 0; xs[2] = 0;

  raytracer.addScan2D(scan, xs, scan.max_range);
  HSS::displayGrid<char>(peekabotRoot, grid, gridfunc);
  getchar();

  grid.moveCenterTo(-3,0);
  HSS::displayGrid<char>(peekabotRoot, grid, gridfunc);
  getchar();

  grid.moveCenterTo(0,0);
  HSS::displayGrid<char>(peekabotRoot, grid, gridfunc);
  getchar();
}
