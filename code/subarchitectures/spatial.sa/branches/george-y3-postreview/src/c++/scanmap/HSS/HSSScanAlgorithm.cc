//
// = FILENAME
//    HSSScanAlgorithm.cc
//
// = AUTHOR
//    Patric Jensfelt
//
// = COPYRIGHT
//    Copyright (c) 2009 Patric Jensfelt
//                  2010 Patric Jensfelt
//
/*----------------------------------------------------------------------*/

#include "HSSScanAlgorithm.hh"
#include "HSSutils.hh"

namespace HSS {

ScanAlgorithm::ScanAlgorithm()
{
  m_MaxRange = 30; // m
  configForHokuyoURG04LX();
}

ScanAlgorithm::~ScanAlgorithm()
{}

void
ScanAlgorithm::configForHokuyoURG04LX()
{
  m_SigmaR = 0.035;
  m_SigmaA = HSS::deg2rad(0.25);
}

void
ScanAlgorithm:: configForPLS200()
{
  m_SigmaR = 0.035; // m   It is 0.025 or so, but the discretization is 0.050
  m_SigmaA = HSS::deg2rad(0.25);
}

void 
ScanAlgorithm::configForLMS200()
{
  m_SigmaR = 0.007; // m
  m_SigmaA = HSS::deg2rad(0.25);
}

void 
ScanAlgorithm::configForLMS291()
{
  m_SigmaR = 0.007; // m
  m_SigmaA = HSS::deg2rad(0.38);
}

};
