//
// = FILENAME
//    NavGraphGateway.cc
//
// = AUTHOR(S)
//    Patric Jensfelt
//
// = DESCRIPTION
//    
// = COPYRIGHT
//    Copyright (c) 2005 Patric Jensfelt
//
/*----------------------------------------------------------------------*/

#include "Navigation/NavGraphGateway.hh"
#include "Utils/CureDebug.hh"

#ifndef DEPEND
#include <sstream>
#endif

using namespace Cure;

NavGraphGateway::NavGraphGateway()
  :NavGraphNode(),
   m_Width(1.0),
   m_DoorState(UNKNOWN)
{
  std::cerr << "Constructed " << *this << std::endl;
  m_Type = NODETYPE_GATEWAY;
}

NavGraphGateway::NavGraphGateway(const std::string &name, int id, 
                                 double x, double y, double a, double maxSpeed,
                                 double width)
  :NavGraphNode(NODETYPE_GATEWAY,name,id,x,y,a,maxSpeed),
   m_Width(width),
   m_DoorState(UNKNOWN)
{}

NavGraphGateway::NavGraphGateway(const NavGraphGateway &src)
  :NavGraphNode(src),
   m_Width(src.m_Width),
   m_DoorState(src.m_DoorState)
{}

NavGraphGateway::~NavGraphGateway()
{}

NavGraphNode* 
NavGraphGateway::getNodeCopy() 
{ 
  return new NavGraphGateway(*this); 
}
