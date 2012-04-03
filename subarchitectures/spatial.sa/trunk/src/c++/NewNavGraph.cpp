//
// = FILENAME
//    NavGraph.cc
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

#include "NewNavGraph.hpp"
#include <Navigation/NavGraphNode.hh>
#include "NavGraphGateway.hpp"
#include <Navigation/NavGraphEdge.hh>
#include <Utils/CureDebug.hh>
#include <Utils/HelpFunctions.hh>

#ifndef DEPEND
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <fstream>
#endif

using namespace Cure;

//===========================================================================

NavGraphEventListener::NavGraphEventListener(const std::string &n)
{
  m_Name = n;
}

NavGraphEventListener::~NavGraphEventListener()
{}

//===========================================================================

NewNavGraph::NewNavGraph(double nodeDist)
  :m_NodeDist(nodeDist),
   m_MaxDist(5),
   m_CurrNode(0),
   m_NextNodeId(0),
   m_MaxSpeed(1),
   m_LastAreaId(-1),
   m_AssignedNewAreaId(false),
   m_AutoFixGateway(true)
{
  m_RLStarSizeNormal = 0.2;
  m_RLStarSizeCurrent = 0.4;
  m_RLStarSizeNamedGoal = 0.3;
  m_RLStarSizeGateway = 0.5;
  m_RLStarSizeGatewayBlocked = 0.5;
}

NewNavGraph::~NewNavGraph()
{
  clear();
}

void 
NewNavGraph::configRLStars(double sizeCurrent,
                        double sizeNormal,
                        double sizeNamedGoal,
                        double sizeGateway,
                        double sizeGatewayBlocked) 
{
  if (sizeCurrent > 0) m_RLStarSizeCurrent = sizeCurrent;
  if (sizeNormal > 0) m_RLStarSizeNormal = sizeNormal;
  if (sizeNamedGoal > 0) m_RLStarSizeNamedGoal = sizeNamedGoal;
  if (sizeGateway > 0) m_RLStarSizeGateway = sizeGateway;
  if (sizeGatewayBlocked > 0) m_RLStarSizeGatewayBlocked = sizeGatewayBlocked;
}

int
NewNavGraph::clear()
{
  for (std::list<NavGraphNode*>::iterator ni = m_Nodes.begin();
       ni != m_Nodes.end(); ni++) {
    delete *ni;
  }
  m_Nodes.clear();
  m_Edges.clear();
  m_Gateways.clear();
  m_NextNodeId = 0;

  m_CurrNode = 0;

  m_LastAreaId = -1;
  m_AssignedNewAreaId = false;

  return 0;
}

int
NewNavGraph::getCurrentNodeId() const
{
  if (m_CurrNode) return m_CurrNode->getId();
  else return -1;
}

int
NewNavGraph::setCurrentNodeId(int id)
{
  Cure::NavGraphNode *nPtr = getNode(id);
  if (nPtr) m_CurrNode = nPtr;

  return getCurrentNodeId();
}

int
NewNavGraph::getCurrentNodeType() const
{
  if (m_CurrNode) return m_CurrNode->getType();
  else return -1;
}

int
NewNavGraph::getCurrentNodeAreaClass() const
{
  if (m_CurrNode) return m_CurrNode->getAreaClass();
  else return -1;
}

int
NewNavGraph::setCurrentNodeAreaClass(int c)
{
  if (m_CurrNode) {
    m_CurrNode->setAreaClass(c);
    return 0;
  }

  return -1;
}

long
NewNavGraph::getCurrentAreaId() const
{
  if (m_CurrNode) return m_CurrNode->getAreaId();
  else return -1;
}

int
NewNavGraph::addFreePose(double x, double y, double a)
{
  maybeAddNewNodeAt(x, y, a, false, NavGraphNode::NODETYPE_NODE);
  return 0;
}

int
NewNavGraph::maybeAddNewNodeAt(double x, double y, double a)
{
  return maybeAddNewNodeAt(x, y, a, false, NavGraphNode::NODETYPE_NODE);
}

int 
NewNavGraph::maybeAddNewNodeAt(double x, double y, double a,
                          bool forceInNode, int type)
{
  // If we have no current pose yet we look for the closest node an
  // dpick that one if it is close enough
  if (m_CurrNode == 0) {    
    m_CurrNode = getClosestNode(x,y,0,0.5*m_NodeDist);
    if (m_CurrNode != 0) {
      notifyChangedCurrentNode(-1, m_CurrNode->getId());
      return 0;
    }
  }

  // If we still have no curret node we are so far away from all other
  // nodes that we should create a new node at this pose and not
  // connect it to anything else
  if (m_CurrNode == 0) {

    if (type == NavGraphNode::NODETYPE_GATEWAY) {
      NavGraphGateway *g = new NavGraphGateway("-", m_NextNodeId, 
                                               x, y, a, m_MaxSpeed,
                                               1.0);
      m_Nodes.push_back(g);
      m_Gateways.push_back(g);
    } else {
      m_Nodes.push_back(new NavGraphNode("-", m_NextNodeId, 
                                         x, y, a, m_MaxSpeed));
    }

    m_CurrNode = m_Nodes.back();
    notifyChangedCurrentNode(-1, m_CurrNode->getId());
    // This node does not seem to connected to anything so we have to
    // assume that it is a new area
    m_LastAreaId++;
    m_CurrNode->setAreaId(m_LastAreaId);
    notifyChangedArea(m_LastAreaId);
    CureCERR(40) << "Added free node " << *m_CurrNode << std::endl;
    m_AssignedNewAreaId = true;
    m_NextNodeId++;
    return 1;
  }
  
  // Calculate distance to the current node
  double d = hypot(y - m_CurrNode->getY(), x - m_CurrNode->getX());

  if (!forceInNode) {  
    // Check if there is a existing node close enough not to have to add
    // a new node
    NavGraphNode *nc = getClosestNode(x,y,a);
    double dc = 1e10;
    if (nc != 0) {
      dc = hypot(y - nc->getY(), x - nc->getX());
    }
    
    if (nc != m_CurrNode &&
        dc < d && dc <= m_NodeDist) {
      // Change curret node to the closest one
      double cost = hypot(m_CurrNode->getY() - nc->getY(),
                          m_CurrNode->getX() - nc->getX());

      connectNodes(m_CurrNode, nc, cost);

      // Check to see if we detect any anomalities with the graph. If
      // we get to a new node (nc) and this node and the one we are
      // coming (m_CurrNode) from are not gateways, they should have
      // the same area_id. If not we have a problem!
      if (m_CurrNode->getAreaId() != nc->getAreaId() &&
          m_CurrNode->getType() != NavGraphNode::NODETYPE_GATEWAY &&
          nc->getType() != NavGraphNode::NODETYPE_GATEWAY) {

        CureCERR(40) << "\n\nWe should fix area ids when we come to node "
                     << *nc << " from node " << *m_CurrNode << "\n\n\n";

        if (m_AutoFixGateway) {
          replaceGatewaysConnecting(nc->getAreaId(), m_CurrNode->getAreaId());
          mergeAreaIds(nc->getAreaId(), m_CurrNode->getAreaId());
        } else {
          // Report this to any listener of these types of events
          std::list<NavGraphGateway*>::iterator gi;
          gi = findConnectingGateway(m_CurrNode->getAreaId(), nc->getAreaId());
          if (gi != m_Gateways.end()) {
            std::list<NavGraphEventListener*>::iterator li;
            for (li = m_EventListeners.begin(); 
                 li != m_EventListeners.end(); li++) {
              (*li)->areaIdConflict(*m_CurrNode, **gi);
            }
          } else {
            // FIXME. We should ley someone know that we connected two
            // nodes that have different area id but no gateway to
            // remove. This could mean that the gateway could not be
            // detected because it is too wide for example. Like the
            // transition between corridor and a hallway. Note that we
            // do not know that this event occurs at the intersection
            // between the two areas so we cannot add a door at this
            // point necessarily
            mergeAreaIds(nc->getAreaId(), m_CurrNode->getAreaId());
            CureCERR(20) << "\n\nCould not find gateway. "
                         << "We should let someone know!!!\n\n\n";
          }
        }
      }

      if (m_CurrNode->getAreaId() != nc->getAreaId()) {
        notifyChangedArea(nc->getAreaId());
      }
      notifyChangedCurrentNode(m_CurrNode->getId(), nc->getId());
      m_CurrNode = nc;

      if (m_AutoFixGateway) return 2;
      else return 0;

    } else if (d <= m_NodeDist) {
      // Nothing to do, we are still close enough to the current node to
      // not drop a new one.
      return 0;
    }
    
    if (d > 2.0 * m_NodeDist) {
      CureCERR(20) << "WARNING: You must call maybeAddNewNodeAt more often. "
                   << "Nodedist is now " << d << std::endl;
    }
  }

  // Create the new node
  if (type == NavGraphNode::NODETYPE_GATEWAY) {
    NavGraphGateway *g = new NavGraphGateway("-", m_NextNodeId, 
                                             x, y, a, m_MaxSpeed,
                                             1.0);
    m_Nodes.push_back(g);
    m_Gateways.push_back(g);
  } else {
    m_Nodes.push_back(new NavGraphNode("-", m_NextNodeId, 
                                       x, y, a, m_MaxSpeed));
  }

  NavGraphNode *n = m_Nodes.back();
  if (m_CurrNode->getType() == NavGraphNode::NODETYPE_GATEWAY) {
    // we are leving a gateway and entering what seems to be a new area
    m_LastAreaId++;
    n->setAreaId(m_LastAreaId);
  } else {
    n->setAreaId(m_CurrNode->getAreaId());
  }
  CureCERR(40) << "Added new node " << *n << " when leaving "
               << *m_CurrNode << std::endl;
  m_AssignedNewAreaId = true;
  m_NextNodeId++;

  // Conect the nodes
  connectNodes(m_CurrNode, n, d);
  
  // Make the new node the current node
  if (m_CurrNode->getAreaId() != n->getAreaId()) {
    notifyChangedArea(n->getAreaId());
  }
  notifyChangedCurrentNode(m_CurrNode->getId(), n->getId());
  m_CurrNode = n;

  std::list<NavGraphEventListener*>::iterator li;
  for (li = m_EventListeners.begin(); 
       li != m_EventListeners.end(); li++) {
    (*li)->newNode(*n);
  }

  return 1;
}

int
NewNavGraph::findPath(NavGraphNode *from, NavGraphNode *to,
                   std::list<NavGraphNode> &path, double *cost)
{
  if (from == 0) {
    CureCERR(20) << "Cannot findPath when from=0\n";
    return -2;
  }
  if (to == 0) {
    CureCERR(20) << "Cannot findPath when to=0\n";
    return -2;
  }

  double c = -1;
  if (!from->findPath(to, c)) {
    CureCERR(30) << "Found no path from "
                 << *from << " to " << *to << std::endl;
    return -1;
  }

  Cure::NavGraphNode *n = to;
  path.clear();
  while (n) {
    path.push_front(*n);
    n = (Cure::NavGraphNode*)n->parent;
  }

  if (cost) *cost = c;
  return 0;
}

int
NewNavGraph::findPath(NavGraphNode *from, int toNodeId,
                   std::list<NavGraphNode> &path, double *cost)
{
  NavGraphNode *to = getNode(toNodeId);
  if (to == 0) {
    CureCERR(20) << "findPath(" << from->getId() << ","
                 << toNodeId << ") Could not find to node\n";
    return -2;
  }

  return findPath(from, to, path, cost);
}
                   
int
NewNavGraph::findPath(int fromNodeId, NavGraphNode *to,
                   std::list<NavGraphNode> &path, double *cost)
{
  NavGraphNode *from = getNode(fromNodeId);
  if (from == 0) {
    CureCERR(20) << "findPath(" << fromNodeId << ","
                 << to->getId() << ") Could not find from node\n";
    return -2;
  }

  return findPath(from, to, path, cost);
}

int
NewNavGraph::findPath(int fromNodeId, int toNodeId,
                   std::list<NavGraphNode> &path, double *cost)
{
  NavGraphNode *from = getNode(fromNodeId);
  if (from == 0) {
    CureCERR(20) << "findPath(" << fromNodeId << ","
                 << toNodeId << ") Could not find from node\n";
    return -2;
  }

  return findPath(from, toNodeId, path, cost);
}

int
NewNavGraph::findPath(NavGraphNode *from, double xG, double yG, double aG,
                   std::list<NavGraphNode> &path, double *cost)
{
  NavGraphNode *to = getClosestNode(xG, yG, aG, m_MaxDist);
  if (to == 0) {
    CureCERR(30) << "No node within " << m_MaxDist 
                 << "m from desired goal (x="
                 << xG << " y=" << yG << "), failed findPath\n";
    return -2;
  }
  
  int ret = findPath(from, to, path, cost);
  if (ret) return ret;

  if (hypot(path.back().getX() - xG, path.back().getY() - yG) > 0.001) {
    CureCERR(40) << "Adding TEMPGOAL to path\n";
    NavGraphNode tempGoal("TEMPGOAL", -42, 
                          xG, yG, aG, 
                          path.back().getMaxSpeed());
    path.push_back(tempGoal);
  }

  return 0;
}

int
NewNavGraph::findPath(double xS, double yS, double aS, NavGraphNode *to, 
                   std::list<NavGraphNode> &path, double *cost)
{
  NavGraphNode *from = getClosestNode(xS, yS, aS, m_MaxDist);
  if (from == 0) {
    CureCERR(30) << "No node within " << m_MaxDist 
                 << "m from desired start (x="
                 << xS << " y=" << yS << "), failed findPath\n";
    return -2;
  }
  
  return findPath(from, to, path, cost);
}

int 
NewNavGraph::findPath(double xS, double yS, double aS,
                   double xG, double yG, double aG,
                   std::list<NavGraphNode> &path, double *cost)
{
  NavGraphNode *from = getClosestNode(xS, yS, aS, m_MaxDist);
  if (from == 0) {
    CureCERR(30) << "No node within " << m_MaxDist 
                 << "m from desired start (x="
                 << xS << " y=" << yS << "), failed findPath\n";
    return -2;
  }

  return findPath(from, xG, yG, aG, path, cost);
}

int
NewNavGraph::deleteNode(int id)
{
  return deleteNode(getNode(id));
}

int
NewNavGraph::deleteNode(const std::string &name)
{
  return deleteNode(getNode(name));
}

int
NewNavGraph::deleteNode(NavGraphNode *node)
{
  if (node == 0) {
    CureCERR(20) << "WARNING: Cannot remove node == NULL\n";
    return 1;
  }

  // Go through the neighbor nodes and disconnect them from this node
  // as well as deleting the edges from the list of edges
  for (std::list<NavGraphEdge*>::iterator nei = node->m_Edges.begin();
       nei != node->m_Edges.end(); nei++) {
    NavGraphNode *neighbor = (*nei)->getNext(node);
    if (neighbor) {
      neighbor->disconnectFrom(node);

      // Now also remove the edge from the list
      for (std::list<NavGraphEdge>::iterator ei = m_Edges.begin();
           ei != m_Edges.end(); ei++) {
        if ( (ei->getNode1() == neighbor && ei->getNode2() == node) ||
             (ei->getNode2() == neighbor && ei->getNode1() == node) ) {
          ei = m_Edges.erase(ei);
          break;
        }
      }
    } else {
      CureCERR(20) << "WARNING: Failed to find neighbor to " << *node << "\n";
    }
  }

  // Delete the node from the list of nodes
  bool removed = false;
  for (std::list<NavGraphNode*>::iterator ni = m_Nodes.begin();
       ni != m_Nodes.end(); ni++) {
    if (*ni == node) {
      m_Nodes.erase(ni);
      removed = true;
      break;
    }
  }
  if (!removed) { 
    CureCERR(20) << "WARNING: Could not find node to remove "
                 << *node << std::endl;
  }

  if (m_CurrNode == node) {
    CureCERR(20) << "WARNING: Deleting current node, not so good\n";
    notifyChangedCurrentNode(m_CurrNode->getId(), -1);
    m_CurrNode = 0;
  }

  if (node->getType() == NavGraphNode::NODETYPE_GATEWAY) {
    // Remove it from the list of gateways
    for (std::list<NavGraphGateway*>::iterator gi = m_Gateways.begin();
         gi != m_Gateways.end(); gi++) {
      if (*gi == node) {
        m_Gateways.erase(gi);
        break;
      }
    }
  }

  delete node;

  return 0;
}

int 
NewNavGraph::addGatewayNode(double x, double y, double a, double width)
{
  bool added = false;
  int ret = 0;

  if (m_CurrNode != 0) {
    if (m_CurrNode->getType() == NavGraphNode::NODETYPE_GATEWAY) {
      CureCERR(40) << "Current node is already a gateway node!!\n";
      return 1;
    } else {
      // Check if we are close to a gateway node connected to the current
      // node
      for (std::list<NavGraphEdge*>::iterator ei = m_CurrNode->m_Edges.begin();
           !added && ei != m_CurrNode->m_Edges.end(); ei++) {
        NavGraphNode *next = (*ei)->getNext(m_CurrNode);
        if (next != 0) {
          if (next->getType() == NavGraphNode::NODETYPE_GATEWAY&&
              hypot(y - next->getY(), x - next->getX()) < 2.0 * m_NodeDist) {
            CureCERR(40) << "Already close to gateway node "
                         << *next << "!!\n";    
            return 1;
          }
        }
      }

      // Check if we have another node very close by that we could move
      // slightly and turn into a gateway node
      if (m_CurrNode->getName() == "-" &&
          m_CurrNode->getType() == NavGraphNode::NODETYPE_NODE &&
          hypot(y - m_CurrNode->getY(), 
                x - m_CurrNode->getX()) < 0.05 * m_NodeDist) {
        if ((m_CurrNode->m_FlagMask & NavGraphNode::NODEFLAG_NOGATEWAY) == 0) {
          ret = makeGatewayOutOfNode(m_CurrNode, x,y,a,width);
        } else {
          // This node has been marked as not being a door which means
          // that it has been a door at some point and turned into a
          // normal node. We should thus not do this again.
          CureCERR(40) << "Not creating gateway, "
                       << "classified as non-gateway region\n";
          return -3;
        }
        added = true;
      }
      
      for (std::list<NavGraphEdge*>::iterator ei = m_CurrNode->m_Edges.begin();
           ei != m_CurrNode->m_Edges.end(); ei++) {
        NavGraphNode *next = (*ei)->getNext(m_CurrNode);
        if (next != 0 && next->getName() == "-") { // normal node
          if (hypot(y - next->getY(), x - next->getY()) < 0.5 * m_NodeDist) {
            if ((next->m_FlagMask & NavGraphNode::NODEFLAG_NOGATEWAY) == 0) {
              ret = makeGatewayOutOfNode(next, x, y, a, width);
            } else {
              // This node has been marked as not being a door which means
              // that it has been a door at some point and turned into a
              // normal node. We should thus not do this again.
              CureCERR(40) << "Not creating gateway, "
                           << "classified as non-gateway region\n";
              return -3;
            }
            added = true;
            break;
          }
        }
      }
    }
  }

  if (!added) {
    // Force in a new node
    CureCERR(40) << "Added gateway at x=" << x << " y=" << y << std::endl;
    ret = maybeAddNewNodeAt(x,y,a,true,true);
  }
  if (ret) {
    CureCERR(20) << "WARNING: Failed to create gateway node\n";
    return ret;
  }
  
  // Report this to any listener of these types of events
  if (m_CurrNode) {
    if (m_CurrNode->getType() == NavGraphNode::NODETYPE_GATEWAY) {
      ((NavGraphGateway*)m_CurrNode)->m_Width = width;
      std::list<NavGraphEventListener*>::iterator li;
      for (li = m_EventListeners.begin(); 
           li != m_EventListeners.end(); li++) {
        (*li)->newGateway(*((NavGraphGateway*)m_CurrNode));
      }
    } else {
      CureCERR(20) << "WARNING: Logic error, m_CurrNode not gateway\n";
    }
  } else {
    CureCERR(20) << "WARNING: Logic error, m_CurrNode == 0\n";
  }

  return 0;
}

int
NewNavGraph::turnGatewayIntoNode(int id, bool uniteAreaIds, long idToKeep)
{
  // First we find the gateway
  Cure::NavGraphNode *np = getNode(id);

  if (np->getType() != Cure::NavGraphNode::NODETYPE_GATEWAY) {
    CureCERR(20) << "WARNING: Expected a node " << id << " to be a gateway\n";
    return 1;
  }

  NavGraphGateway *g = (NavGraphGateway*)np;

  // Get hold of the iterator so that we can remove it from the list
  // of gateways
  std::list<NavGraphGateway*>::iterator gi;
  gi = find(m_Gateways.begin(), m_Gateways.end(), g);
  if (gi == m_Gateways.end()) {
    CureCERR(20) << "WARNING: Did not find gateway with id "
                 << g->getId() << std::endl;
    return 1;
  }

  // Remove pointer to the gateway from the list of gateways
  m_Gateways.erase(gi);

  // Turn the gateway into a node
  makeNodeOutOfGateway(*gi, (*gi)->getX(), (*gi)->getY(), (*gi)->getTheta());

  std::list<NavGraphEventListener*>::iterator li;
  for (li = m_EventListeners.begin(); 
       li != m_EventListeners.end(); li++) {
    (*li)->changedNodeType(id);
  }
  
  if (!uniteAreaIds) return 0;

  np = getNode(id);
  // Mark this node as not being a gateway so that we do not try to
  // turn it into such again at a later on
  np->m_FlagMask |= NavGraphNode::NODEFLAG_NOGATEWAY;

  if (np->m_Edges.size() <= 1) {
    // Simple case where we do not have to care out about changing any
    // area ids
    return 0;    
  }

  if (np->m_Edges.size() > 2) {
    CureCERR(20) << "WARNING: Not handling gateway update with #edgse>2\n";
    return 1;
  }

  // Find the ids on the two side of the gateway
  NavGraphNode *n1 = np->m_Edges.back()->getNext(np);
  NavGraphNode *n2 = np->m_Edges.front()->getNext(np);

  long aid1 = n1->getAreaId();
  long aid2 = n2->getAreaId();

  if (idToKeep >= 0) {
    if (idToKeep == aid1) {
      // All set to call mergeAreaIds!
    } else if (idToKeep == aid2) {
      // Need to swap order
      long tmp = aid1;
      aid1 = aid2;
      aid2 = tmp;
    } else {
      CureCERR(20) << "WARNING: idToKeep=" << idToKeep
                   << " does not match any of the two ids "
                   << aid1 << " " << aid2 << std::endl;
    }
  } else {
    // We have no preference regarding what id to keep so we keep the
    // one with the lowest id
    if (aid1 > aid2) {
      long tmp = aid1;
      aid1 = aid2;
      aid2 = tmp;
    }
  }

  mergeAreaIds(aid1, aid2);

  return 0;
}

int
NewNavGraph::replaceNode(NavGraphNode *origNode, NavGraphNode *newNode)
{
  if (!origNode) {
    CureCERR(20) << "WARNING: Cannot replace node == NULL\n";
    return 1;
  }
  if (!newNode) {
    CureCERR(20) << "WARNING: Cannot replace node with node == NULL\n";
    return 1;
  }

  if (origNode == newNode) {
    CureCERR(30) << "No need to replace nodes, they are the same "
                 << *newNode << std::endl;
    return 0;
  }

  // Make a list of all pointer to neighboring ndoes from te
  std::list<NavGraphNode*> neighbors;
  for (std::list<NavGraphEdge*>::iterator nei = origNode->m_Edges.begin();
       nei != origNode->m_Edges.end(); nei++) {
    NavGraphNode *neighbor = (*nei)->getNext(origNode);
    if (neighbor) neighbors.push_back(neighbor);
  }

  // We need to make sure that we do not leave a pointer to something
  // we will delete
  if (m_CurrNode == origNode) {
    if (m_CurrNode->getAreaId() != newNode->getAreaId()) {
      notifyChangedArea(newNode->getAreaId());
    }
    notifyChangedCurrentNode(m_CurrNode->getId(), newNode->getId());
    m_CurrNode = newNode;
  }

  // Delete the old node and with it all connections
  deleteNode(origNode);

  // Add the new node
  m_Nodes.push_back(newNode);

  // Connect the new node to the neighbors of the node it replaces
  for (std::list<NavGraphNode*>::iterator ni = neighbors.begin();
       ni != neighbors.end(); ni++) {
    connectNodes(newNode, *ni, hypot(newNode->getY() - (*ni)->getY(),
                                     newNode->getY() - (*ni)->getY()));
  }

  return 0;
}

int
NewNavGraph::makeGatewayOutOfNode(NavGraphNode *node, 
                               double x, double y, double a, double width)
{
  if (node == 0) {
    CureCERR(20) << "WARNING: Cannot turn node=NULL into gateway\n";
    return 1;
  }
  
  CureCERR(40) << "\nShifting existing node " << *node << " and turning it "
               << "into gateway node at "
               << "x=" << x << " y=" << y << " a=" << a << "\n";

  // If this node has more than one connection we might have a problem
  // because it might be a door that we have not detected before.
  if (node->m_Edges.size() > 1) {
    // Prepare for recursive search, by marking all nodes as not
    // visited
    for (std::list<NavGraphNode*>::iterator ni = m_Nodes.begin();
         ni != m_Nodes.end(); ni++) {
      (*ni)->costG = 0; // Mark as not visited      
    }

    // Look for the node to start searching at. This should be the
    // node behind the gateway we are about to create
    NavGraphNode *behind[node->m_Edges.size()];
    int nBehind = 0;
    for (std::list<NavGraphEdge*>::iterator ei = node->m_Edges.begin();
         ei != node->m_Edges.end(); ei++) {
      NavGraphNode *next = (*ei)->getNext(node);
      if (next) {
        double ang = atan2(next->getY() - y, next->getX() - x);
        double da = fabs(HelpFunctions::angleDiffRad(a, ang));
        if (da > M_PI_2) {
          CureCERR(50) << "Behind[" << nBehind << "] "
                       << *next << std::endl;
          behind[nBehind++] = next;
       }
      }
    }

    if (nBehind > 0) {

      // First we check to make sure that there is no way to reach the
      // node that will become a gateway without going through another
      // gateway or any of the nodes that have been classifed as being
      // behind it. If such a path exist it is likely not a door but
      // most likely just some objects being close to each other.

      // Lopp over the nodes behind
      for (int i = 0; i < nBehind; i++) {
        behind[i]->costG = 1; // Mark as visited to close this path
        for (std::list<NavGraphEdge*>::iterator ei=behind[i]->m_Edges.begin();
             ei != behind[i]->m_Edges.end(); ei++) {
          // Start a searc as long as the node has not yet been
          // visited and is not the gateway we try to find an
          // alternative route to.
          NavGraphNode *next = (*ei)->getNext(behind[i]);
          if (next && next != node) {
            if (foundPathToNode(node, next)) {
              CureCERR(30) << "Found path around to the node to turn "
                           << " into gateway, probably not a gateway\n";
              return 1;
            }
          }
        }
      }
      CureCERR(40) << "No other path to gateway\n";

      // Clear the cost of all nodes again
      for (std::list<NavGraphNode*>::iterator ni = m_Nodes.begin();
           ni != m_Nodes.end(); ni++) {
        (*ni)->costG = 0; // Mark as not visited      
      }
      
      m_LastAreaId++;
      // Change the area id of all nodes behind us
      for (int i = 0; i < nBehind; i++) {
        behind[i]->costG = 1; // Mark as visited to close this path
        behind[i]->setAreaId(m_LastAreaId);
        for (std::list<NavGraphEdge*>::iterator ei=behind[i]->m_Edges.begin();
             ei != behind[i]->m_Edges.end(); ei++) {
          NavGraphNode *next = (*ei)->getNext(behind[i]);
          if (next && next != node) {
            spreadNewAreaId(m_LastAreaId, next);
          }
        }
      }

    } else {
      CureCERR(20) << "WARNING: Strange, all nodes infront of gateway "
                   << *node << "\n";
    }
  }

  NavGraphGateway *g = new NavGraphGateway(node->getName(),
                                           node->getId(),
                                           x, y, a,
                                           node->getMaxSpeed(),
                                           width);
  g->setAreaId(node->getAreaId());
  replaceNode(node, g);
  m_Gateways.push_back(g);

  return 0;
}

bool
NewNavGraph::foundPathToNode(NavGraphNode *goal, NavGraphNode *node)
{
  if (node == 0) return false;

  node->costG = 1;
  for (std::list<NavGraphEdge*>::iterator ei = node->m_Edges.begin();
       ei != node->m_Edges.end(); ei++) {
    NavGraphNode *next = (*ei)->getNext(node);
    if (next && node->costG == 0 && 
        next->getType() != NavGraphNode::NODETYPE_GATEWAY) {      
      if (next == goal) return true;
      else return foundPathToNode(goal, next);
    }
  }

  return false;
}

void
NewNavGraph::spreadNewAreaId(long areaId, NavGraphNode *node)
{
  if (node == 0) return;

  node->costG = 1;
  node->setAreaId(areaId);
  for (std::list<NavGraphEdge*>::iterator ei = node->m_Edges.begin();
       ei != node->m_Edges.end(); ei++) {
    NavGraphNode *next = (*ei)->getNext(node);
    if (next && node->costG == 0 && 
        next->getType() != NavGraphNode::NODETYPE_GATEWAY) {      
      spreadNewAreaId(areaId, next);
    }
  }
}

int
NewNavGraph::makeNodeOutOfGateway(NavGraphGateway *g, 
                               double x, double y, double a)
{
  if (g == 0) {
    CureCERR(20) << "WARNING: Cannot turn gateway=NULL into node\n";
    return 1;
  }
  
  CureCERR(40) << "\nTurning gateway " << *g << " into normal node at "
               << "x=" << x << " y=" << y << " a=" << a << "\n";

  NavGraphNode *node = new NavGraphNode(g->getName(),
                                        g->getId(),
                                        x, y, a,
                                        g->getMaxSpeed());
  node->setAreaId(g->getAreaId());
  replaceNode(g, node);

  return 0;
}

std::list<NavGraphGateway*>::iterator
NewNavGraph::findConnectingGateway(long aid1, long aid2)
{
  // Loop over all gateways
  for (std::list<NavGraphGateway*>::iterator gi = m_Gateways.begin();
       gi != m_Gateways.end(); gi++) {
    
    // Check if has an area_id that matches one of the two conflicting
    // ids and has 2 connections.
    if (((*gi)->getAreaId() == aid1 || (*gi)->getAreaId() == aid2) && 
        (*gi)->m_Edges.size() == 2) {
      
      // Loop over all edges
      for (std::list<NavGraphEdge*>::iterator ei = (*gi)->m_Edges.begin();
           ei != (*gi)->m_Edges.end(); ei++) {
        
        // Get a pointer to the node connected to by this edge
        NavGraphNode *next = (*ei)->getNext(*gi);
        if (next) {
          
          // Check if the gateway node (gi) and the conected node
          // (next) have ids corresponding to the conflicting ones.
          if (((*gi)->getAreaId() == aid1 && next->getAreaId() == aid2) ||
              ((*gi)->getAreaId() == aid2 && next->getAreaId() == aid1) ) {
            
            // We have found a gateway that seem to be the problem
            return gi;
          }
        }
      }
    }
  }

  return m_Gateways.end();
}

int
NewNavGraph::replaceGatewaysConnecting(long aid1, long aid2)
{
  std::list<NavGraphGateway*>::iterator gi = findConnectingGateway(aid1, aid2);
  if (gi == m_Gateways.end()) {
    CureCERR(20) << "WARNING: Did not find gateway connecting "
                 << aid1 << " and " << aid2 << std::endl;
    return 1;
  }

  // We remove it from the list before we turn it into a
  // node as the gateway object will be deleted and the
  // iterator thus no longer valid
  m_Gateways.erase(gi);
  makeNodeOutOfGateway(*gi, (*gi)->getX(), (*gi)->getY(), (*gi)->getTheta());
  return 0;
}

int
NewNavGraph::mergeAreaIds(long aid1, long aid2)
{
  for (std::list<NavGraphNode*>::iterator ni = m_Nodes.begin();
       ni != m_Nodes.end(); ni++) {
    if ((*ni)->getAreaId() == aid2) (*ni)->setAreaId(aid1);
  }
  
  std::list<NavGraphEventListener*>::iterator li;
  for (li = m_EventListeners.begin(); 
       li != m_EventListeners.end(); li++) {
    (*li)->mergedAreaIds(aid1, aid2);
  }
  
  return 0;
}

int 
NewNavGraph::connectNodes(NavGraphNode *n1, NavGraphNode *n2, double cost)
{
  if (n1 == 0 || n2 == 0) {
    CureCERR(20) << "Cannot connect nodes==0, n1=" 
                 << n1 << " n2=" << n2 << "\n";
    return 1;
  }

  // Make sure that these nodes are not already connected
  for (std::list<NavGraphEdge>::iterator ei = m_Edges.begin();
       ei != m_Edges.end(); ei++) {
    if ( (ei->getNodeId1() == n1->getId() &&
          ei->getNodeId2() == n2->getId()) ||
         (ei->getNodeId1() == n2->getId() &&
          ei->getNodeId2() == n1->getId()) ) {
      CureCERR(60) << "Nodes already connected\n";
      return 0;
    }
  }

  // Create the edge between the two nodes
  m_Edges.push_back(NavGraphEdge(n1, n2, 
                                 hypot(n1->getY() - n2->getY(), 
                                       n1->getX() - n2->getX())));

  // Node 1
  NavGraphNode::NavGraphEdgeInfo gei;
  gei.nextId = n2->getId();
  gei.cost = cost;
  n1->m_EdgeInfo.push_back(gei);
  n1->m_Edges.push_back(&(m_Edges.back()));
  
  // Node 2
  gei.nextId = n1->getId();
  n2->m_EdgeInfo.push_back(gei);
  n2->m_Edges.push_back(&(m_Edges.back()));

  return 0;
}

int
NewNavGraph::markPose(double x, double y, double a, const std::string &tag)
{
  if (tag == "") {
    CureCERR(30) << "Cannot tag pose with empty string\n";
    return 1;
  }

  int ret = maybeAddNewNodeAt(x, y, a, true, NavGraphNode::NODETYPE_NODE);
  if (ret == 0 && m_CurrNode) {
    m_CurrNode->setName(tag);
  }
  return ret;
}

NavGraphNode*
NewNavGraph::getClosestNode(double x, double y, double a, double maxDist)
{
  NavGraphNode *ret = 0;
  double minD = 1e10;
  for(std::list<NavGraphNode*>::iterator ni = m_Nodes.begin();
      ni != m_Nodes.end(); ni++) {
    double d = hypot(y - (*ni)->getY(), x - (*ni)->getX());
    if ((maxDist < 0 || d < maxDist) && d < minD) {
      minD = d;
      ret = *ni;
    }
  }
  
  return ret;
}

int 
NewNavGraph::loadGraph(const std::string &filename)
{
  std::fstream fs;
  fs.open(filename.c_str(), std::ios::in);
  if (fs <= 0) {
    CureCERR(20) << "Failed to open graph file \""
                 << filename << "\"\n";
    return 1;
  }

  int ret = readFromStream(fs);
  fs.close();
  return ret;
}

int 
NewNavGraph::saveGraph(const std::string &filename)
{
  std::fstream fs;
  fs.open(filename.c_str(), std::ios::out);
  if (fs <= 0) {
    CureCERR(20) << "Failed to open graph file \""
                 << filename << "\"\n";
    return 1;
  }

  int ret = writeToStream(fs);
  fs.close();
  return ret;
}

int 
NewNavGraph::addNodeToNodeList(int id, int areaId,
                            double x, double y, double a,
                            const std::string &name,
                            double maxspeed)
{
  m_Nodes.push_back(new NavGraphNode(name, id, x, y, a, maxspeed));
  m_Nodes.back()->setAreaId(areaId);

  if (id >= m_NextNodeId) m_NextNodeId = id +1;
  if (areaId > m_LastAreaId) m_LastAreaId = areaId;

  return 0;
}

int 
NewNavGraph::addDoorToNodeList(int id, int areaId,
                            double x, double y, double a, double width,
                            const std::string &name,
                            double maxspeed)
{
  NavGraphGateway *g = new NavGraphGateway(name, id, 
                                           x, y, a, maxspeed,
                                           width);
  m_Nodes.push_back(g);
  m_Gateways.push_back(g);
  m_Nodes.back()->setAreaId(areaId);  

  if (id > m_NextNodeId) m_NextNodeId = id +1;
  if (areaId > m_LastAreaId) m_LastAreaId = areaId;

  return 0;
}


int
NewNavGraph::addEdgeToEdgeList(int id1, int id2)
{
  if (getEdge(id1, id2)) {
    return 0;
  }
  
  NavGraphNode *n1 = getNode(id1);
  if (n1 == 0) {
    return -1;
  }
  NavGraphNode *n2 = getNode(id2);
  if (n2 == 0) {
    return -1;
  }

  double cost = hypot(n2->getY() - n1->getY(), n2->getX() - n1->getX());
  m_Edges.push_back(NavGraphEdge(n1, n2, cost));
  return 0;
}

int
NewNavGraph::readFromStream(std::istream &is)
{
  std::string line, key;
  int n;

  clear();
  
  // Read NODES
  int lineno = 0;
  getline(is, line);
  {
    std::istringstream str(line);
    str >> key >> n;
    std::cerr << "Got " << n << " nodes\n";
    for (int i = 0; i < n; i++) {
      lineno++;
      getline(is, line);
      std::istringstream istr(line);
      int type, id, aid = 0;
      std::string name = "-";
      double x,y,a;
      if ( istr >> type >> id >> x >> y >> a) {
        if (id >= m_NextNodeId) m_NextNodeId = id+1;
        if (istr >> name >> aid) {
          if (aid > m_LastAreaId) m_LastAreaId = aid;
        }
        if (type == NavGraphNode::NODETYPE_GATEWAY) {
          // Attempt to read the width of the door
          double width;
          if ( !(istr >> width) ) {
            width = 1.0;
            CureCERR(30) << "No width for gateway id=" << id
                         << " assuming " << width << "m\n";
          }

          addDoorToNodeList(id, aid, x, y, a, width, name, m_MaxSpeed);
        } else {

          addNodeToNodeList(id, aid, x,y,a, name, m_MaxSpeed);
        }
      } else {
        CureCERR(20) << "Failed to read node data on line " << lineno << "\n";
      }
    }
  }

  // Read EDGES
  lineno++;
  getline(is, line);
  {
    std::istringstream str(line);
    str >> key >> n;
    std::cerr << "Got " << n << " edges\n";
    for (int i = 0; i < n; i++) {
      lineno++;
      getline(is, line);
      std::istringstream istr(line);
      long id1, id2;
      if ( istr >> id1 >> id2 ) {

        if (addEdgeToEdgeList(id1, id2) == 0) {

          // Read cost if it is given in the file
          double cost;
          if (istr >> cost) {
            m_Edges.back().setCost(cost);
          }
        } else {
          CureCERR(20) << "WARNING: Problems with edge on line " 
                       << lineno << "\n";
        }
        
      } else {
        CureCERR(20) << "WARNING: Failed to read edge data on line " 
                     << lineno << "\n";
      }
    }
  }

  return connectNodes();
}

int
NewNavGraph::writeToStream(std::ostream &os)
{
  // Write NODES
  os << "NODES " << m_Nodes.size() << std::endl;
  for (std::list<NavGraphNode*>::iterator ni = m_Nodes.begin();
       ni != m_Nodes.end(); ni++) {
    os << (*ni)->getType() << " "
       << (*ni)->getId() << " " 
       << (*ni)->getX() << " "
       << (*ni)->getY() << " "
       << (*ni)->getTheta() << " "
       << (*ni)->getName() << " "
       << (*ni)->getAreaId();
    if ((*ni)->getType() == NavGraphNode::NODETYPE_GATEWAY) {
      os << " " << ((NavGraphGateway*)(*ni))->getWidth();
    }
    os << std::endl;
  }

  // Write EDGES
  os << "EDGES " << m_Edges.size() << std::endl;
  for (std::list<NavGraphEdge>::iterator ei = m_Edges.begin();
       ei != m_Edges.end(); ei++) {
    os << ei->getNodeId1() << " "
       << ei->getNodeId2() << " " 
       << ei->getCost() << std::endl;
  }

  return 0;
}

int
NewNavGraph::connectNodes()
{
  for(std::list<NavGraphEdge>::iterator ei = m_Edges.begin();
      ei != m_Edges.end(); ei++) {
    if (ei->getNode1() == 0 || ei->getNode2() == 0) {      
      CureCERR(30) << "Edge " << *ei << " NOT CONNECTED\n";
      continue;
    }

    // Node 1
    NavGraphNode::NavGraphEdgeInfo gei;
    gei.nextId = ei->getNodeId2();
    gei.cost = ei->getCost();
    ei->getNode1()->m_EdgeInfo.push_back(gei);
    ei->getNode1()->m_Edges.push_back(&*ei);
    
    // Node 2
    gei.nextId = ei->getNodeId1();
    gei.cost = ei->getCost();
    ei->getNode2()->m_EdgeInfo.push_back(gei);
    ei->getNode2()->m_Edges.push_back(&*ei);
  }

  return 0;
}

NavGraphNode* 
NewNavGraph::getNode(const std::string &name)
{
  for(std::list<NavGraphNode*>::iterator ni = m_Nodes.begin();
      ni != m_Nodes.end(); ni++) {
    if ((*ni)->getName() == name) return *ni;
  }

  return 0;
}

NavGraphNode* 
NewNavGraph::getNode(const long id)
{
  for(std::list<NavGraphNode*>::iterator ni = m_Nodes.begin();
      ni != m_Nodes.end(); ni++) {
    if ((*ni)->getId() == id) return *ni;
  }

  return 0;
}

NavGraphEdge*
NewNavGraph::getEdge(const long id1, const long id2)
{
  for(std::list<NavGraphEdge>::iterator ei = m_Edges.begin();
      ei != m_Edges.end(); ei++) {
    if ( (ei->getNodeId1() == id1 && ei->getNodeId2() == id2) ||
         (ei->getNodeId1() == id2 && ei->getNodeId2() == id1) ) {
      return &*ei;
    }
  }

  return 0;
}

NavGraphGateway* 
NewNavGraph::getGateway(const long id)
{
  for(std::list<NavGraphGateway*>::iterator gi = m_Gateways.begin();
      gi != m_Gateways.end(); gi++) {
    if ((*gi)->getId() == id) return *gi;
  }

  return 0;
}

std::list<NavGraphGateway*>
NewNavGraph::getGatewaysInArea(const long aid)
{
  std::list<NavGraphGateway*> ret;

  for (std::list<Cure::NavGraphGateway*>::iterator gi = m_Gateways.begin();
       gi != m_Gateways.end(); gi++) {
    Cure::NavGraphGateway *gw = *gi;
    Cure::NavGraphNode *n = gw;
    // Check if this gateway is connected to the area in question
    bool inThisArea = (n->getAreaId() == aid);

    if (!inThisArea) {
      for (std::list<Cure::NavGraphEdge*>::iterator ei = n->m_Edges.begin();
           ei != n->m_Edges.end(); ei++) {
        if ((*ei)->getNode1()->getAreaId() == aid) {
          n = (*ei)->getNode1();
          inThisArea = true;
          break;
        } else if ((*ei)->getNode2()->getAreaId() == aid) {
          n = (*ei)->getNode2();
          inThisArea = true;
          break;
        }
      }
    }

    if (!inThisArea) continue; // Not interesting

    ret.push_back(gw);
  }

  return ret;
}

void 
NewNavGraph::displayRL(RoboLookProxy *rlp, int env, 
                    bool clearStars, bool clearLines, bool showDoorFrames)
{
  if (!rlp) return;

  CureCERR(60) << "Displaying NavGraph\n";

  RL_StarItem star[m_Nodes.size()];
  int n = 0;
  for (std::list<NavGraphNode*>::iterator ni = m_Nodes.begin();
       ni != m_Nodes.end(); ni++) {
    star[n].x = (*ni)->getX();
    star[n].y = (*ni)->getY();
    star[n].z = 0;
    star[n].w = 0.1;
    if (*ni == m_CurrNode) { 
      star[n].color = 1;  // Green
      star[n].w = m_RLStarSizeCurrent;
    } else if ((*ni)->getType() == NavGraphNode::NODETYPE_GATEWAY) {
      star[n].color = 2;  // Red
      NavGraphGateway *g = (NavGraphGateway*)(*ni);      
      if (g->getDoorState() == NavGraphGateway::CLOSED) {
        star[n].w = m_RLStarSizeGatewayBlocked;
      } else {
        star[n].w = m_RLStarSizeGateway;
      }
    } else if ((*ni)->getName() != "-") {
      star[n].color = 2;  // Red
      star[n].w = m_RLStarSizeNamedGoal;
    } else {
      star[n].color = 3+((*ni)->getAreaId()%6);  // changing color
      star[n].w = m_RLStarSizeNormal;
    }
    n++;
  }
  CureCERR(60) << "Displaying " << n << " nodes\n";
  rlp->addStars(env, star, n, clearStars);  

  RL_LineItem edge[m_Edges.size()];
  n = 0;
  for (std::list<NavGraphEdge>::iterator ei = m_Edges.begin();
       ei != m_Edges.end(); ei++) {
    if (ei->getNode1() == 0 || ei->getNode2() == 0) {
      continue;
    }

    edge[n].xS = ei->getNode1()->getX();
    edge[n].yS = ei->getNode1()->getY();
    edge[n].zS = 0;
    edge[n].xE = ei->getNode2()->getX();
    edge[n].yE = ei->getNode2()->getY();
    edge[n].zE = 0;
    edge[n].color = 7;
    edge[n].width = 1;
    n++;
  }
  CureCERR(60) << "Displaying " << n << " edges\n";
  rlp->addLines(env, edge, n, clearStars);

  if (showDoorFrames) {
    RL_DoorItem door[m_Gateways.size()];
    n = 0;
    for (std::list<NavGraphGateway*>::iterator gi = m_Gateways.begin();
         gi != m_Gateways.end(); gi++) {

      NavGraphGateway *gw = (NavGraphGateway*)*gi;

      door[n].xC = (*gi)->getX();
      door[n].yC = (*gi)->getY();
      door[n].angle = (*gi)->getTheta() + M_PI_2;
      door[n].zMin = 0;
      door[n].zMax = 2;
      door[n].width = (*gi)->getWidth();
      CureCERR(60) << "Door " << gw->getId() << ", state=" 
                   << gw->getDoorState() << std::endl;
      //if (gw->getDoorState() == NavGraphGateway::CLOSED) {
      //door[n].hasLeaf = 1;
      //} else {
      door[n].hasLeaf = 0;
      //}
      door[n].rightSideHinges = 0;
      door[n].leafAngle = 0;
      n++;
    }
    rlp->addDoors(env, door, n, true);
  }
}

void
NewNavGraph::addEventListener(NavGraphEventListener *l)
{
  // Make sure that this is not already a registered listener
  if (find(m_EventListeners.begin(), m_EventListeners.end(), l) != 
      m_EventListeners.end()) {
    CureCERR(30) << l->getName() << " already added\n";
    return;
  }

  m_EventListeners.push_back(l);
}

void
NewNavGraph::delEventListener(NavGraphEventListener *l)
{
  std::list<NavGraphEventListener*>::iterator li;
  li = find(m_EventListeners.begin(), m_EventListeners.end(), l);
  if (li != m_EventListeners.end()) {
    m_EventListeners.erase(li);
    return;
  }

  CureCERR(30) << l->getName() << " not registered\n";
}

void
NewNavGraph::setAutoFix(bool autofix)
{
  m_AutoFixGateway = autofix;
}

bool 
NewNavGraph::setAreaName(int areaId, const std::string &name)
{
  std::map<int, NavArea>::iterator ai = m_Areas.find(areaId);
  if (ai != m_Areas.end()) {
    ai->second.m_AreaName = name;
    return true;
  }

  return false;
}

bool 
NewNavGraph::setAreaClass(int areaId, int c)
{
  std::map<int, NavArea>::iterator ai = m_Areas.find(areaId);
  if (ai != m_Areas.end()) {
    ai->second.m_AreaClass = c;
    return true;
  }

  return false;  
}

std::string 
NewNavGraph::getAreaName(int areaId) const
{
  std::map<int, NavArea>::const_iterator ai = m_Areas.find(areaId);
  if (ai != m_Areas.end()) {
    return ai->second.m_AreaName;    
  }

  return "-";  
}

int 
NewNavGraph::getAreaClass(int areaId)
{
  // We use the mojority vote between the nodes to tell what type this
  // area is
  int minClass = NavArea::AREACLASS_CORRIDOR;
  int maxClass = NavArea::AREACLASS_ROOM;
  int votes[maxClass - minClass + 1];
  for (int i = 0; i <= maxClass - minClass; i++) {
    votes[i] = 0;
  }

  for (std::list<NavGraphNode*>::iterator ni = m_Nodes.begin();
       ni != m_Nodes.end(); ni++) {
    if ((*ni)->getAreaId() == areaId) {
      if (minClass <= (*ni)->getAreaClass() && 
          (*ni)->getAreaClass() <= maxClass) {
        votes[(*ni)->getAreaClass() - minClass]++;
      }
    }
  }
  
  int bestClass = -1;
  int maxVotes = 0;
  CureCERR(30) << "Area votes for area " << areaId << ": ";
  for (int i = 0; i <= maxClass - minClass; i++) {    
    std::cerr << votes[i] << " ";
    if (votes[i] >= maxVotes) {
      maxVotes = votes[i];
      bestClass = minClass + i;
    }
  }
  std::cerr << std::endl;

  if (maxVotes > 0) {
    CureCERR(30) << "Best vote for area " << areaId 
                 << " is " << bestClass << std::endl;
  } else {
    CureCERR(30) << "No class found for area " << areaId << std::endl;
  }
  return bestClass;

  /*
  std::map<int, NavArea>::const_iterator ai = m_Areas.find(areaId);
  if (ai != m_Areas.end()) {
    return ai->second.m_AreaClass;    
  }

  return NavArea::AREACLASS_UNKNOWN;    
  */
}

bool 
NewNavGraph::setGatewayStatesToUnknown()
{
  for (std::list<NavGraphGateway*>::iterator gi = m_Gateways.begin();
       gi != m_Gateways.end(); gi++) {
    (*gi)->setDoorState(NavGraphGateway::UNKNOWN);
  }
  return true;
}

void 
NewNavGraph::notifyChangedArea(long areaId)
{
  CureCERR(30) << "Notifying that the area changed to " << areaId << "\n";
  std::list<NavGraphEventListener*>::iterator li;
  for (li = m_EventListeners.begin(); 
       li != m_EventListeners.end(); li++) {
    (*li)->changedArea(areaId);
  }
}

void 
NewNavGraph::notifyChangedCurrentNode(long fromId, long toId)
{
  CureCERR(60) << "Notifying that the current noded changed from " 
               << fromId << " to " << toId << std::endl;
  std::list<NavGraphEventListener*>::iterator li;
  for (li = m_EventListeners.begin(); 
       li != m_EventListeners.end(); li++) {
    (*li)->changedCurrentNode(fromId, toId);
  }
}

std::ostream&
operator<< (std::ostream &os, const NewNavGraph &a)
{
  os << "NavGraph: currNodeId=" << a.getCurrentNodeId() << "\n";
  os << "=================== Nodes =======================\n";
  for(std::list<NavGraphNode*>::const_iterator ni = a.m_Nodes.begin();
      ni != a.m_Nodes.end(); ni++) {
    os << **ni << std::endl;
  }
  os << "=================== Edges =======================\n";
  for(std::list<NavGraphEdge>::const_iterator ei = a.m_Edges.begin();
      ei != a.m_Edges.end(); ei++) {
    os << *ei << std::endl;
  }
  return os;
}

