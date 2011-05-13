/**
 * @file ObjRep.cpp
 * @author Andreas Richtsfeld
 * @date March 2011
 * @version 0.1
 * @brief Object representation as probabilistic graph, consisting of nodes and links.
 */


#include "ObjRep.h"
#include "Gestalt3D.h"
#include "GraphLink.h"

namespace Z
{
 
/**
 * @brief Constructor for a object representation.
 */
ObjRep::ObjRep()
{}

/**
 * @brief Constructor for a object representation.
 * @param sc Stereo core to process.
 */
void ObjRep::Process(Z::StereoCore *sc)
{
  CheckLinkProbability();

  for(int t=0; t<Gestalt3D::MAX_TYPE; t++)
  {
    Gestalt3D::Type type = ((Gestalt3D::Type) t);
    for(unsigned i=0; i<sc->NumGestalts3D(type); i++)
    {
      vector<GraphLink> graphLinks;
      if(sc->Gestalts3D(type, i)->GetLinks(graphLinks))
      {
	for(unsigned li=0; li<graphLinks.size(); li++)
	{
	  AddLink(graphLinks[li]);
	}
      }
    }   
  }

  DeleteUnusedNodes();
//   PrintGraph();
}

/**
 * @brief Get the graph model as links with start and end (linking points)
 * and with the probability value of correct link.
 * @param first First point of the link
 * @param Second Second point of the link
 * @param probabi1ity Probability values 
 * @return Return true for success.
 */
bool ObjRep::GetGraphModel(std::vector<cv::Point3d> &first, std::vector<cv::Point3d> &second, std::vector<double> &probability)
{
  for(unsigned i=0; i<links.size(); i++)
  {
    cv::Point3d p0, p1;
    for(unsigned j=0; j<nodes.size(); j++)
    {
      if(nodes[j].ID() == links[i].GetNodeID(0))
	p0 =  nodes[j].GetPosition();
      if(nodes[j].ID() == links[i].GetNodeID(1))
	p1 =  nodes[j].GetPosition();
    }
    
    first.push_back(p0);
    second.push_back(p1);
    probability.push_back(links[i].GetProbability());
  }
  return true;
}


/**
 * @brief Get the graph model as links with start and end (linking points)
 * and with the probability value of correct link.
 * @param first First point of the link
 * @param Second Second point of the link
 * @param probabi1ity Probability values 
 * @return Return true for success.
 */
bool ObjRep::GetObjectGraphModel(std::vector< std::vector<cv::Point3d> > &first, 
				 std::vector< std::vector<cv::Point3d> > &second,
				 std::vector< std::vector<double> > &probability, 
				 std::vector< std::vector<std::string> > &link, 
				 std::vector< std::vector<std::string> > &node_0, 
				 std::vector< std::vector<std::string> > &node_1)
{
  SetNodesUnconsidered();
  SetLinksUnconsidered();

  std::vector<cv::Point3d> f;
  std::vector<cv::Point3d> s;
  std::vector<double> p;
  std::vector<std::string> l;
  std::vector<std::string> n0;
  std::vector<std::string> n1;
    
  for(unsigned i=0; i<nodes.size(); i++)
  {
    std::vector<unsigned> nodeIDs;
    
    if(!nodes[i].IsConsidered())
    {
      nodeIDs.push_back(nodes[i].ID());
      nodes[i].SetConsidered();
    }
      
   
// printf("\nObjRep::GetObjectGraphModel: start\n");
    while (nodeIDs.size() != 0)
    {
// printf("      ::GetObjectGraphModel: node: %u\n", nodeIDs[0]);

      // find all (unconsidered) links with this node and add other node (if not considered)
      for(unsigned j=0; j<links.size(); j++)
      {
	if(links[j].GetNodeID(0) == nodeIDs[0])
	{
	  // link already considered?
	  if(!links[j].IsConsidered())
	  {
	    // save link
	    links[j].SetConsidered();
	    
	    /// TODO NOW we have a node and a link?
// printf("hier passiert ein Fehler 0\n");
	    f.push_back(GetNode(links[j].GetNodeID(0)).GetPosition());
// printf("hier passiert ein Fehler 1\n");
	    s.push_back(GetNode(links[j].GetNodeID(1)).GetPosition());
// printf("hier passiert ein Fehler 2\n");
	    p.push_back(links[j].GetProbability());
	    
	    
	    char link[20];
	    sprintf(link, "%u", j);
	    l.push_back(link);
	    n0.push_back(link);
	    n1.push_back(link);

	    // push_back link with node
	    if(!IsNodeConsidered(links[j].GetNodeID(1)))
	    {
	      nodeIDs.push_back(links[j].GetNodeID(1));
	      SetNodeConsidered(links[j].GetNodeID(1));
	    }
	  }
	}
	if(links[j].GetNodeID(1) == nodeIDs[0])
	{
	  // link already considered?
	  if(!links[j].IsConsidered())
	  {
	    // save link
	    links[j].SetConsidered();

	    /// TODO NOW we have a node and a link?
// printf("hier passiert ein Fehler 3\n");
	    f.push_back(GetNode(links[j].GetNodeID(0)).GetPosition());
// printf("hier passiert ein Fehler 4\n");
	    s.push_back(GetNode(links[j].GetNodeID(1)).GetPosition());
// printf("hier passiert ein Fehler 5\n");
	    p.push_back(links[j].GetProbability());

	    char link[20];
	    sprintf(link, "%u", j);
	    l.push_back(link);
	    n0.push_back(link);
	    n1.push_back(link);
	    
	    // push_back link with node
	    if(!IsNodeConsidered(links[j].GetNodeID(0)))
	    {
	      nodeIDs.push_back(links[j].GetNodeID(0));
	      SetNodeConsidered(links[j].GetNodeID(0));
	    }
	  }
	}
      }
      nodeIDs.erase(nodeIDs.begin());
    }
    
    if(f.size() > 0)
    {
// printf("ObjRep::GetObjectGraphModel: WE HAVE A NEW OBJECT!!!\n");

      first.push_back(f);
      second.push_back(s);
      probability.push_back(p);
      link.push_back(l);
      node_0.push_back(n0);
      node_1.push_back(n1);

      f.clear();
      s.clear();
      p.clear();
      l.clear();
      n0.clear();
      n1.clear();
    }
  }
  return true;
}

// ************************ private ************************ //
/**
 * @brief Set all nodes unconsidered.
 */
void ObjRep::SetNodesUnconsidered()
{
  for(unsigned i=0; i<nodes.size(); i++)
    nodes[i].SetUnconsidered();
}

/**
 * @brief Set all links unconsidered.
 */
void ObjRep::SetLinksUnconsidered()
{
  for(unsigned i=0; i<links.size(); i++)
    links[i].SetUnconsidered();
}

/**
 * @brief Get a node with a certain id.
 * @param id ID of the node
 * @return Returns the node with id
 */
Node ObjRep::GetNode(unsigned id)
{
  for(unsigned i=0; i<nodes.size(); i++)
    if(nodes[i].ID() == id)
      return nodes[i];
  printf("ObjRep::GetNode: error: Node with id=%u is not allocable.\n", id);
  return nodes[0]; // HACK!!!
}

/**
 * @brief Get the position of a node within the vector array.
 * @param id ID of the node
 * @return Returns the node postion in the nodes-array.
 */
unsigned ObjRep::GetNodePos(unsigned id)
{
  for(unsigned i=0; i<nodes.size(); i++)
    if(nodes[i].ID() == id)
      return i;
  printf("ObjRep::GetNodePos: error: Node with id=%u is not allocable.\n", id);
  return 0;
}

/**
 * @brief Set a node as considered.
 * @param id ID of the node
 * @return Returns true for success.
 */
bool ObjRep::SetNodeConsidered(unsigned id)
{
  for(unsigned i=0; i<nodes.size(); i++)
  {
    if(nodes[i].ID() == id)
    {
      nodes[i].SetConsidered();
      return true;
    }
  }
  return false;
}

/**
 * @brief Returns true, if node is already considered.
 * @param id ID of the node
 * @return Returns true, if the node is already considered.
 */
bool ObjRep::IsNodeConsidered(unsigned id)
{
  for(unsigned i=0; i<nodes.size(); i++)
    if(nodes[i].ID() == id)
      if(nodes[i].IsConsidered())
	return true;
  return false;
}

/**
 * @brief Print the graph.
 */
unsigned ObjRep::GetUniqueID()
{
  static int id = 0;
  return id++;
}

/**
 * @brief Print the graph.
 */
void ObjRep::PrintGraph()
{
  printf("ObjRep::PrintGraph: we have %u nodes and %u links\n", nodes.size(), links.size());
  
  for(unsigned i=0; i<links.size(); i++)
  {
    cv::Point3d p0, p1;
    for(unsigned j=0; j<nodes.size(); j++)
    {
      if(nodes[j].ID() == links[i].GetNodeID(0))
	p0 =  nodes[j].GetPosition();
      if(nodes[j].ID() == links[i].GetNodeID(1))
	p1 =  nodes[j].GetPosition();
    }
    
    printf("ObjRep::PrintGraph: link %u (%4.3f) (%u %u): %4.3f/%4.3f/%4.3f and %4.3f/%4.3f/%4.3f\n", 
	   i, links[i].GetProbability(), links[i].GetNodeID(0), links[i].GetNodeID(1), p0.x, p0.y, p0.z, p1.x, p1.y, p1.z);    
  }
}


// ************* Private **************//
/**
 * @brief Add a new link to the object representation.
 * @param link TODO TODO TODO
 */
void ObjRep::AddLink(GraphLink &link)
{
// printf("\nObjRep::AddLink: ???\n");
  Link l(link.probability);
  
  unsigned nodeID_0 = IsNode(link.node[0]);
  unsigned nodeID_1 = IsNode(link.node[1]);

// printf("ObjRep::AddLink: isNode: %u - %u\n", nodeID_0, nodeID_1);

  /// TODO Eigene Funktion: Check if link already exists
  bool isLink = false;
  if(nodeID_0 != UNDEF_ID && nodeID_1 != UNDEF_ID && nodeID_0 != nodeID_1)
  {
// printf("Link is already there: nodes are available, but link?\n");
    unsigned linkID = IsLink(nodeID_0, nodeID_1);
    if(linkID != UNDEF_ID)
    {
      isLink = true;
      links[linkID].UpdateProbability(link.probability);
      nodes[GetNodePos(nodeID_0)].UpdatePosition(link.node[0], link.probability);
      nodes[GetNodePos(nodeID_1)].UpdatePosition(link.node[1], link.probability);
// printf("Link is already there: %u\n", linkID);
    }
  }

  // we do not take the same nodes!
  if(nodeID_0 != UNDEF_ID && nodeID_0 == nodeID_1)
    return;
  
  if(!isLink)
  {
// printf("Link is not there.\n");

    if(nodeID_0 == UNDEF_ID)
    {
      Node n0(GetUniqueID(), link.node[0], link.probability);
      nodes.push_back(n0);
      l.SetNode(0, n0.ID());
// printf("New node_0: %u.\n", nodes.size()-1);
    }
    else 
    {
      nodes[GetNodePos(nodeID_0)].UpdatePosition(link.node[0], link.probability);
// printf("Set existing node0 %u.\n", nodeID_0);
      l.SetNode(0, nodeID_0);
    }
    
    if(nodeID_1 == UNDEF_ID)
    {
      Node n1(GetUniqueID(), link.node[1], link.probability);
      nodes.push_back(n1); 
      l.SetNode(1, n1.ID());
// printf("New node_1: %u.\n", nodes.size()-1);
    }
    else 
    {
      nodes[GetNodePos(nodeID_1)].UpdatePosition(link.node[1], link.probability);
      l.SetNode(1, nodeID_1);
// printf("Set existing node1 %u.\n", nodeID_1);
    }

    if(l.GetNodeID(0) == l.GetNodeID(1))
      printf("ObjRep::AddLink: error: same nodes for one link: %u.\n", l.GetNodeID(0));

    links.push_back(l);
  }
}
  
/**
 * @brief Add a new link to the object representation.
 * @param nodeID ID of the node to unset.
 */
void ObjRep::UnsetNodeDeleteFlag(unsigned nodeID)
{
  for(unsigned i=0; i<nodes.size(); i++)
    if(nodes[i].ID() == nodeID)
    {
      nodes[i].UnsetDeleteFlag();
      return;
    }
}

/**
 * @brief Delete all the marked nodes.
 */
void ObjRep::DeleteCheckedNodes()
{
  std::vector<Node>::iterator it;
  for(it=nodes.begin(); it<nodes.end(); it++)
    if((*it).GetDeleteFlag())
      nodes.erase(it--);
}

/**
 * @brief Find the unused nodes, mark and delete them.
 */
void ObjRep::DeleteUnusedNodes()
{
  for(unsigned i=0; i<nodes.size(); i++)
    nodes[i].SetDeleteFlag();
  for(unsigned i=0; i<links.size(); i++)
  {
    UnsetNodeDeleteFlag(links[i].GetNodeID(0));
    UnsetNodeDeleteFlag(links[i].GetNodeID(1));
  }
  DeleteCheckedNodes();
}


/**
 * @brief Check, if a node already exists at this position.
 * @param position Position of the new node.
 * @return Returns the ID of the found node or UNDEF_ID
 */
unsigned ObjRep::IsNode(const cv::Point3d position)
{
  bool nodeFound = false;
  double distance3D, distance2D, avDistance;
  double minDist = HUGE;
  unsigned minID = UNDEF_ID;
  
  // get the node with the smallest distances
  for(unsigned i=0; i<nodes.size(); i++)
  {
    distance3D = nodes[i].Distance3D(position);
    distance2D = nodes[i].Distance2D(position);
    avDistance = distance2D + distance3D/10.;
    if(avDistance < minDist)
    {
      minDist = avDistance;
      minID = nodes[i].ID();
      nodeFound = true;
    }
  } 
  
  if(nodeFound)
  {
    if(minDist < DISTANCE_THRESHOLD)		// TODO Distance Threshold => How can we make this "cognitive"?
      return minID;
    else 
      return UNDEF_ID;
  }
  else return UNDEF_ID;
}


/**
 * @brief New frame was processed. Update probabilities!
 */
void ObjRep::CheckLinkProbability()
{
  for(unsigned i=0; i<links.size(); i++)
    links[i].UnsetDeleteFlag();

  for(unsigned i=0; i<links.size(); i++)
    if(!links[i].DecreaseProbability(PROBABILITY_DEC_THRESHOLD))
      links[i].SetDeleteFlag();

  DeleteCheckedLinks();
}

/**
 * @brief Delete all the marked links
 */
void ObjRep::DeleteCheckedLinks()
{
  std::vector<Link>::iterator it;
  for(it=links.begin(); it<links.end(); it++)
    if((*it).GetDeleteFlag())
      links.erase(it--);
}

/**
 * @brief Check, if link between two existing nodes already exists.
 * @param node_0 First node
 * @param node_1 Second node
 * @return Return array-id, if link between the nodes already exists, otherwise UNDEF_ID.
 */
unsigned ObjRep::IsLink(unsigned nodeID_0, unsigned nodeID_1)
{
  for(unsigned i=0; i<links.size(); i++)
  {
    if((links[i].GetNodeID(0) == nodeID_0 && links[i].GetNodeID(1) == nodeID_1) || 
       (links[i].GetNodeID(1) == nodeID_0 && links[i].GetNodeID(0) == nodeID_1))
    {
      return i;
    }
  }
  return UNDEF_ID;  
}

/**
 * @brief Update a link from the object representation.
 */
void ObjRep::UpdateLink()
{
  printf("ObjRep::UpdateLink: Not yet implemented!\n");
}



  
}
