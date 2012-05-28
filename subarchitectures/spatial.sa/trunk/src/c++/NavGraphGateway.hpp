//
// = FILENAME
//    NavGraphGateway.hh
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

#ifndef NavGraphGateway_hh
#define NavGraphGateway_hh

#include "Navigation/NavGraphNode.hh"

namespace Cure {

/**
 * Special type of node defining gateways such as a door in the graph
 * 
 * @author Patric Jensfelt 
 * @see
 */
class NavGraphGateway: public NavGraphNode {
	friend class NewNavGraph;

public:
	enum DoorState {
		UNKNOWN = 0, OPEN, CLOSED
	};

public:
	NavGraphGateway();

	/**
	 * Constructor
	 *
	 * @param name name of the gateway
	 * @param id id for the node
	 * @param x x-coordinate of the gateway
	 * @param y y-coordinate of the gateway
	 * @param a orientation of the gateway
	 * @param maxSpeed max speed near this node (not used right now)
	 * @param width widht of the gateway
	 */
	NavGraphGateway(const std::string &name, int id, double x, double y,
			double a, double maxSpeed, double width);

	/**
	 * Copy constructor
	 *
	 * @param src graph to copy
	 */
	NavGraphGateway(const NavGraphGateway &src);

	/**
	 * Destructor
	 */
	~NavGraphGateway();

	/**
	 * Use this function to get a copy of this objects
	 */
	virtual NavGraphNode* getNodeCopy();

	/**
	 * @return width of this gateway [m]
	 */
	double getWidth() const {
		return m_Width;
	}

	/// @return the state of the door if this is a door
	int getDoorState() const {
		return m_DoorState;
	}

	/// Set the state of the door
	void setDoorState(int s) {
		m_DoorState = s;
	}

protected:
	/// Width of the gateway [m]
	double m_Width;

	/// The state of the door if this is a door
	int m_DoorState;
};

}
; // namespace Cure

#endif // NavGraphGateway_hh
