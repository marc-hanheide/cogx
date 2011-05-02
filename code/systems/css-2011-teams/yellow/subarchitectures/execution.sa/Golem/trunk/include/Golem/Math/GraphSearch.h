/** @file GraphSearch.h
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#pragma once
#ifndef _GOLEM_MATH_GRAPHSEARCH_H_
#define _GOLEM_MATH_GRAPHSEARCH_H_

//------------------------------------------------------------------------------

#include <Golem/Defs/Pointers.h>
#include <Golem/Math/Math.h>
#include <Golem/Math/Node.h>
#include <iterator>
#include <vector>
#include <set>
#include <algorithm>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Abstract class implementing a generic A* graph search algorithm.
*
*	A* graph search algorithm searches for the minimum cost path between root and goal nodes.
*	By overriding pure abstract findCost(), nextNode() and other virtual functions
*	from protected part of the class, one can obtain eg. breath-first or depth-first search
*	strategies, as well as different memory management strategies for finite or infinite
*	(say very large) number of nodes.
*	GraphSearch has a nearly optimal design - the only penalty (negligible in practise) are virtual
*	function calls. GraphSearch uses some extra improvements like efficient node cost sorting.
*
* @see "Artificial Intelligence: A new synthesis", Nils J. Nilsson, 1998, Morgan Kaufmann
*/
class GraphSearch {
protected:
	/** Sorted set with no more than ~2log2(n+1) extra complexity (red-black tree) */
	typedef std::set<Node, Node::cost_less> NodeRank;

	U32 _capacity;
	U32 _size;
	U32 nextNodeIndex;

	shared_ptr<Real, arr_cnt<Real> > costMat;
	shared_ptr<U32, arr_cnt<U32> > openVec;
	shared_ptr<U32, arr_cnt<U32> > closedVec;

	/** Default implementation of cost map helper functions, i E <0, size - 1> */
	virtual inline Real getCost(U32 i) {
		return costMat[i];
	}
	virtual inline void setCost(U32 i, Real c) {
		costMat[i] = c;
	}
	/** Initialisation of cost matrix. */
	virtual void initCost(U32 capacity, U32 size);

	/** Default implementation of open nodes helper functions, i E <0, size - 1> */
	virtual inline U32 getOpen(U32 i) {
		return openVec[i];
	}
	virtual inline void setOpen(U32 i, U32 j) {
		openVec[i] = j;
	}
	/** Initialisation of open nodes vector. */
	virtual void initOpen(U32 capacity, U32 size);

	/** Default implementation of closed nodes helper functions, i E <0, size - 1> */
	virtual inline U32 getClosed(U32 i) {
		return closedVec[i];
	}
	virtual inline void setClosed(U32 i, U32 j) {
		closedVec[i] = j;
	}
	/** Initialisation of closed nodes vector */
	virtual void initClosed(U32 capacity, U32 size);

	/** Default implementation of node expansion function iterates through all nodes.
	*
	*	@param	i		node to be expanded, i E <0, size - 1>
	*	@return			destination node directly reacheable from node i, or
	*					IDX_UINI to finish expansion
	*/
	virtual inline U32 nextNode(U32 i) {
		if (++nextNodeIndex >= _size)
			nextNodeIndex = Node::IDX_UINI;
		return nextNodeIndex;
	}

	/** Declaration of the graph heuristic function.
	*
	*	Graph heuristic function is defined as f() := g() + h(), where
	*	h() is the estimated cost of reaching the goal node,
	*	g() is the relative cost of reaching node j from i
	*	The root and the goal nodes have a priori specified indices IDX_ROOT and IDX_GOAL.
	*
	*	@param	i		current node, j E <0, size - 1>
	*	@param	j		destination node, i E <0, size - 1>
	*	@return			<COST_ZERO, COST_INF> estimated cost of reaching the goal node from j
	*/
	virtual Real cost(U32 i, U32 j) const = 0;

	/** Collision detection function
	*/
	virtual bool collides(U32 i, U32 j) const = 0;
	
	/** node cost rank */
	NodeRank nodeRank;

public:
	/**	Creates GraphSearch and initialises basic graph data structures.
	*	@param	capacity	maximal size of the graph (number of nodes).
	*/
	GraphSearch();
	
	/** Virtual destructor */
	virtual ~GraphSearch();

	/** Search algorithm finds the minimum cost path between root and goal nodes.
	*
	*	The first and the last element of the returned sequence are suitably:
	*	- the root node
	*	- a node from which the goal node is directly reacheable
	*	The returned pointer is NULL if there is no finite-cost route to the goal node.
	*
	*	@param	seq		collection of nodes of the graph
	*	@param	iter	iterator locating the insertion point
	*	@return			iterator locating the first element of the path
	*/
	template <class Seq, class Iter> Iter find(Seq& seq, Iter iter) {
		ASSERT(_size >= 2)
		// the current node.
		U32 nCurrent = Node::IDX_ROOT;
		// the node through which passes the best path to nCurrent.
		U32 rCurrent = Node::IDX_ROOT;
		// a cost of reaching nCurrent.
		Real cCurrent = Node::COST_ZERO;
		// clear node cost rank
		nodeRank.clear();

		for (;;) {
			// The best path to nCurrent goes through rCurrent.
			setClosed(nCurrent, rCurrent);
		
			// Break the loop if nCurrent is the goal.
			if (nCurrent == Node::IDX_GOAL)
				break;
			
			// then remove nCurrent from open.
			setOpen(nCurrent, Node::IDX_UINI);
			
			for (;;) {
				// Expand the current node nCurrent,
				// find for all possible destination nodes j
				U32 j = nextNode(nCurrent);

				// Break if there are no more destination nodes
				if (j == Node::IDX_UINI)
					break;
				// Do not expand if the destination node j is:
				// equal the expanded node nCurrent, or is already on close.
				if (j == nCurrent || getClosed(j) != Node::IDX_UINI)
					continue;

				// Calculate the cost of reaching j through nCurrent.
				Real cost = this->cost(nCurrent, j);

				// if j is reachable
				if (cost < Node::COST_INF) {
					// Calculate the total cost
					cost += cCurrent;

					// Extract a node, through which passes the best path to j.
					U32 k = getOpen(j);
					// Redirect the best path to j if there was no
					// path to j before, or the new path is less costly.
					if (k == Node::IDX_UINI) {
						if (this->collides(nCurrent, j))
							continue;
					}
					else {
						Real c = getCost(j);
						if (c <= cost || this->collides(nCurrent, j))
							continue;
						nodeRank.erase(Node(j, c));
					}
				
					// the best path to j passes nCurrent
					setOpen(j, nCurrent);
					// Update cost map and rank
					setCost(j, cost);
					nodeRank.insert(Node(j, cost));
				}
			}

			// Get a node nCurrent with the lowest cost value.
			NodeRank::iterator node = nodeRank.begin();
			// Break the loop if nodeRank/open is empty - goal node is unreacheable.
			if (node == nodeRank.end())
				return iter;

			nCurrent = node->index;
			rCurrent = getOpen(nCurrent);
			cCurrent = getCost(nCurrent);

			// remove nCurrent from nodeRank
			nodeRank.erase(node);
		}

		// Build the outcoming path as an array of wpNum2 waypoints by
		// simple iteration close(n) -> n until n points the root.
		ASSERT(nCurrent == Node::IDX_GOAL)

		// Goal node: put elements in reverse order, requires a suitable (implicit) copy constructor!
		iter = seq.insert(iter, Node(nCurrent, Node::COST_ZERO));

		for (;;) {
			rCurrent = getClosed(nCurrent);

			// Other nodes: put elements in reverse order, requires a suitable (implicit) copy constructor!
			iter = seq.insert(iter, Node(rCurrent, getCost(nCurrent)));
			
			// break if the root node has been reached
			if (rCurrent == Node::IDX_ROOT)
				break;

			nCurrent = rCurrent;
		}

		return iter;
	}

	/** Sets number of nodes 
	*	@param	size	current size of the graph (number of nodes).
	*/
	void resize(U32 size);

	/** Current number of nodes */
	inline U32 size() const {
		return _size;
	}

	/** Sets maximum number of nodes
	*	@param	capacity	maximal size of the graph (number of nodes).
	*/
	void reserve(U32 capacity);

	/** Maximum number of nodes */
	inline U32 capacity() const {
		return _capacity;
	}
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_MATH_GRAPHSEARCH_H_*/
