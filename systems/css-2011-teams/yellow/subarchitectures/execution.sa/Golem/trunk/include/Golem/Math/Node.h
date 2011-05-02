/** @file Node.h
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#pragma once
#ifndef _GOLEM_MATH_NODE_H_
#define _GOLEM_MATH_NODE_H_

//------------------------------------------------------------------------------

#include <Golem/Math/Math.h>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

/** Node base class.
*/
class Node {
public:
	/** Goal node index */
	static const U32 IDX_GOAL = 0;
	/** Root node index */
	static const U32 IDX_ROOT = 1;
	/** Uninitialised node index */
	static const U32 IDX_UINI = numeric_const<U32>::MAX;

	/** Zero/uninitialised cost */
	static const Real COST_ZERO;
	/** Infinite cost (node unreachable) */
	static const Real COST_INF;
	
	/** Index comparator */
	struct index_less {
		inline bool operator () (const Node &left, const Node &right) const {
			return left.index < right.index;
		}
	};
	
	/** Cost comparator */
	struct cost_less {
		inline bool operator () (const Node &left, const Node &right) const {
			return left.cost < right.cost;
		}
	};
	
	/** Node index, index E <0, size - 1> */
	U32 index;
	/** (Minimal) cost of reaching the goal node, cost E <COST_ZERO, COST_INF> */
	Real cost;

	Node(U32 index = IDX_UINI, Real cost = COST_ZERO) : index(index), cost(cost) {
	}
	Node(const Node& node) : index(node.index), cost(node.cost) {
	}

	inline bool operator < (const Node &right) const {
		return index < right.index;
	}
};

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_MATH_NODE_H_*/
