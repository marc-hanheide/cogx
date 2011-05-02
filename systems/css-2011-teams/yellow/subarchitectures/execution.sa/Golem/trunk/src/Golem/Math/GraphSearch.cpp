/** @file GraphSearch.cpp
 * 
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#include <Golem/Math/GraphSearch.h>
#include <memory.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

const Real Node::COST_ZERO = REAL_ZERO;
const Real Node::COST_INF = numeric_const<Real>::MAX;

//------------------------------------------------------------------------------

GraphSearch::GraphSearch() :
	_capacity(0), _size(0), nextNodeIndex(Node::IDX_UINI)
{
}

GraphSearch::~GraphSearch()
{
}

void GraphSearch::resize(U32 size) {
	ASSERT(size >= 2)

	U32 capacity = this->_capacity < size ? size : this->_capacity;
	
	initCost(capacity, size);
	initOpen(capacity, size);
	initClosed(capacity, size);

	this->_capacity = capacity;
	this->_size = size;
}

void GraphSearch::reserve(U32 capacity) {
	ASSERT(capacity >= 2)
	
	U32 size = this->_size > capacity ? capacity : this->_size;

	initCost(capacity, size);
	initOpen(capacity, size);
	initClosed(capacity, size);

	this->_capacity = capacity;
	this->_size = size;
}

void GraphSearch::initCost(U32 capacity, U32 size) {
	if (this->_capacity != capacity)
		costMat.reset(new Real [capacity]);
	
	ASSERT(costMat != NULL)
	
	//U32 i = size;
	//while (i > 0) costMat[--i] = Node::COST_ZERO;
	::memset(costMat.get(), 0, size);
}

void GraphSearch::initOpen(U32 capacity, U32 size) {
	if (this->_capacity != capacity)
		openVec.reset(new U32 [capacity]);

	ASSERT(openVec != NULL)

	U32 i = size;
	while (i > 0) openVec[--i] = Node::IDX_UINI;
}

void GraphSearch::initClosed(U32 capacity, U32 size) {
	if (this->_capacity != capacity)
		closedVec.reset(new U32 [capacity]);

	ASSERT(closedVec != NULL)

	U32 i = size;
	while (i > 0) closedVec[--i] = Node::IDX_UINI;
}

//------------------------------------------------------------------------------
