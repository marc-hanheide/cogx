/**
 * @file Edge.h
 * @author Andreas Richtsfeld
 * @date July 2011
 * @version 0.1
 * @brief Edge definitions for the graph.
 */

#ifndef E_EDGE_H
#define E_EDGE_H

namespace E
{

 
struct Edge
{
  int type;   /// type of node
  float w;    /// weightning factor
  int a, b;   /// node numbers
};

}

#endif

