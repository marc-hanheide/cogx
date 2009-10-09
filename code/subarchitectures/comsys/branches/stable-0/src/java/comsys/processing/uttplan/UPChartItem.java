//=================================================================
//Copyright (C) 2005 Geert-Jan M. Kruijff, Maria Staudte (gj@acm.org)
//
//This library is free software; you can redistribute it and/or
//modify it under the terms of the GNU Lesser General Public License
//as published by the Free Software Foundation; either version 2.1 of
//the License, or (at your option) any later version.
//
//This library is distributed in the hope that it will be useful, but
//WITHOUT ANY WARRANTY; without even the implied warranty of
//MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//Lesser General Public License for more details.
//
//You should have received a copy of the GNU Lesser General Public
//License along with this program; if not, write to the Free Software
//Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
//02111-1307, USA.
//=================================================================

//=================================================================
//PACKAGE DEFINITION 
//=================================================================

package comsys.processing.uttplan;

//=================================================================
// IMPORTS
//=================================================================

import comsys.datastructs.lf.*;

import java.util.*;

//=================================================================
//JAVADOC CLASS DOCUMENTATION 
//=================================================================

/** 
*  The class <b>UPChartItem</b> implements the datastructure for
*  items that can appear on the chart used in the utterance
*  planner. 
* 
*/ 

//=================================================================
//CLASS DEFINITION 
//=================================================================

public class UPChartItem {
 
   
 //=================================================================
 // CLASS-INTERNAL GLOBAL VARIABLES
 //=================================================================

 private LinkedList list;
  

 //=================================================================
 // CONSTRUCTOR METHODS
 //=================================================================

 /** 
  *  The basic constructor initializes the internal variables
  */
 public UPChartItem () {
	list = new LinkedList();
 }
 

 //=================================================================
 // ACCESSOR METHODS
 //=================================================================

 /** add a Logical Form to the Chart Cell
  */ 
 public void add (LFNominal lf) { list.add(lf); }

 
 /** get the LogicalForm at position i of the Chart Cell
  */
 public LFNominal get (int i)  { return (LFNominal)list.get(i); }

 
 /** query the Chart Cell for whether it already contains a given LogicalForm
  */ 
 public boolean contains (LFNominal lf) { return list.contains(lf); }
 
 
 /** query the size/length of the Chart Cell
  */
 public int size () { return list.size(); }


 //=================================================================
 // I/O METHODS
 //=================================================================

 public String toString () { 
 	String s = "";
 	for (int i = 0; i<list.size(); i++) s = s + list.get(i).toString() + "\n";
 	if (s.length()==0) return "Unfortunately Chart-Item is empty !";
	else {return "CHART_ITEM : " + "\n" +s;} 
 } // end toString

 //=================================================================
 // MAIN METHOD
 //=================================================================


} // end class definition 
