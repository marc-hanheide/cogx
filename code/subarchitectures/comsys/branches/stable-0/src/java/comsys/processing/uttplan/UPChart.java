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
*  The class <b>UPChart</b> implements the chart used in the
*  utterance planner. One can put chart items onto the chart using
*  the <i>store</i> method, and retrieve them using <i>next</i>. The
*  chart's size can be directly queried using <i>size</i>, and
*  indirectly through <i>hasNext</i> (>0?) and <i>isEmpty</i> (=0?).
*  The chart can be cleant by calling the method <i>reset</i>. 
* 
*  <p> The agenda uses a HashMap with <tt>UPChartItem</tt> objects,
*  ...
* 
*/ 

//=================================================================
//CLASS DEFINITION 
//=================================================================

 public class UPChart {
 
   
 //=================================================================
 // CLASS-INTERNAL GLOBAL VARIABLES
 //=================================================================

 HashMap items; 
 int count;
 int retrieve;
 
 //=================================================================
 // CONSTRUCTOR METHODS
 //=================================================================

 /** 
  *   The basic constructor initializes the internal variables. 
  */
 public UPChart () {
	items = new HashMap ();
	count = 0;
	retrieve = 0;
 } // end constructor


 //=================================================================
 // ACCESSOR METHODS
 //=================================================================

 /** The method <i>newItem</i> xpands the chart-hashmap with one more
  *  UPChartItem
  */
 public void newItem () {
 	UPChartItem upchart = new UPChartItem();
 	Integer c = new Integer(count);
 	items.put(c,upchart);
 	count++;
 }
 
 
 /**
  * The method <i>addItem</i> adds the given chart item to the
  * chart.
  */
 public void addItem (UPChartItem it) {
 	Integer c = new Integer(count);
	if (!items.containsValue(it)) { 
	    items.put(c,it); 
	} // end if check for duplicates
	count++;
 } // end addItem
 
 
 
 /** Checking whether Chart is empty
  * 
  * @return boolean
  */
 public boolean isEmpty () { return items.isEmpty(); }


 /**
  * The method <i>next</i> retrieves the next item on the
  * chart. If there is no next item, <tt>null</tt> is returned.
  */ 
 public UPChartItem getItem(int i) { 
	if (retrieve > count) {
		return null;	
	} else {
		Integer c = new Integer(i);
		return (UPChartItem)items.get(c);
	}
 } // end next 

 
 /**
  * The method <i>next</i> retrieves the next item on the
  * chart. If there is no next item, <tt>null</tt> is returned.
  */ 
 public UPChartItem getCurrent() { 
	if (retrieve > count) {
		return null;	
	} else {
		Integer c = new Integer(retrieve);
		return (UPChartItem)items.get(c);
	}
 } // end next
 
 
 /**
  * The method <i>next</i> retrieves the next item on the
  * chart. If there is no next item, <tt>null</tt> is returned.
  */ 
 public UPChartItem next () { 
	if (retrieve > count) {
		return null;	
	} else {
		Integer c = new Integer(retrieve);
		retrieve++;
		return (UPChartItem)items.get(c);
	}
 } // end next 

 
//=================================================================
 // I/O METHODS
 //=================================================================

 public String toString () { 
	String result = "\nCHART:\n";
	for (int i=0;i<count;i++) { 
		Integer c = new Integer(i);
		System.out.println("in toString() of Chart");
	    UPChartItem item = (UPChartItem) items.get(c);
	    result = result + "\n" + i + ". " + item.toString()+"\n";
	} // end for over items
	return result;
 } // end toString


 //=================================================================
 // MAIN METHOD
 //=================================================================


} // end class definition 
 
