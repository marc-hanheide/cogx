//=================================================================
// Copyright (C) 2005 Geert-Jan M. Kruijff (gj@acm.org)
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation; either version 2.1 of
// the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
// 02111-1307, USA.
//=================================================================

//=================================================================
// PACKAGE DEFINITION 
//=================================================================

package comsys.processing.uttplan;

//=================================================================
// IMPORTS
//=================================================================

import comsys.datastructs.lf.*;

import java.util.*; 

//=================================================================
// JAVADOC CLASS DOCUMENTATION 
//=================================================================

/** 
 *  The class <b>UPAgenda</b> implements the agenda used in the
 *  utterance planner. One can put agenda items onto the agenda using
 *  the <i>store</i> method, and retrieve them using <i>next</i>. The
 *  agenda's size can be directly queried using <i>size</i>, and
 *  indirectly through <i>hasNext</i> (>0?) and <i>isEmpty</i> (=0?).
 *  The agenda can be cleant by calling the method <i>reset</i>. 
 * 
 *  <p> The agenda uses a HashMap with <tt>UPAgendaItem</tt> objects,
 *  indexed by system id, and a Vector with the system ids. The Vector
 *  models the queue. We use the HashMap to facilitate efficient
 *  duplicate checking.
 * 
 *  @version 050214 (started 050210)
 *  @author  Geert-Jan M. Kruijff
 */ 

//=================================================================
// CLASS DEFINITION 
//=================================================================

public class UPAgenda {
    
      
	public boolean logging = true;
	
    //=================================================================
    // CLASS-INTERNAL GLOBAL VARIABLES
    //=================================================================

    HashMap done;
    HashMap items; 
    Vector itq; 

    String queuetype = "fifo";

    //=================================================================
    // CONSTRUCTOR METHODS
    //=================================================================

    /** 
     *   The basic constructor initializes the internal variables. 
     */

    public UPAgenda () {
	items = new HashMap ();
	done  = new HashMap (); 
	itq = new Vector();
    } // end constructor
 

    //=================================================================
    // ACCESSOR METHODS
    //=================================================================

    /**
     * The method <i>addItem</i> adds the given agenda item to the
     * agenda. We are checking for duplicates, going by system id and nominal variable. 
     */

    public void addItem (UPAgendaItem it) { 
	String sysid = createID(it.getSystemId(),it.getNomvar());
	log("System to be added to agenda: ["+sysid+"]");
	if (!items.containsKey(sysid)) { 
	    items.put(sysid,it); 
	    itq.addElement(sysid);
	} else {
		log("Agenda already contains ["+sysid+"]");
	} // end if check for duplicates
    } // end addItem

    /**
     * The method <i>addItems</i> adds the agenda items in the HashMap
     * object to the agenda.
     */

    public void addItems (Vector itv) {
	log("Adding Items");
	for (Iterator iIter = itv.iterator(); iIter.hasNext(); ) { 
	    UPAgendaItem it = (UPAgendaItem) iIter.next();
		// String sysid = it.getSystemId(); 
		String sysid = createID(it.getSystemId(),it.getNomvar());  //-- not a good idea --> cycle
		// log("Agenda item: ["+it+"]");
		// log("System id: ["+sysid+"]");
		// log("System id+nomvar: ["+sysid+"+"+it.getNomvar()+"]");
		    if (!items.containsKey(sysid)) { 
			items.put(sysid,it); 
			itq.addElement(sysid);
			log("Added system ["+sysid+"] for item ["+it+"] to agenda");
	    } // end if check for duplicates
	} // end for over items in the vector
    } // end addItems

    /**
     * The method <i>getQueueType</i> returns the type of queue that
     * the agenda implements: <tt>fifo</tt> or <tt>lifo</tt>. 
     */ 

    public String getQueueType () { return queuetype; } 


    /**
     * The method <i>hasItem</i> checks whether the agenda contains an
     * item for the given system ID and locus nominal. 
     */

    public boolean hasItem (String sysid, String nomvar) { 
	String sysID = createID(sysid,nomvar);
	if (items.containsKey(sysID)) { 
	    UPAgendaItem it = (UPAgendaItem) items.get(sysID);
	    if (it.getNomvar().equals(nomvar)) { 
		return true; 
	    } else {
		return false;
	    } // end if..else check whether item for locus
	} else {
	    return false;
	} // end if..else check whether there
    } // end hasItem

    /**
     * The method <i>hasDoneItem</i> checks whether the agenda already
     * did an item for the given system ID and locus nominal. 
     */

    public boolean hasDoneItem (String sysid, String nomvar) { 
	String sysID = createID(sysid,nomvar);	
	if (done.containsKey(sysID)) { 
	    UPAgendaItem it = (UPAgendaItem) done.get(sysID);
	    if (it.getNomvar().equals(nomvar)) { 
		return true; 
	    } else {
		return false;
	    } // end if..else check whether item for locus
	} else {
	    return false;
	} // end if..else check whether there
    } // end hasItem
    



    /**
     * The method <i>hasNext</i> returns a boolean indicating whether
     * there are still items on the agenda.
     */

    public boolean hasNext () { return (items.size() > 0); } 

    /**
     * The method <i>isEmpty</i> returns a boolean indicating whether
     * the agenda is empty.
     */

    public boolean isEmpty () { return items.isEmpty(); }

    /**
     * The method <i>next</i> retrieves the next item on the
     * agenda. If there is no next item, <tt>null</tt> is returned.
     */ 

    public UPAgendaItem next () { 
	UPAgendaItem item = null;
	if (!itq.isEmpty()) { 
	    String itemid = "";
	    if (queuetype.equals("fifo")) { 
		itemid = (String) itq.firstElement();  
		item =  (UPAgendaItem)items.get(itemid); 
	    } // end if fifo
	    if (queuetype.equals("lifo")) { 
		itemid = (String) itq.lastElement(); 
		item =  (UPAgendaItem)items.get(itemid); 
	    } // end if lifo  
	    // remove the elements from the queue and the map
	    items.remove(itemid); 
	    itq.remove(itemid);
	    // add the item to the done map
	    done.put(itemid,item);
	} // end if.. check whether there are items
	return item; 
    } // end 

    /**
     * The method <i>reset</i> resets the agenda to an empty structure.
     */

    public void reset () { 
	done  = new HashMap ();
	items = new HashMap (); 
	itq = new Vector (); 
    } // end reset 
 
    /**
     * The method <i>setQueueType</i> sets the type of queue the
     * agenda implements. Valued types are <tt>fifo</tt> and
     * <tt>lifo</tt>; <tt>fifo</tt> is the default.
     */ 

    public void setQueueType (String t) { 
	if (t.equals("fifo")) { queuetype = "fifo"; }
	if (t.equals("lifo")) { queuetype = "lifo"; }
    } // end setQueueType

    /**
     * The method <i>size</i> returns the size of the agenda. 
     */

    public int size () { return items.size(); } 



	public String createID (String systemId, String nv) { 
	  return systemId+"."+nv;
	} // end createID

    //=================================================================
    // I/O METHODS
    //=================================================================

    public String toString () { 
	String result = "\nAGENDA:\n";
	for (Iterator iIter=itq.iterator(); iIter.hasNext(); ) { 
	    String itemid = (String) iIter.next();
	    UPAgendaItem item = (UPAgendaItem) items.get(itemid);
	    result = result + item.toString()+"\n";
	} // end for over items
	return result;
    } // end toString


    private void log (String str) {
    	if (logging) {
    		// System.out.println("[UP Agenda] " + str);
    	}
    }
    //=================================================================
    // MAIN METHOD
    //=================================================================


} // end class definition 
