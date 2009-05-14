//=================================================================
// Copyright (C) 2007 Geert-Jan M. Kruijff (gj@dfki.de)
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

package org.cognitivesystems.comsys.monitors;

import java.util.*;

public class PendingProxyRelation {

	public String headNomVar = "";
	public String depNomVar  = "";
	public String relMode    = "";
	public String tempFrameType = "ASSERTED";
	public TreeSet contentStatus = new TreeSet();

	public PendingProxyRelation ()  {}

	public PendingProxyRelation (String h, String d, String m) { 
		headNomVar = h; 
		depNomVar = d;
		relMode = m;
	} // end constructor

	public PendingProxyRelation (String h, String d, String m, String tempFrameType) { 
		headNomVar = h; 
		depNomVar = d;
		relMode = m;
		this.tempFrameType = tempFrameType;
	} // end constructor
	
	public void addContentStatus (String status) { 
		contentStatus.add(status);
	} // end addContentStatus
	
	public Iterator<String> getContentStatus () { 
		return contentStatus.iterator();
	} // end getContentStatus
	
	public boolean hasContentStatus (String status) { 
		return contentStatus.contains(status);
	} // end hasContentStatus
	
	public boolean isSetContentStatus () { 
		return (contentStatus.size() > 0); 
	} 	
	
	public String toString () { 
		return "Relation of type ["+relMode+"] from ["+headNomVar+"] to ["+depNomVar+"], with status "+contentStatus.toString();
	} 

} // end class
