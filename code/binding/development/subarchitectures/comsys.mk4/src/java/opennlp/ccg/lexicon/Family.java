///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2002-8 Jason Baldridge, Gann Bierner and 
//                      University of Edinburgh / Michael White
// 
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
// 
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public
// License along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
//////////////////////////////////////////////////////////////////////////////

package opennlp.ccg.lexicon;

import org.jdom.*;
import java.util.*;
import java.util.regex.*;

/**
 * Lexicon category family.
 *
 * @author      Jason Baldridge
 * @author      Gann Bierner
 * @author      Michael White
 * @version     $Revision: 1.8 $, $Date: 2008/04/12 20:29:35 $
 */
public class Family {
	
    private String name = "";
    private String supertag = "";
    private Boolean closed = Boolean.FALSE;
    private String pos = "";
    private String indexRel = ""; 
    private String coartRel = ""; 
    private DataItem[] data;
    private EntriesItem[] entries;

    public Family(Element famel) {
    	
        setName(famel.getAttributeValue("name"));
        pos = famel.getAttributeValue("pos");
        
        String isClosed = famel.getAttributeValue("closed");
        if (isClosed != null && isClosed.equals("true")) {
            setClosed(Boolean.TRUE);
        }

        String indexRelVal = famel.getAttributeValue("indexRel");
        if (indexRelVal != null) { indexRel = indexRelVal; }

        String coartRelVal = famel.getAttributeValue("coartRel");
        if (coartRelVal != null) { coartRel = coartRelVal; }

        List entriesList = famel.getChildren("entry");
        entries = new EntriesItem[entriesList.size()];
        for (int j=0; j < entriesList.size(); j++) {
            entries[j] = new EntriesItem((Element)entriesList.get(j), this);
        }
        
        List members = famel.getChildren("member");
        data = new DataItem[members.size()];
        for (int j=0; j < members.size(); j++) {
            data[j] = new DataItem((Element)members.get(j));
        }
    }

    public Family(String s) { setName(s); }

    public boolean isClosed() { return closed.booleanValue(); }
    
    public void setName(String s) { name = s; setSupertag(); }
    public void setClosed(Boolean b) { closed = b; }
    public void setPOS(String s) { pos = s; }
    public void setIndexRel(String s) { indexRel = s; }
    public void setCoartRel(String s) { coartRel = s; }
    public void setData(DataItem[] dm) { data = dm; }
    public void setEntries(EntriesItem[] em) { entries = em; }

    public String getName() { return name; }
    public String getSupertag() { return supertag; }
    public Boolean getClosed() { return closed; }
    public String getPOS() { return pos; }
    public String getIndexRel() { return indexRel; }
    public String getCoartRel() { return coartRel; }
    public DataItem[] getData() { return data; }
    public EntriesItem[] getEntries() { return entries; }
    
    // sets the supertag to the name minus any underscored coindexations and slash modes
    private void setSupertag() { supertag = deriveSupertag(name); }
    
    /** Pattern for removing supertag extras. */
    public static Pattern extrasPattern = Pattern.compile("_~?\\d+|[*><\\^]");
    
    /** 
     * Derives a supertag name from a cat name by removing underscored coindexations and slash modes.
     * Also removes semantic part following a colon (where the colon is not followed by a 
     * right square bracket). 
     */
    public static String deriveSupertag(String cat) {
    	int colonPos = cat.indexOf(":");
    	int rsbPos = cat.indexOf("]", colonPos);
    	while (colonPos > 0 && rsbPos > 0) {
    		colonPos = cat.indexOf(":", rsbPos);
    		rsbPos = cat.indexOf("]", colonPos);
    	}
    	if (colonPos > 0) cat = cat.substring(0, colonPos);
    	return extrasPattern.matcher(cat).replaceAll("");
    }
}
