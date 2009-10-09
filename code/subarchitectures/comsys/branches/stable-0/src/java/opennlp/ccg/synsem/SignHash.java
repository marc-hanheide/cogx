///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2003-5 Jason Baldridge, Gann Bierner and 
//                      University of Edinburgh (Michael White)
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

package opennlp.ccg.synsem;

import gnu.trove.*;
import java.util.*;

/**
 * A set of signs, unique up to surface words.
 * Signs with lower derivational complexity are kept during insertion.
 *
 * @author      Jason Baldridge
 * @author      Gann Bierner
 * @author      Michael White
 * @version     $Revision: 1.10 $, $Date: 2005/11/09 22:17:00 $
 */
public class SignHash extends THashSet {

	private static final long serialVersionUID = 1L;
	
	/** Hashing strategy that uses Sign's surfaceWordHashCode and surfaceWordEquals methods. */
    protected static TObjectHashingStrategy surfaceWordHashingStrategy = new TObjectHashingStrategy() {
		private static final long serialVersionUID = 1L;
		public int computeHashCode(java.lang.Object o) {
            return ((Sign)o).surfaceWordHashCode();
        }
        public boolean equals(java.lang.Object o1, java.lang.Object o2) {
            return ((Sign)o1).surfaceWordEquals((Sign)o2);
        }
    };

    /** Default constructor. */
    public SignHash() { super(surfaceWordHashingStrategy); }

    /**
     * Constructor which adds one sign.
     */
    public SignHash(Sign sign) {
        this(); insert(sign);
    }

    /**
     * Constructor which adds a collection of signs.
     */
    public SignHash(Collection c) {
        this();
        for (Iterator i = c.iterator(); i.hasNext();) {
            insert((Sign)i.next());
        }
    }
    
    /**
     * Returns this as a set of signs.
     */
    @SuppressWarnings("unchecked")
	public Set<Sign> asSignSet() { return (Set<Sign>) this; }

    /**
     * Adds a sign, keeping the one with lower derivational complexity 
     * if there is an equivalent one there already; returns the old 
     * sign if it was displaced, the new sign if there was no equivalent 
     * old sign, or null if the sign was not actually added.
     */
    public Sign insert(Sign sign) {
        int pos = index(sign);
        if (pos >= 0) {
            Sign oldSign = (Sign) _set[pos];
            if (oldSign == sign) return null;
            int complexity = sign.getDerivationHistory().complexity();
            int oldComplexity = oldSign.getDerivationHistory().complexity();
            if (complexity < oldComplexity) {
            	_set[pos] = sign; return oldSign;
            }
            else return null;
        }
        else {
        	add(sign); return sign;
        }
    }
}
