///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2002-5 Jason Baldridge and University of Edinburgh (Michael White)
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

import opennlp.ccg.unify.*;
import gnu.trove.*;

/**
 * An interface for objects that can be arguments in a category.
 *
 * @author      Jason Baldridge
 * @author      Michael White
 * @version     $Revision: 1.6 $, $Date: 2005/10/18 22:20:15 $
 */
public interface Arg extends Unifiable, Mutable {

    public Arg copy();
    public void forall(CategoryFcn fcn);
    public void unifySlash(Slash s) throws UnifyFailure;
    
    /**
     * Returns a hash code for this arg, 
     * using the given map from vars to ints.
     */
    public int hashCode(TObjectIntHashMap varMap);
    
    /**
     * Returns whether this arg equals the given object  
     * up to variable names, using the given maps from vars to ints.
     */
    public boolean equals(Object obj, TObjectIntHashMap varMap, TObjectIntHashMap varMap2);
    
    /**
     * Returns the supertag for the arg.
     */
    public String getSupertag();
    
    /**
     * Returns a TeX-formatted string representation for the arg.
     */
    public String toTeX();
}


