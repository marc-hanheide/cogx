///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2008 Michael White
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

package opennlp.ccg.realize;

import opennlp.ccg.lexicon.*;
import opennlp.ccg.hylo.*;
import java.util.*;

/**
 * A hypertagger is a realization supertagger.  It must extend the 
 * SupertaggerAdapter interface for plugging a supertagger into the 
 * lexicon.  It returns beta-best categories for lexical lookup 
 * according to a sequence of beta settings it maintains internally.
 * 
 * @author      Michael White
 * @version     $Revision: 1.1 $, $Date: 2008/01/05 22:45:13 $
 */
//NB: It might make sense to promote the beta-related methods to the 
//    SupertaggerAdapter interface. 
public interface Hypertagger extends SupertaggerAdapter {
	
	/**
	 * Maps the given elementary predications to their predicted categories, 
	 * so that the beta-best categories can be returned by calls to setPred
	 * and getFamilyNames. 
	 */
	public void mapPreds(List<SatOp> preds);
	
	/**
	 * Sets the current elementary predication to the one with the given index, 
	 * so that the beta-best categories for it can be returned by a call to 
	 * getFamilyNames.
	 */
	public void setPred(int index);
	
	/**
	 * Resets beta to the most restrictive value.
	 */
	public void resetBeta();

	/**
	 * Advances beta to the next most restrictive setting.
	 */
	public void nextBeta();
	
	/**
	 * Returns whether there are any less restrictive beta settings
	 * remaining in the sequence.
	 */
	public boolean hasMoreBetas();
}
