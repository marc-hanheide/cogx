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

package opennlp.ccg.lexicon;

import java.util.*;

/**
 * The SupertaggerAdapter interface is for plugging a supertagger into the 
 * lexicon in order to return only the desired, high probability categories 
 * during lexical lookup.  Once the supertagger has been plugged in, 
 * using Lexicon.setSupertagger, the supertagger will be consulted during 
 * each lexical lookup for the desired categories, using getSupertags 
 * to return the set of categories by their supertags (derived from family names).  
 * Note that this entails that the supertagger must update its state between lexical 
 * lookup calls; in this way, identical words in a sentence can have different 
 * predicted categories.  Also, the set of categories (ie, families) can 
 * vary according to the morph item, whose POS and stem are available through
 * the item's full word (via MorphItem.getWord).
 * 
 * At present, the lexicon must contain appropriate morph items for all words. 
 * However, the supertags assigned (via family names) to a word need not be 
 * limited to those explicitly listed in the lexicon.  When there is an explicit 
 * entry, it will be used, as doing so allows the specification of a 'pred' 
 * which differs from the stem.  Otherwise, when using a supertagger, it is no 
 * longer necessary to list stems with categories in the lexicon, as the supertagger 
 * becomes responsible for this mapping.
 *
 * Note also that at present, supertaggers for parsing and realization are not 
 * distinguished, and only one may be plugged in to the lexicon at a time.
 *  
 * @author      Michael White
 * @version     $Revision: 1.3 $, $Date: 2008/01/06 20:31:45 $
 */
public interface SupertaggerAdapter {
	
	/**
	 * Returns the supertags of the desired categories for the current lexical lookup
	 * (or null to accept all). 
	 */
	public Set<String> getSupertags(MorphItem morphItem);
}
