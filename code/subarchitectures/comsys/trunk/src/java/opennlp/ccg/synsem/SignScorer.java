///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2003-4 University of Edinburgh (Michael White)
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


/**
 * Interface for sign scoring models.
 *
 * @author      Michael White
 * @version     $Revision: 1.1 $, $Date: 2007/12/21 05:13:37 $
 */
public interface SignScorer
{
    /** 
     * Returns a score between 0 (worst) and 1 (best) for the given sign 
     * and completeness flag.
     */
    public double score(Sign sign, boolean complete);

    /** A scorer that returns 0 for all signs. */
    public static SignScorer nullScorer = new SignScorer() { 
        public double score(Sign sign, boolean complete) { return 0; }
    };
}

