///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2004 University of Edinburgh (Michael White)
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

package opennlp.ccg.ngrams;

import opennlp.ccg.synsem.Sign;
import opennlp.ccg.synsem.SignScorer;

// import java.util.*;

/**
 * Product of sign scorers.
 *
 * @author      Michael White
 * @version     $Revision: 1.3 $, $Date: 2007/12/21 05:13:37 $
 */
public class SignScorerProduct implements SignScorer
{
    /** The component models. */
    protected SignScorer[] models;
    
    /**
     * Constructor with component models.
     */
    public SignScorerProduct(SignScorer[] models) { 
        this.models = models;
    }
    
    /** 
     * Returns a score between 0 (worst) and 1 (best) for the given sign 
     * and completeness flag, as the product of the scores assigned 
     * by the component models.
     */
    public double score(Sign sign, boolean complete) {
        double retval = 1.0;
        for (int i = 0; i < models.length; i++) {
            retval *= models[i].score(sign, complete);
        }
        return retval;
    }
}

