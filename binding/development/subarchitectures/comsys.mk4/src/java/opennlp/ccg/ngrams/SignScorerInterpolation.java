///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2005 University of Edinburgh (Michael White)
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
 * Linear interpolation of sign scorers.
 *
 * @author      Michael White
 * @version     $Revision: 1.2 $, $Date: 2007/12/21 05:13:37 $
 */
public class SignScorerInterpolation implements SignScorer
{
    /** The component models. */
    protected SignScorer[] models;
    
    /** The weights. */
    protected double weights[];
    
    /**
     * Constructor with component models, which are given uniform weights.
     */
    public SignScorerInterpolation(SignScorer[] models) { 
        this.models = models;
        this.weights = new double[models.length];
        for (int i = 0; i < models.length; i++) {
            weights[i] = 1.0 / models.length;
        }
    }
    
    /**
     * Constructor with component models and weights.
     * The weights are assumed to sum to 1, 
     * and the number of weights is assumed to match the number of models.
     */
    public SignScorerInterpolation(SignScorer[] models, double[] weights) { 
        this.models = models;
        this.weights = weights;
    }
    
    /** 
     * Returns a score between 0 (worst) and 1 (best) for the given sign 
     * and completeness flag, as the interpolation of the scores assigned 
     * by the component models.
     * In particular, returns the linear combination using the established weights 
     * of the scores given by the component models. 
     */
    public double score(Sign sign, boolean complete) {
        double retval = 0;
        for (int i = 0; i < models.length; i++) {
            retval += models[i].score(sign, complete) * weights[i];
        }
        return retval;
    }
}

