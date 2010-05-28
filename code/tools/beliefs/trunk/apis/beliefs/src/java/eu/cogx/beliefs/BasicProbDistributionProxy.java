// =================================================================
// Copyright (C) 2010 DFKI GmbH Talking Robots 
// Geert-Jan M. Kruijff (gj@dfki.de)
//                                                                                                                          
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License 
// as published by the Free Software Foundation; either version 2.1 of
// the License, or (at your option) any later version.
//                                                                                                                          
// This library is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// Lesser General Public License for more details.
//                                                                                                                          
// You should have received a copy of the GNU Lesser General Public
// License along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
// 02111-1307, USA.
// =================================================================

// =================================================================
// PACKAGE DEFINITION 
package eu.cogx.beliefs;

//=================================================================
//IMPORTS

//Java
import java.util.List;

import de.dfki.lt.tr.beliefs.slice.distribs.BasicProbDistribution;
import de.dfki.lt.tr.beliefs.slice.distribs.DistributionValues;
import de.dfki.lt.tr.beliefs.slice.distribs.FormulaProbPair;
import de.dfki.lt.tr.beliefs.slice.distribs.FormulaValues;
import de.dfki.lt.tr.beliefs.slice.distribs.ProbDistribution;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.UnknownFormula;
import de.dfki.lt.tr.beliefs.util.BeliefInvalidOperationException;
import de.dfki.lt.tr.beliefs.util.BeliefMissingValueException;

public class BasicProbDistributionProxy extends ContentProxy<BasicProbDistribution> {

	private final BasicProbDistribution _content;
	
	public String getId() {
		return _content.key;
	}
	
	public BasicProbDistributionProxy(ProbDistribution pd) {
		if (pd instanceof BasicProbDistribution) {
			_content = (BasicProbDistribution) pd;
		} else {
			throw (new BeliefInvalidOperationException("trying to create "
					+ BasicProbDistributionProxy.class
							.getName() + " from " + pd.getClass().getName()));
		}
	} // end init

	public BasicProbDistributionProxy (String key, DistributionValues distValues)
	{
		// abort if the key is null or empty
		if (key == null || key.equals("")) 
		{
			throw new BeliefMissingValueException("Cannot create basic distribution: Provided key is null/empty");
		}
		// abort if the distribution values list is null or empty
		if (distValues == null)
		{
			throw new BeliefMissingValueException("Cannot create basic distribution: Provided distribution values is null");
		}
		
		_content=new BasicProbDistribution(key, distValues);
	} // end createBasicDistribution
	
	public BasicProbDistributionProxy (String key, List<FormulaProbPair> distValues)
	throws BeliefInvalidOperationException
	{
		// initialize the list of values
		List<FormulaProbPair> _values = distValues; 
		// initialize the total sum of the probabilities to 0
		float total = 0.0f;
		// cycle over the list of formula value pairs in the provided distribution 
		for (FormulaProbPair value : _values) {
			total += value.prob;
		}
		// check that the total values is not more than 1.0
		if (total > 1.01) {
			throw new BeliefInvalidOperationException("Error in creating a feature distribution: "+
					"probabilities of formula distribution sum up to more than 1 [" + total+"]");
		}
		else if (total < 0.99) 
		{
				// create a formula with proposition "unknown" and add it with a value 1-total
				FormulaProxy<?> _formula =FormulaProxy.createElementary("unknown", 1-total);
				_formula.setId(-100);
				FormulaProbPair uval = _formula.getAsPair();
				_values.add(uval);
		}
		// 
		FormulaValues formValues = new FormulaValues(_values);
		_content = new BasicProbDistribution (key, formValues);
	} // end createBasicDistFromProbabilityFormulas


	public BasicProbDistribution get() 
	{
		return _content;
	}

	@Override
	public BasicProbDistribution getContent() {
		return _content;
	}
	
} // end class
