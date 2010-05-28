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
import java.util.Collection;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;
import java.util.Vector;

import de.dfki.lt.tr.beliefs.slice.distribs.CondIndependentDistribs;
import de.dfki.lt.tr.beliefs.slice.distribs.ProbDistribution;
import de.dfki.lt.tr.beliefs.util.BeliefInvalidOperationException;
import de.dfki.lt.tr.beliefs.util.BeliefNotInitializedException;

/**
 * The <tt>Content</tt> class provides all the principal methods for operating
 * on the content of a situated, multi-agent belief. The basic underlying
 * structure is a collection of one or more probabilistic distributions. This
 * collection can be provided directly to set up the content, or it can be
 * constructed.
 * <p>
 * The class provides methods for constructing several types of distributions,
 * including basic distributions from (formula, probability) pairs and
 * dictionaries of conditionally independent distributions. These distributions
 * can be constructed as externally used objects. In addition, the class
 * provides direct methods for initializing a content object with a dictionary
 * of conditionally independent distributions, and accessing this dictionary
 * directly (rather than through externally constructed objects).
 * <p>
 * The following creates a basic probability distribution:
 * 
 * <pre>
 * LinkedList fpList = new LinkedList&lt;FormulaProbPair&gt;();
 * Formula fp1 = new Formula();
 * fp1.init();
 * fp1.setId(1);
 * fp1.asProposition(&quot;prop1&quot;); // set the proposition 
 * fp1.setProbability(0.3f); // set the probability
 * fpList.add(fp1.getAsPair()); // get the (formula, probability) as pair, and add
 * Formula fp2 = new Formula();
 * fp2.init();
 * fp2.setId(2);
 * fp2.asProposition(&quot;prop2&quot;);
 * fp2.setProbability(0.6f);
 * fpList.add(fp2.getAsPair());
 * FormulaValues formulaValues = new FormulaValues(fpList);
 * BasicProbDistribution baseDist = Content.createBasicDistribution(&quot;formula&quot;,
 * 		formulaValues);
 * </pre>
 * <p>
 * 
 * We can then create a dictionary of conditionally independent distributions,
 * and add the above one:
 * 
 * <pre>
 * // create a dictionary of conditionally independent distributions
 * CondIndependentDistribs ciDists = Content
 * 		.createConditionallyIndependentDistributions();
 * // add the new distribution directly to the dictionairy of conditionally independent dists
 * Content.addConditionallyIndependentDistribution(ciDists, baseDist);
 * </pre>
 * <p>
 * 
 * The dictionary can be set as the distribution in the content object using the
 * <tt>setDictionary</tt> method. This then allows us to add more distributions,
 * directly through the content object:
 * 
 * <pre>
 * // now create another base distribution, 
 * BasicProbDistribution baseDist2 = Content.createBasicDistribution(&quot;formula2&quot;,
 * 		formulaValues);
 * // add it to the content directly
 * content.addDistributionToDictionary(baseDist2);
 * </pre>
 * <p>
 * 
 * The class also provides a method <tt>getDistributionFromDictionary</tt> for
 * immediately retrieving a distribution, whereas
 * <tt>removeDistributionFromDictionary</tt> removes a distribution by key.
 * <p>
 * In principle these methods make it possible to create content around a
 * dictionary without having to construct that dictionary as external object at
 * all. By initializing the content structure, and then calling
 * <tt>asDictionary</tt> we can directly apply
 * <tt>add/get/removeDistributionFromDictionary</tt>.
 * 
 * <pre>
 * Content content = new Content();
 * content.init();
 * content.asDictionary();
 * // construct baseDist 
 * // ... 
 * content.addConditionallyIndependentDistribution(ciDists, baseDist);
 * // construct baseDist2
 * // ... 
 * content.addConditionallyIndependentDistribution(ciDists, baseDist2);
 * </pre>
 * <p>
 * 
 * 
 * @author Geert-Jan M. Kruijff (gj@dfki.de)
 * @author Pierre Lison (pierre.lison@dfki.de)
 * @version 100521
 * @started 100523
 */

public class ConditionallyIndependentDistributionProxy extends
		ContentProxy<CondIndependentDistribs> implements Map<String, ContentProxy<?>> {

	private final CondIndependentDistribs _content;

	/**
	 * Initializes the internal datastructures
	 */

	public ConditionallyIndependentDistributionProxy() {
		_content = new CondIndependentDistribs(new HashMap<String, ProbDistribution>());
	} 

	public ConditionallyIndependentDistributionProxy(ProbDistribution pd) {
		if (pd instanceof CondIndependentDistribs) {
			_content = (CondIndependentDistribs) pd;
		} else {
			throw (new BeliefInvalidOperationException("trying to create "
					+ ConditionallyIndependentDistributionProxy.class
							.getName() + " from " + pd.getClass().getName()));
		}
	} // end init

	/**
	 * Checks for initialization of the content data structure
	 * 
	 * @param msg
	 *            The message for the exception
	 * @throws BeliefNotInitializedException
	 *             If the content has not yet been initialized
	 */

	public CondIndependentDistribs get() {
		return _content;
	}

	/**
	 * 
	 * @see java.util.Map#clear()
	 */
	public void clear() {
		_content.distribs.clear();
	}

	/**
	 * @param arg0
	 * @return
	 * @see java.util.Map#containsKey(java.lang.Object)
	 */
	public boolean containsKey(Object arg0) {
		return _content.distribs.containsKey(arg0);
	}

	/**
	 * @param arg0
	 * @return
	 * @see java.util.Map#containsValue(java.lang.Object)
	 */
	public boolean containsValue(Object arg0) {
		return _content.distribs.containsValue(arg0);
	}

	/**
	 * @return
	 * @see java.util.Map#entrySet()
	 */
	public Set<Entry<String, ContentProxy<?>>> entrySet() {
		Map<String, ContentProxy<?>> result = new HashMap<String, ContentProxy<?>>();
		for (Entry<String, ProbDistribution> pd : _content.distribs.entrySet()) {
			result.put(pd.getKey(), createDelegate(pd.getValue()));
		}
		return result.entrySet();
	}

	/**
	 * @param arg0
	 * @return
	 * @see java.util.Map#equals(java.lang.Object)
	 */
	public boolean equals(Object arg0) {
		return _content.distribs.equals(arg0);
	}

	/**
	 * @param arg0
	 * @return
	 * @see java.util.Map#get(java.lang.Object)
	 */
	public ContentProxy<?> get(Object arg0) {
		ProbDistribution pd = _content.distribs.get(arg0);
		if (pd!=null)
			return createDelegate(pd);
		else
			return null;
	}

	/**
	 * @return
	 * @see java.util.Map#hashCode()
	 */
	public int hashCode() {
		return _content.distribs.hashCode();
	}

	/**
	 * @return
	 * @see java.util.Map#isEmpty()
	 */
	public boolean isEmpty() {
		return _content.distribs.isEmpty();
	}

	/**
	 * @return
	 * @see java.util.Map#keySet()
	 */
	public Set<String> keySet() {
		return _content.distribs.keySet();
	}

	/**
	 * @param arg0
	 * @param arg1
	 * @return
	 * @see java.util.Map#put(java.lang.Object, java.lang.Object)
	 */
	public ContentProxy<?> put(String arg0, ContentProxy<?> arg1) {
		ProbDistribution pd = _content.distribs.put(arg0, arg1.getContent());
		if (pd!=null)
			return createDelegate(pd);
		else
			return null;
	}

	/**
	 * @param arg0
	 * @see java.util.Map#putAll(java.util.Map)
	 */
	public void putAll(Map<? extends String, ? extends ContentProxy<?>> arg0) {
		for (Entry<? extends String, ? extends ContentProxy<?>> e : arg0
				.entrySet()) {
			_content.distribs.put(e.getKey(), e.getValue().getContent());
		}
	}

	/**
	 * @param arg0
	 * @return
	 * @see java.util.Map#remove(java.lang.Object)
	 */
	public ContentProxy<?> remove(Object arg0) {
		ProbDistribution pd = _content.distribs.remove(arg0);
		if (pd!=null)
			return createDelegate(pd);
		else
			return null;
	}

	/**
	 * @return
	 * @see java.util.Map#size()
	 */
	public int size() {
		return _content.distribs.size();
	}

	/**
	 * @return
	 * @see java.util.Map#values()
	 */
	public Collection<ContentProxy<?>> values() {
		Vector<ContentProxy<?>> result = new Vector<ContentProxy<?>>(
				_content.distribs.size());
		for (ProbDistribution p : _content.distribs.values()) {
			result.add(createDelegate(p));
		}
		return result;
	}

	@Override
	public CondIndependentDistribs getContent() {
		return _content;
	}

	public void put(BasicProbDistributionProxy basicProbDistributionProxy) {
		put(basicProbDistributionProxy.getId(), basicProbDistributionProxy);
	}

} // end class
