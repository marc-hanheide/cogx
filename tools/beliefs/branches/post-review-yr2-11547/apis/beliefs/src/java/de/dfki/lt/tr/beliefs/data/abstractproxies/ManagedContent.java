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
package de.dfki.lt.tr.beliefs.data.abstractproxies;

//=================================================================
//IMPORTS

//Java
import de.dfki.lt.tr.beliefs.data.genericproxies.Distribution;
import de.dfki.lt.tr.beliefs.slice.distribs.ProbDistribution;

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

public abstract class ManagedContent<T extends ProbDistribution, C extends Ice.Object, P extends Proxy<? extends C>>
		extends Distribution<T> {

	protected ProxyFactory<C, ? extends P> _factory;

	protected ManagedContent(Class<? extends T> class1,
			ProxyFactory<C, ? extends P> factory, ProbDistribution content) {
		super(class1, content);
		_factory = factory;
	}

	protected P createElement(C o) {
		return _factory.create(o);
	}

} // end class
