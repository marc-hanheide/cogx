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
package de.dfki.lt.tr.beliefs.data;

import de.dfki.lt.tr.beliefs.data.genericproxies.DistributionContent;
import de.dfki.lt.tr.beliefs.slice.distribs.DistributionValues;
import de.dfki.lt.tr.beliefs.slice.distribs.NormalValues;

/**
 * The <tt>Formula</tt> class implements the basic structure for building up
 * content.
 * 
 * 
 * @author Geert-Jan M. Kruijff (gj@dfki.de), Marc Hanheide (marc@hanheide.de)
 * @started 100521
 * @version 100521
 */

public class Gaussian extends DistributionContent<NormalValues> {

	public static Gaussian create(DistributionValues o) {
		return new Gaussian(o);
	}
	
	public static Gaussian create(double m, double v) {
		NormalValues nv = new NormalValues(m, v);
		return create(nv);
	}

	
	/**
	 * Object is created from underlying slice-based datastructure.
	 */
	protected Gaussian(DistributionValues gaussian) {
		super(NormalValues.class, gaussian);

	} // end constructor

	public double getMean() {
		return _content.mean;
	}

	/**
	 * compute the density of value query in the current Gaussian
	 * 
	 * @param query
	 * @return the probability density value for the given value
	 */
	public double getProb(double query) {
		double exponent = (-((query - _content.mean) * (query - _content.mean)))
				/ (2 * _content.variance);
		return Math.exp(exponent) / Math.sqrt(2 * Math.PI * _content.variance);
	}

	public double getVariance() {
		return _content.variance;
	}

	public void set(double m, double v) {
		setMean(m);
		setVariance(v);
	}

	public void setMean(double m) {
		_content.mean = m;
	}

	public void setVariance(double v) {
		_content.variance = v;
	}

} // end class
