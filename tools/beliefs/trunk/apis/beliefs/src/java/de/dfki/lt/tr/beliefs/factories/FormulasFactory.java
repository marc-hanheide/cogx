/**
 * 
 */
package de.dfki.lt.tr.beliefs.factories;

import java.util.LinkedList;

import de.dfki.lt.tr.beliefs.data.Formula;
import de.dfki.lt.tr.beliefs.data.Formulas;
import de.dfki.lt.tr.beliefs.data.abstractproxies.AbstractProxyFactory;
import de.dfki.lt.tr.beliefs.slice.distribs.FormulaProbPair;
import de.dfki.lt.tr.beliefs.slice.distribs.FormulaValues;
/**
 * @author marc
 * 
 */
public class FormulasFactory extends
		AbstractProxyFactory<Formulas<? extends Formula<?>>> {

	public Formulas<? extends Formula<?>> create() {
		FormulaValues fv = new FormulaValues(new LinkedList<FormulaProbPair>());
		return create(fv);
	}

	@Override
	public Formulas<? extends Formula<?>> create(Ice.Object pd) {
		return Formulas.create(new FormulaFactory(), pd);
	}

}
