/**
 * 
 */
package de.dfki.lt.tr.beliefs.data;

import de.dfki.lt.tr.beliefs.data.abstractproxies.Proxy;
import de.dfki.lt.tr.beliefs.data.abstractproxies.ProxyFactory;
import de.dfki.lt.tr.beliefs.slice.distribs.DistributionValues;
import de.dfki.lt.tr.beliefs.slice.distribs.FormulaValues;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ElementaryFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.FloatFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.IntegerFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.UnknownFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.beliefs.util.BeliefInvalidOperationException;

/**
 * @author marc
 * 
 */
public class DistributionContent<T extends DistributionValues, Factory extends ProxyFactory<Proxy<? extends dFormula>>>
		extends Proxy<T> {
	protected final Factory _factory;

	public DistributionContent(Class<? extends T> type, Factory factory, Ice.Object content) {
		super(type, content);
		this._factory = factory;
	}

	protected dFormula getFormulaObject(Object object) {
		if (object == null)
			return new UnknownFormula(-1);
		if (object instanceof String)
			return new ElementaryFormula(-1, (String) object);
		else if (object instanceof Double)
			return new FloatFormula(-1, ((Double) object).floatValue());
		else if (object instanceof Integer)
			return new IntegerFormula(-1, ((Integer) object).intValue());
		else
			throw new BeliefInvalidOperationException(
					"cannot create Formula objects for type "
							+ object.getClass().getName());
	}

}
