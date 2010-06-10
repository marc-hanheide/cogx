/**
 * 
 */
package eu.cogx.beliefproxies.proxies.values;

import java.util.Collections;
import java.util.Iterator;
import java.util.Map;
import java.util.Vector;
import java.util.Map.Entry;

import de.dfki.lt.tr.beliefs.slice.distribs.FormulaProbPair;
import de.dfki.lt.tr.beliefs.slice.distribs.FormulaValues;
import de.dfki.lt.tr.beliefs.slice.distribs.ProbDistribution;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ElementaryFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.FloatFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.IntegerFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.UnknownFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.beliefs.util.BeliefInvalidOperationException;
import eu.cogx.beliefproxies.factories.ProxyFactory;
import eu.cogx.beliefproxies.proxies.Proxy;

/**
 * @author marc
 * 
 */
public class FormulaValuesProxy<T extends ProxyFactory<Proxy<? extends dFormula>>>
		extends Proxy<FormulaValues> implements Iterable<FormulaProbPair> {
	protected final T factory;

	public FormulaValuesProxy(T factory, Ice.Object content) {
		super(FormulaValues.class, content);
		this.factory = factory;
	}

	@Override
	public Iterator<FormulaProbPair> iterator() {
		return _content.iterator();
	}

	public void add(dFormula formula, double d) {
		_content.values.add(new FormulaProbPair(formula, (float) d));
	}

	public void add(String f, double d) {
		add(new ElementaryFormula(-1, f),d);
	}

	public void add(float f, double prob) {
		add(new FloatFormula(-1, f), prob);
	}

	public void add(int f, double prob) {
		add(new IntegerFormula(-1, f), prob);
	}

	public FormulaProbPair findFormula(String query) {
		for (FormulaProbPair f : _content.values) {
			if (f.val instanceof ElementaryFormula) {
				ElementaryFormula ef = (ElementaryFormula) f.val;
				if (ef.prop.equals(query)) {
					return f;
				}
			}
		}
		return null;
	}

	public float getProb(String query) {
		FormulaProbPair f = findFormula(query);
		return (float) ((f == null) ? 0.0 : f.prob);
	}

	public FormulaProbPair findFormula(int query) {
		for (FormulaProbPair f : _content.values) {
			if (f.val instanceof IntegerFormula) {
				IntegerFormula ef = (IntegerFormula) f.val;
				if (ef.val == query) {
					return f;
				}
			}
		}
		return null;
	}

	public float getProb(int query) {
		FormulaProbPair f = findFormula(query);
		return (float) ((f == null) ? 0.0 : f.prob);
	}

	public void setProb(String query, double prob) {
		FormulaProbPair f = findFormula(query);
		if (f == null)
			_content.values.add(new FormulaProbPair(new ElementaryFormula(-1,
					query), (float) prob));
		else
			f.prob = (float) prob;
	}

	public void setProb(int query, float prob) {
		FormulaProbPair f = findFormula(query);
		if (f == null)
			_content.values.add(new FormulaProbPair(new IntegerFormula(-1,
					query), prob));
		else
			f.prob = prob;
	}

	public void setAll(Object[][] init) {
		for (Object[] entry : init) {
			dFormula formula = getFormulaObject(entry[0]);
			double prob = ((Double) entry[1]).floatValue();
			this.add(formula, prob);
		}
		
	}

	private dFormula getFormulaObject(Object object) {
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

	/* (non-Javadoc)
	 * @see java.lang.Object#toString()
	 */
	@Override
	public String toString() {
			String result= "{";
			for (FormulaProbPair e : _content.values) {
				String value="*";
				if (e.val instanceof ElementaryFormula)
					value = ((ElementaryFormula) e.val).prop;
				else if (e.val instanceof IntegerFormula)
					value = Integer.toString(((IntegerFormula) e.val).val);
				result+=value + "=>" + e.prob+" ";
			}
			result+="}";
			return result;
	}

}
