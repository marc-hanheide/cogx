package de.dfki.lt.tr.beliefs.data;

import java.util.Iterator;
import java.util.LinkedList;

import de.dfki.lt.tr.beliefs.data.abstractproxies.Proxy;
import de.dfki.lt.tr.beliefs.data.formulas.Formula;
import de.dfki.lt.tr.beliefs.data.genericproxies.DistributionContent;
import de.dfki.lt.tr.beliefs.data.genericproxies.GenericBasicDistribution;
import de.dfki.lt.tr.beliefs.data.genericproxies.GenericFormula;
import de.dfki.lt.tr.beliefs.slice.distribs.DistributionValues;
import de.dfki.lt.tr.beliefs.slice.distribs.FormulaProbPair;
import de.dfki.lt.tr.beliefs.slice.distribs.FormulaValues;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.BooleanFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ElementaryFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.FloatFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.IntegerFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.UnknownFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.beliefs.util.BeliefInvalidOperationException;
import de.dfki.lt.tr.beliefs.util.ProbFormula;

/**
 * a {@link Proxy} for the list of formulas considered as alternative values in
 * a {@link GenericBasicDistribution}.
 * 
 * @author Marc Hanheide (marc@hanheide.de)
 * 
 * @param <T>
 *            the type of the factory used to generate the Formula proxies.
 */
public class Formulas extends DistributionContent<FormulaValues> implements
		Iterable<ProbFormula> {

	public static Formulas create(DistributionValues content) {
		return new Formulas(content);
	}

	public static Formulas create() {
		return new Formulas(
				new FormulaValues(new LinkedList<FormulaProbPair>()));
	}

	public static Formulas create(GenericFormula<?> f) {
		Formulas fs = new Formulas(new FormulaValues(
				new LinkedList<FormulaProbPair>()));
		fs.add(f.get(), 1.0);
		return fs;
	}

	protected Formulas(DistributionValues content) {
		super(FormulaValues.class, content);
	}

	public void add(boolean b, double prob) {
		add(new BooleanFormula(-1, b), prob);
	}

	public void add(dFormula formula, double d) {
		_content.values.add(new FormulaProbPair(formula, (float) d));
	}

	public void add(float f, double prob) {
		add(new FloatFormula(-1, f), prob);
	}

	public void add(int f, double prob) {
		add(new IntegerFormula(-1, f), prob);
	}

	public void add(String f, double d) {
		add(new ElementaryFormula(-1, f), d);
	}

	public float getProb(int query) {
		FormulaProbPair f = findFormula(query);
		return (float) ((f == null) ? 0.0 : f.prob);
	}

	public float getProb(String query) {
		FormulaProbPair f = findFormula(query);
		return (float) ((f == null) ? 0.0 : f.prob);
	}

	// public float getProb(WorkingMemoryAddress query) {
	// FormulaProbPair f = findFormula(query);
	// return (float) ((f == null) ? 0.0 : f.prob);
	// }

	// @Override
	public Iterator<ProbFormula> iterator() {
		final Iterator<FormulaProbPair> internalIter = _content.values
				.iterator();
		return new Iterator<ProbFormula>() {

			@Override
			public boolean hasNext() {
				return internalIter.hasNext();
			}

			@Override
			public ProbFormula next() {
				FormulaProbPair pair = internalIter.next();
				if (pair != null) {
					Formula s = Formula.create(pair.val);
					return new ProbFormula(s, pair.prob);
				}
				return null;
			}

			@Override
			public void remove() {
				internalIter.remove();
			}
		};
	}

	public void addAll(Object[][] init) {
		for (Object[] entry : init) {
			dFormula formula = getFormulaObject(entry[0]);
			double prob = ((Double) entry[1]).floatValue();
			this.add(formula, prob);
		}

	}

	public void setProb(int query, double prob) {
		FormulaProbPair f = findFormula(query);
		if (f == null)
			_content.values.add(new FormulaProbPair(new IntegerFormula(-1,
					query), (float) prob));
		else
			f.prob = (float) prob;
	}

	public void setProb(String query, double prob) {
		FormulaProbPair f = findFormula(query);
		if (f == null)
			_content.values.add(new FormulaProbPair(new ElementaryFormula(-1,
					query), (float) prob));
		else
			f.prob = (float) prob;
	}

	// public void setProb(WorkingMemoryAddress query, double prob) {
	// FormulaProbPair f = findFormula(query);
	// if (f == null)
	// _content.values.add(new FormulaProbPair(new PointerFormula(-1,
	// query), (float) prob));
	// else
	// f.prob = (float) prob;
	// }

	/*
	 * (non-Javadoc)
	 * 
	 * @see java.lang.Object#toString()
	 */
	@Override
	public String toString() {
		String result = "{";
		for (FormulaProbPair e : _content.values) {
			String value = "*";
			if (e.val instanceof ElementaryFormula)
				value = ((ElementaryFormula) e.val).prop;
			else if (e.val instanceof IntegerFormula)
				value = Integer.toString(((IntegerFormula) e.val).val);
			result += value + "=>" + e.prob + " ";
		}
		result += "}";
		return result;
	}

	protected FormulaProbPair findFormula(boolean query) {
		for (FormulaProbPair f : _content.values) {
			if (f.val instanceof BooleanFormula) {
				BooleanFormula ef = (BooleanFormula) f.val;
				if (ef.val == query) {
					return f;
				}
			}
		}
		return null;
	}

	protected FormulaProbPair findFormula(int query) {
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

	protected FormulaProbPair findFormula(String query) {
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

	// protected FormulaProbPair findFormula(WorkingMemoryAddress query) {
	// for (FormulaProbPair f : _content.values) {
	// // if (f.val instanceof PointerFormula) {
	// // PointerFormula ef = (PointerFormula) f.val;
	// // if (ef.pointer.equals(query)) {
	// // return f;
	// // }
	// // }
	// }
	// return null;
	// }

	protected dFormula getFormulaObject(Object object) {
		if (object == null)
			return new UnknownFormula(-1);
		if (object instanceof String)
			return new ElementaryFormula(-1, (String) object);
		else if (object instanceof Double)
			return new FloatFormula(-1, ((Double) object).floatValue());
		else if (object instanceof Integer)
			return new IntegerFormula(-1, ((Integer) object).intValue());
		// else if (object instanceof WorkingMemoryAddress)
		// return new PointerFormula(-1, ((WorkingMemoryAddress) object));
		else
			throw new BeliefInvalidOperationException(
					"cannot create Formula objects for type "
							+ object.getClass().getName());
	}

	public int size() {
		return _content.values.size();
	}

	public Formula firstValue() {
		return Formula.create(_content.values.get(0).val);
	}

	public Formula getMostLikely() {
		double max = -1;
		dFormula maxFormula = null;
		for (FormulaProbPair f : _content.values) {
			if (f.prob > max) {
				max = f.prob;
				maxFormula = f.val;
			}
		}
		// if (maxFormula != null) {
			return Formula.create(maxFormula);
//		} else {
//			return null;
//		}

	}

	/**
	 * computes the Shannon entropy (in 'nat')
	 * (http://en.wikipedia.org/wiki/Entropy_(information_theory)} on all
	 * formulas in this distribution. The entropy is 0 if we have one discrete
	 * value for sure and some positive number increasing with the 'randomness'
	 * of the distribution.
	 * 
	 * @return the entropy
	 */
	public double entropy() {
		double result = 0.0;
		for (FormulaProbPair f : _content.values) {
			if (f.prob > 0.0) {
				result += Math.log(f.prob) * f.prob;
			}
		}
		return -result;
	}

	// /* (non-Javadoc)
	// * @see java.lang.Object#equals(java.lang.Object)
	// */
	// @Override
	// public boolean equals(Object obj) {
	// if (!(obj instanceof Formulas))
	// return super.equals(obj);
	// Formulas other=(Formulas) obj;
	// Formula ml1=this.getMostLikely();
	// Formula ml2=other.getMostLikely();
	// // we cannot compare apple and pears...
	// if (!ml1.get().getClass().isInstance(ml2.get()))
	// return false;
	//		
	// return ml1.equals(ml2);
	// }

}
