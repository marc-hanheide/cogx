package comsys.processing.cca;

import Abducer.FunctionTerm;
import Abducer.Term;
import beliefmodels.domainmodel.cogx.Color;
import beliefmodels.domainmodel.cogx.ColorProperty;
import beliefmodels.domainmodel.cogx.ContinualFormula;
import beliefmodels.domainmodel.cogx.ObjectType;
import beliefmodels.domainmodel.cogx.ObjectTypeProperty;
import beliefmodels.domainmodel.cogx.Shape;
import beliefmodels.domainmodel.cogx.ShapeProperty;

import comsys.processing.cca.abduction.PredicateFactory;

public class ClarificationUtils {

	/**
	 * Return a term that corresponds to the given continual formula.
	 * @param cf
	 * @return the term, null if the formula cannot be converted to a term
	 */
	public static FunctionTerm continualFormulaToTerm(ContinualFormula cf) {
		String functor = null;
		Term arg = null;
		
		if (cf instanceof ObjectTypeProperty) {
			ObjectTypeProperty pf = (ObjectTypeProperty) cf;
			functor = "objecttype";
			arg = polarizeTerm(pf.polarity, PredicateFactory.term(pf.typeValue.toString()));
		}
		if (cf instanceof ColorProperty) {
			ColorProperty pf = (ColorProperty) cf;
			functor = "color";
			arg = polarizeTerm(pf.polarity, PredicateFactory.term(pf.colorValue.toString()));
		}
		if (cf instanceof ShapeProperty) {
			ShapeProperty pf = (ShapeProperty) cf;
			functor = "shape";
			arg = polarizeTerm(pf.polarity, PredicateFactory.term(pf.shapeValue.toString()));
		}
		
		if (functor != null) {
			return PredicateFactory.term(functor, new Term[] {arg});
		}
		else {
			return null;
		}
	}
	
	/**
	 * Return the term "not(<i>arg</i>)" if polarity is true, "<i>arg</i>" otherwise.
	 * @param polarity
	 * @param arg
	 * @return
	 */
	public static Term polarizeTerm(boolean polarity, Term arg) {
		if (polarity) {
			return arg;
		}
		else {
			return PredicateFactory.term("not", new Term[] {arg});
		}
	}

	public static String[] valueTermArgs(ContinualFormula cf) {
		String propName = "";
		String propValue = null;
		
		if (cf instanceof ColorProperty) {
			propName = "color";
			Color val = ((ColorProperty)cf).colorValue;
			if (val != Color.unknownColor)
				propValue = val.toString();
		}
		if (cf instanceof ShapeProperty) {
			propName = "shape";
			Shape val = ((ShapeProperty)cf).shapeValue;
			if (val != Shape.unknownShape)
				propValue = val.toString();
		}
		if (cf instanceof ObjectTypeProperty) {
			propName = "objecttype";
			ObjectType val = ((ObjectTypeProperty)cf).typeValue;
			if (val != ObjectType.unknownObjectType)
				propValue = val.toString();
		}
		if (propValue != null) {
			return new String[] {propName, propValue};
		}
		else {
			return new String[] {propName};
		}
	}

}
