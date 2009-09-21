package binder.utils;


import binder.autogen.beliefmodel.Color;
import binder.autogen.beliefmodel.ColorProperty;
import binder.autogen.beliefmodel.ComplexFormula;
import binder.autogen.beliefmodel.GraspableProperty;
import binder.autogen.beliefmodel.LinguisticAttributeProperty;
import binder.autogen.beliefmodel.LinguisticLabelProperty;
import binder.autogen.beliefmodel.LocationProperty;
import binder.autogen.beliefmodel.LogicalOp;
import binder.autogen.beliefmodel.ObjectType;
import binder.autogen.beliefmodel.ObjectTypeProperty;
import binder.autogen.beliefmodel.Shape;
import binder.autogen.beliefmodel.ShapeProperty;
import binder.autogen.beliefmodel.SuperFormula;
import binder.autogen.beliefmodel.UncertainSuperFormula;
import binder.autogen.core.FeatureValue;

public class BeliefModelUtils {

	public static boolean LOGGING = true;

	public BeliefModelUtils() {

	}


	public static UncertainSuperFormula createNewProperty(String featlabel, FeatureValue fv) {

		if (featlabel.equals("colour")) {
			ColorProperty property = new ColorProperty();

			if (FeatureValueUtils.hasValue(fv, "red")) {
				property.colorValue = Color.red;
			}
			else if (FeatureValueUtils.hasValue(fv, "blue")) {
				property.colorValue = Color.blue;
			}
			else if (FeatureValueUtils.hasValue(fv, "green")) {
				property.colorValue = Color.green;
			}
			else {
				log("WARNING: feature value \"" + FeatureValueUtils.toString(fv) + "\" currently not supported in the typed belief model");
			}

			property.unc = fv.independentProb;
			return property;
		}
		
		else if (featlabel.equals("shape")) {
			ShapeProperty property = new ShapeProperty();

			if (FeatureValueUtils.hasValue(fv, "cylindrical")) {
				property.shapeValue = Shape.cylindrical;
			}
			else if (FeatureValueUtils.hasValue(fv, "spherical")) {
				property.shapeValue = Shape.spherical;
			}
			else if (FeatureValueUtils.hasValue(fv, "cubic")) {
				property.shapeValue = Shape.cubic;
			}
			else {
				log("WARNING: feature value \"" + FeatureValueUtils.toString(fv) + "\" currently not supported in the typed belief model");
			}

			property.unc = fv.independentProb;
			return property;
		}
		
		else if (featlabel.equals("obj_label")) {
			ObjectTypeProperty property = new ObjectTypeProperty();

			if (FeatureValueUtils.hasValue(fv, "mug")) {
				property.typeValue = ObjectType.mug;
			}
			else if (FeatureValueUtils.hasValue(fv,"ball")) {
				property.typeValue = ObjectType.ball;
			}
			else {
				log("WARNING: feature value \"" + FeatureValueUtils.toString(fv) + "\" currently not supported in the typed belief model");
			}

			property.unc = fv.independentProb;
			return property;
		}
		
		else if (featlabel.equals("graspable")) {
			GraspableProperty property = new GraspableProperty();

			if (FeatureValueUtils.hasValue(fv, true)) {
				property.graspableValue = true;
			}
			else if (FeatureValueUtils.hasValue(fv,false)) {
				property.graspableValue = false;
			}
			else {
				log("WARNING: feature value \"" + FeatureValueUtils.toString(fv) + "\" currently not supported in the typed belief model");
			}

			property.unc = fv.independentProb;
			return property;
		}
		
		else if (featlabel.equals("location")) {
			LocationProperty property = new LocationProperty();

			property.location = FeatureValueUtils.getString(fv);
			
			property.unc = fv.independentProb;
			return property;
		}
		
		else if (featlabel.equals("ling_label")) {
			LinguisticLabelProperty property = new LinguisticLabelProperty();

			property.label = FeatureValueUtils.getString(fv);
			
			property.unc = fv.independentProb;
			return property;
		}
		
		else if (featlabel.equals("ling_attribute")) {
			LinguisticAttributeProperty property = new LinguisticAttributeProperty();

			property.attribute = FeatureValueUtils.getString(fv);
			
			property.unc = fv.independentProb;
			return property;
		}
		

		log("WARNING: feature \"" + featlabel + "\" not supported in the typing currently specified for the belief model");
		return new UncertainSuperFormula();
	}

	
	/**
	public static Object getValue(SuperFormula property) {
		if (property instanceof ColorProperty) {
			return ((ColorProperty)property).colorValue;
		}
		else if (property instanceof ShapeProperty) {
			return ((ShapeProperty)property).shapeValue;
		}
		return null;
	}
	*/
	
	public static String getFormulaPrettyPrint(SuperFormula formula) {
		String result = "@(" ;
		
		result += getFormulaPrettyPrint(formula, 1);
		
		result += ")";
		
		return result;
	}
	
	private static String getIndent(int nbIndents) {
		String str = "";
		for (int i = 0 ; i < nbIndents; i++) {
			str += "     ";
		}
		return str;
	}
	
	private static String getOperatorPrettyPrint (ComplexFormula form) {
		String operator = " ";
		if ((form).op.equals(LogicalOp.and)) {
			operator = "^";
		}
		else if ((form).op.equals(LogicalOp.xor)) {
			operator = "v";
		}
		return operator;
	}
	
	private static String getUncertaintyValuePrettyPrint(UncertainSuperFormula formula) {
		String str = "";
		if (formula.unc > 0.0) {
			str += "[" + Math.round(formula.unc*100.0) / 100.0 + "]";
		}
		return str;
	}
	
	
	public static String getFormulaPrettyPrint(SuperFormula formula, int depth) {
		
		String result = formula.id + " ^";
		
		if (formula instanceof ComplexFormula) {
			
			result += "\n" +  getIndent(depth) + "(";
			
			String operator = getOperatorPrettyPrint((ComplexFormula)formula);
			
			for (int i = 0; i < ((ComplexFormula)formula).formulae.length ; i++) {
				SuperFormula subformula = ((ComplexFormula)formula).formulae[i];
				
				result += getFormulaPrettyPrint(subformula, depth + 1);
				
				if (i < (((ComplexFormula)formula).formulae.length - 1)) {
					result += " " + operator + "\n" + getIndent(depth) + " ";
				}
			}
			result += ")";
		}
		
		else if (formula instanceof ColorProperty) {
			result += " <Colour> " + ((ColorProperty)formula).colorValue;
			if (formula instanceof UncertainSuperFormula) {
				result += " " + getUncertaintyValuePrettyPrint((UncertainSuperFormula)formula);
			}
		}
		else if (formula instanceof ShapeProperty) {
			result += " <Shape> " + ((ShapeProperty)formula).shapeValue;
			if (formula instanceof UncertainSuperFormula) {
				result += " " + getUncertaintyValuePrettyPrint((UncertainSuperFormula)formula);
			}
		}
		else if (formula instanceof ObjectTypeProperty) {
			result += " <ObjectType> " + ((ObjectTypeProperty)formula).typeValue;	
			if (formula instanceof UncertainSuperFormula) {
				result += " " + getUncertaintyValuePrettyPrint((UncertainSuperFormula)formula);
			}
		}
		else if (formula instanceof GraspableProperty) {
			result += " <Graspable> " + ((GraspableProperty)formula).graspableValue;	
			if (formula instanceof UncertainSuperFormula) {
				result += " " + getUncertaintyValuePrettyPrint((UncertainSuperFormula)formula);
			}
		}
		else if (formula instanceof LocationProperty) {
			result += " <Location> " + ((LocationProperty)formula).location;	
			if (formula instanceof UncertainSuperFormula) {
				result += " " + getUncertaintyValuePrettyPrint((UncertainSuperFormula)formula);
			}
		}
		else if (formula instanceof LinguisticLabelProperty) {
			result += " <LingLabel> " + ((LinguisticLabelProperty)formula).label;	
			if (formula instanceof UncertainSuperFormula) {
				result += " " + getUncertaintyValuePrettyPrint((UncertainSuperFormula)formula);
			}
		}
		else if (formula instanceof LinguisticAttributeProperty) {
			result += " <LingAttribute> " + ((LinguisticAttributeProperty)formula).attribute;	
			if (formula instanceof UncertainSuperFormula) {
				result += " " + getUncertaintyValuePrettyPrint((UncertainSuperFormula)formula);
			}
		}
		
		 		
		return result;
	}
	

	private static void log(String str) {
		if (LOGGING)
			System.out.println("[BeliefModelUtils] "  + str);
	}
}
