package binder.utils;


import binder.autogen.beliefmodel.Color;
import binder.autogen.beliefmodel.ColorProperty;
import binder.autogen.beliefmodel.ComplexFormula;
import binder.autogen.beliefmodel.LogicalOp;
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
			if (FeatureValueUtils.hasValue(fv, "blue")) {
				property.colorValue = Color.blue;
			}
			if (FeatureValueUtils.hasValue(fv, "green")) {
				property.colorValue = Color.green;
			}

			property.unc = fv.independentProb;
			return property;
		}
		
		if (featlabel.equals("shape")) {
			ShapeProperty property = new ShapeProperty();

			if (FeatureValueUtils.hasValue(fv, "cylindrical")) {
				property.shapeValue = Shape.cylindrical;
			}
			if (FeatureValueUtils.hasValue(fv, "spherical")) {
				property.shapeValue = Shape.spherical;
			}
			if (FeatureValueUtils.hasValue(fv, "cubic")) {
				property.shapeValue = Shape.cubic;
			}

			property.unc = fv.independentProb;
			return property;
		}

		return new UncertainSuperFormula();
	}

	
	
	public static Object getValue(SuperFormula property) {
		if (property instanceof ColorProperty) {
			return ((ColorProperty)property).colorValue;
		}
		else if (property instanceof ShapeProperty) {
			return ((ShapeProperty)property).shapeValue;
		}
		return null;
	}
	
	
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
		 		
		return result;
	}
	

	private static void log(String str) {
		if (LOGGING)
			System.out.println("[BeliefModelUtils] "  + str);
	}
}
