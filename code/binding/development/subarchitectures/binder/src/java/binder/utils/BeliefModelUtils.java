package binder.utils;


import binder.autogen.beliefmodel.Color;
import binder.autogen.beliefmodel.ColorProperty;
import binder.autogen.beliefmodel.ComplexFormula;
import binder.autogen.beliefmodel.ComplexProperty;
import binder.autogen.beliefmodel.Entity;
import binder.autogen.beliefmodel.LogicalOp;
import binder.autogen.beliefmodel.Property;
import binder.autogen.beliefmodel.Shape;
import binder.autogen.beliefmodel.ShapeProperty;
import binder.autogen.beliefmodel.SuperFormula;
import binder.autogen.core.FeatureValue;

public class BeliefModelUtils {

	public static boolean LOGGING = true;

	public BeliefModelUtils() {

	}



	public static Property createNewProperty(String featlabel, FeatureValue fv) {

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

			return property;
		}
		
		if (featlabel.equals("shape")) {
			ShapeProperty property = new ShapeProperty();

			if (FeatureValueUtils.hasValue(fv, "cylindrical")) {
				property.shapedValue = Shape.cylindrical;
			}
			if (FeatureValueUtils.hasValue(fv, "spherical")) {
				property.shapedValue = Shape.spherical;
			}
			if (FeatureValueUtils.hasValue(fv, "cubic")) {
				property.shapedValue = Shape.cubic;
			}

			return property;
		}

		return new Property();
	}

	
	public static Object getValue(Property property) {
		if (property instanceof ColorProperty) {
			return ((ColorProperty)property).colorValue;
		}
		else if (property instanceof ShapeProperty) {
			return ((ShapeProperty)property).shapedValue;
		}
		return null;
	}
	
	public static String getFormulaPrettyPrint(SuperFormula formula) {
		String result = "\n";
		result += "@(" + formula.id + ":" + formula.getClass().getSimpleName() + " ^ ";
		if (formula instanceof ComplexFormula) {
			log("Number of formulae in complex formula: "  + ((ComplexFormula)formula).formulae.length);

			for (int i = 0 ; i < ((ComplexFormula)formula).formulae.length ; i++) {
				SuperFormula subformula = ((ComplexFormula)formula).formulae[i];

				result += "(" + subformula.id + " ^ "; 

				if (subformula instanceof Entity) {
					log("\tnumber of features for subformula "  + i + ": " + ((Entity)subformula).properties.length);
					for (int j = 0 ; j < ((Entity)subformula).properties.length ; j++) {
						
						Property property = ((Entity)subformula).properties[j];
						
						if (property instanceof ComplexProperty) {
							result += "(";
							for (int k = 0 ; k < ((ComplexProperty)property).alternativeProperties.length ; k++) {
								Property altproperty = ((ComplexProperty)property).alternativeProperties[k];
								result += "<" + altproperty.getClass().getSimpleName() + "> ";
								result += getValue(altproperty);
								if (k < (((ComplexProperty)property).alternativeProperties.length - 1)) {
									result += " v \n\t\t ";
								}
							}
							result += ")";
						}
						else {
						result += "<" + property.getClass().getSimpleName() + "> ";
						result += getValue(property);
						log("current property: <" + property.getClass().getSimpleName() + "> " + getValue(property));
						
						}
						
						if (j < (((Entity)subformula).properties.length - 1)) {
							result += " ^ \n\t\t";
						}
					}
				}
				result += ")";
				
				if (i < (((ComplexFormula)formula).formulae.length - 1)) {
					if (((ComplexFormula)formula).op.equals(LogicalOp.and)) {
						result += " ^ \n\t";
					}
				}
			}
		}
		result += ")\n";
		return result;
	}

	private static void log(String str) {
		if (LOGGING)
			System.out.println("[BeliefModelUtils] "  + str);
	}
}
