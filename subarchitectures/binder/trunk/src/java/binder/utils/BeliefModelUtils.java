// =================================================================                                                        
// Copyright (C) 2009-2011 Pierre Lison (pierre.lison@dfki.de)                                                                
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

package binder.utils;


import beliefmodels.adl.Agent;
import beliefmodels.adl.AgentStatus;
import beliefmodels.adl.AttributedAgentStatus;
import beliefmodels.adl.Belief;
import beliefmodels.adl.BeliefModel;
import beliefmodels.adl.MutualAgentStatus;
import beliefmodels.adl.PrivateAgentStatus;
import beliefmodels.adl.SpatioTemporalFrame;
import beliefmodels.domainmodel.cogx.Color;
import beliefmodels.domainmodel.cogx.ColorProperty;
import beliefmodels.domainmodel.cogx.ComplexFormula;
import beliefmodels.domainmodel.cogx.Graspable;
import beliefmodels.domainmodel.cogx.GraspableProperty;
import beliefmodels.domainmodel.cogx.LinguisticAttributeProperty;
import beliefmodels.domainmodel.cogx.LinguisticLabelProperty;
import beliefmodels.domainmodel.cogx.LocationProperty;
import beliefmodels.domainmodel.cogx.LogicalOp;
import beliefmodels.domainmodel.cogx.ObjectType;
import beliefmodels.domainmodel.cogx.ObjectTypeProperty;
import beliefmodels.domainmodel.cogx.Proximity;
import beliefmodels.domainmodel.cogx.ProximityProperty;
import beliefmodels.domainmodel.cogx.Saliency;
import beliefmodels.domainmodel.cogx.SaliencyProperty;
import beliefmodels.domainmodel.cogx.Shape;
import beliefmodels.domainmodel.cogx.ShapeProperty;
import beliefmodels.domainmodel.cogx.UncertainSuperFormula;
import beliefmodels.domainmodel.cogx.ContinualFormula;
import beliefmodels.domainmodel.cogx.UnionRefProperty;
import binder.autogen.core.FeatureValue;
import binder.autogen.featvalues.FloatValue;


/**
 * Utility library for creating and monitoring belief models
 * 
 * @author Pierre Lison
 * @version 23/09/2009 (started 18/09/2009)
 * 
 */

public class BeliefModelUtils {

	// flag to activate logging
	public static boolean LOGGING = true;

	public static boolean showProbabilitiesInPrettyPrint = true;

	// ================================================================= 
	// FORMULA CREATION METHODS   
	// ================================================================= 


	/**
	 * Create a new belief model formula containing the assignment of a feature to a 
	 * particular feature value
	 * 
	 * TODO: find another solution than hardcoding to enforce the strong typing of formulae
	 * 
	 * @param featlabel the feature label
	 * @param fv the feature value
	 * @return the belief model formula
	 */

	public static UncertainSuperFormula createNewProperty(String featlabel, FeatureValue fv) {

		// types for the colour feature
		if (featlabel.equals("colour")) {

			ColorProperty property = new ColorProperty();
			property.prob = fv.independentProb;

			if (FeatureValueUtils.hasValue(fv, "red")) { property.colorValue = Color.red; }
			else if (FeatureValueUtils.hasValue(fv, "blue")) {property.colorValue = Color.blue;	}
			else if (FeatureValueUtils.hasValue(fv, "green")) {	property.colorValue = Color.green;}
			else if (FeatureValueUtils.isUnknownValue(fv)) {	property.colorValue = Color.unknownColor;}

			else {
				log("WARNING: feature value \"" + FeatureValueUtils.toString(fv) + 
				"\" currently not supported in the typed belief model");
			}

			return property;
		}

		// types for the shape feature
		else if (featlabel.equals("shape")) {

			ShapeProperty property = new ShapeProperty();
			property.prob = fv.independentProb;

			if (FeatureValueUtils.hasValue(fv, "cylindrical")) { property.shapeValue = Shape.cylindrical; }
			else if (FeatureValueUtils.hasValue(fv, "spherical")) {	property.shapeValue = Shape.spherical; }
			else if (FeatureValueUtils.hasValue(fv, "cubic")) { property.shapeValue = Shape.cubic; }
			else if (FeatureValueUtils.isUnknownValue(fv)) {	property.shapeValue = Shape.unknownShape;}

			else {
				log("WARNING: feature value \"" + FeatureValueUtils.toString(fv) + 
				"\" currently not supported in the typed belief model");
			}

			return property;
		}

		// types for the obj_label feature
		else if (featlabel.equals("obj_label")) {

			ObjectTypeProperty property = new ObjectTypeProperty();
			property.prob = fv.independentProb;

			if (FeatureValueUtils.hasValue(fv, "mug")) { property.typeValue = ObjectType.mug; }
			else if (FeatureValueUtils.hasValue(fv,"ball")) { property.typeValue = ObjectType.ball;	}
			else if (FeatureValueUtils.isUnknownValue(fv)) {	property.typeValue = ObjectType.unknownObjectType; }

			else {
				log("WARNING: feature value \"" + FeatureValueUtils.toString(fv) + 
				"\" currently not supported in the typed belief model");
			}
			return property;
		}

		// types for the graspable feature
		else if (featlabel.equals("graspable")) {

			GraspableProperty property = new GraspableProperty();
			property.prob = fv.independentProb;

			if (FeatureValueUtils.hasValue(fv, true)) { property.graspableValue = Graspable.grasp;	}
			else if (FeatureValueUtils.hasValue(fv,false)) { property.graspableValue = Graspable.nograsp; }
			else if (FeatureValueUtils.isUnknownValue(fv)) { property.graspableValue = Graspable.unknownGrasp; }

			else {
				log("WARNING: feature value \"" + FeatureValueUtils.toString(fv) + 
				"\" currently not supported in the typed belief model");
			}

			return property;
		}

		// types for the location feature
		else if (featlabel.equals("location")) {

			LocationProperty property = new LocationProperty();
			property.prob = fv.independentProb;
			property.location = FeatureValueUtils.toString(fv);

			return property;
		}

		// types for the ling_label feature
		else if (featlabel.equals("ling_label")) {

			LinguisticLabelProperty property = new LinguisticLabelProperty();
			property.prob = fv.independentProb;
			property.label = FeatureValueUtils.toString(fv);

			return property;
		}

		// types for the ling_attribute feature
		else if (featlabel.equals("ling_attribute")) {

			LinguisticAttributeProperty property = new LinguisticAttributeProperty();
			property.prob = fv.independentProb;
			property.attribute = FeatureValueUtils.toString(fv);

			return property;
		}

		// types for the ling_attribute feature
	/**	else if (featlabel.equals("boundPhantom")) {

			BoundPhantomProxyProperty property = new BoundPhantomProxyProperty();
			property.prob = fv.independentProb;
			property.boundProxy = FeatureValueUtils.toString(fv);
			return property;
		} */
		
		else if (featlabel.equals("unionRef")) {
			UnionRefProperty property = new UnionRefProperty();
			property.prob = fv.independentProb;
			property.unionRef = FeatureValueUtils.toString(fv);
			
			return property;
		}
		
		else if (featlabel.equals("saliency")) {
			SaliencyProperty property = new SaliencyProperty();
			property.prob = fv.independentProb;
			if (FeatureValueUtils.hasValue(fv, "low")) { property.sal = Saliency.low; }
			if (FeatureValueUtils.hasValue(fv, "high")) { property.sal = Saliency.high; }
			
			return property;
		}
		
		else if (featlabel.equals("ling_proximity")) {
			ProximityProperty property = new ProximityProperty();
			property.prob = fv.independentProb;
			if (FeatureValueUtils.hasValue(fv, "proximal")) { property.prox = Proximity.proximal; }
			if (FeatureValueUtils.hasValue(fv, "distal")) { property.prox = Proximity.distal; }
			
			return property;
		}

		// and if the feature doesn't belong to one of the above categories...
		else {
			log("WARNING: feature \"" + featlabel + 
			"\" not supported in the typing currently specified for the belief model");
		}
		return new UncertainSuperFormula();
	}


	public static boolean arePropertiesEqual (UncertainSuperFormula form1, UncertainSuperFormula form2) {

		if (form1.getClass().equals(form2.getClass())) {

			// if the formula is a simple colour property
			if (form1 instanceof ColorProperty) {
				Color color1 = ((ColorProperty)form1).colorValue;
				Color color2 = ((ColorProperty)form2).colorValue;
				return (color1.equals(color2));
			}

			// if the formula is a simple shape property
			else if (form1 instanceof ShapeProperty) {
				Shape shape1 = ((ShapeProperty)form1).shapeValue;
				Shape shape2 = ((ShapeProperty)form2).shapeValue;
				return (shape1.equals(shape2));
			}
			else if (form1 instanceof ObjectTypeProperty) {
				ObjectType ot1 = ((ObjectTypeProperty)form1).typeValue;
				ObjectType ot2 = ((ObjectTypeProperty)form2).typeValue;
				return (ot1.equals(ot2));
			}

			// if the formula is a simple graspable property
			else if (form1 instanceof GraspableProperty) {
				Graspable grasp1 = ((GraspableProperty)form1).graspableValue;
				Graspable grasp2 = ((GraspableProperty)form2).graspableValue;
				return (grasp1.equals(grasp2));
			}

			// if the formula is a simple location property
			else if (form1 instanceof LocationProperty) {
				String loc1 = ((LocationProperty)form1).location;
				String loc2 = ((LocationProperty)form2).location;
				return (loc1.equals(loc2));
			}

			// if the formula is a simple linguistic label property
			else if (form1 instanceof LinguisticLabelProperty) {
				String label1 = ((LinguisticLabelProperty)form1).label;
				String label2 = ((LinguisticLabelProperty)form2).label;
				return (label1.equals(label2));
			}

			// if the formula is a simple linguistic attribute property
			else if (form1 instanceof LinguisticAttributeProperty) {
				String attr1 = ((LinguisticAttributeProperty)form1).attribute;
				String attr2 = ((LinguisticAttributeProperty)form2).attribute;
				return (attr1.equals(attr2));
			}
			
			else if (form1 instanceof UnionRefProperty) {
				String ur1 = ((UnionRefProperty)form1).unionRef;
				String ur2 = ((UnionRefProperty)form2).unionRef;
				return (ur1.equals(ur2));
			}	
			
			else if (form1 instanceof SaliencyProperty) {
				Saliency s1 = ((SaliencyProperty)form1).sal;
				Saliency s2 = ((SaliencyProperty)form2).sal;
				return s1.equals(s2);
			}	
			
			else if (form1 instanceof ProximityProperty) {
				Proximity prox1 = ((ProximityProperty)form1).prox;
				Proximity prox2 = ((ProximityProperty)form2).prox;
				return (prox1.equals(prox2));
			}
			
			else {
				log("WARNING: Property unknown!");
				return false;
			}

		}
		else {
			return false;
		}
	}

	// ================================================================= 
	// PRETTY PRINT METHODS   
	// ================================================================= 



	/**
	 * Returns a string containing a string-formatted version of the belief model
	 * formula
	 */

	public static String getFormulaPrettyPrint(UncertainSuperFormula formula) {
		String result = "@(" ;

		result += getFormulaPrettyPrint(formula, 1);

		result += ")";

		return result;
	}


	/**
	 * Returns a string-formatted version of the logical operator included in form
	 * 
	 * @param form the complex formula
	 * @return the string
	 */

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

	/**
	 * Returns a string-formatted version of the uncertainty values contained in the
	 * formula, surrounded by brackets
	 * 
	 * @param formula the formula
	 * @return the string
	 */

	private static String getProbabilityValuePrettyPrint(UncertainSuperFormula formula) {
		String str = "";
		if (formula.prob > 0.0) {
			str += "[" + Math.round(formula.prob*100.0) / 100.0 + "]";
		}
		return str;
	}

	private static String getContinualStatusPrettyPrint(ContinualFormula formula) {
		String str = "";
		if (formula.cstatus != null) {
			str += "{" + formula.cstatus.toString() + "}";
		}
		else {
			str += "{CONT-STATUS?}";
		}
		return str;
	}
	
		
	
	private static String getAgentsPrettyPrint (Agent[] agents) {
		String result = "";
		for (int i = 0 ; i < agents.length; i++) {
			result += agents[i].id;
			if ( i < (agents.length - 1)) {
				result += ", ";
			}
		}
		return result;
	}
	
	public static String getAgentStatusPrettyPrint (AgentStatus status) {
	
		String result = "{" ;
		if (status instanceof PrivateAgentStatus) {
			result += ((PrivateAgentStatus)status).ag.id;
		}
		else if (status instanceof AttributedAgentStatus) {
			result += " ["  + ((AttributedAgentStatus)status).ag.id + "] " + ((AttributedAgentStatus)status).ag2.id;
		}
		else if (status instanceof MutualAgentStatus) {
			result += getAgentsPrettyPrint(((MutualAgentStatus)status).ags);
		}
		
		result += "}";
		return result;
	}
	
	public static String getSpatioTemporalFramePrettyPrint (SpatioTemporalFrame frame) {
		String result = "S_{"  + frame.id + "}";
		return result;
	}
	
	private static String getSpatioTemporalFramesPrettyPrint (SpatioTemporalFrame[] frames) {
		String result = "";
		for (int i = 0 ; i < frames.length; i++) {
			result += getSpatioTemporalFramePrettyPrint(frames[i]);
			if ( i < (frames.length - 1)) {
				result += ", ";
			}
		}
		return result;
	}
	
	public static String getBeliefPrettyPrint (Belief belief, int indent) {
		String result = getIndent(indent) + "Belief " + belief.id + " defined as \"" + "K_{" + belief.id + "} S_k A_k Phi\", with: \n" ;
		
		if (belief.sigma != null) {
		result += getIndent(indent+1) + "S_k spatio-temporal frame: " + getSpatioTemporalFramePrettyPrint(belief.sigma) + "\n";
		}
		if (belief.ags != null) {
		result += getIndent(indent+1) + "A_k agent status of the belief: " + getAgentStatusPrettyPrint (belief.ags) + " \n";
		}
		result += getIndent(indent+1) + "Phi formula incorporated in the belief, expressed as:\n";
		result += getIndent(indent+2) + "@(" ;
		result += getFormulaPrettyPrint((UncertainSuperFormula)belief.phi, indent+3);
		result += ")";
		
		return result;
	}
	
	public static String getBeliefModelPrettyPrint (BeliefModel bmodel, int indent) {
		String result = getIndent(indent) + "Belief model defined as tuple B = <A,S, K, T, F>, with: \n";
		
		result += getIndent(indent+1) + "A set of agents: {"  + getAgentsPrettyPrint(bmodel.a) + "}\n";
		
		result += getIndent(indent+1) + "S spatio-temporal model, defined on the set of spatio-temporal frames: {" + 
			getSpatioTemporalFramesPrettyPrint (bmodel.s.frames)+ "}\n";
		
		result += getIndent(indent+1) + "K set of private and/or mutual beliefs: {";
		for (int i = 0; i < bmodel.k.length; i++) {
			result += bmodel.k[i];
			if (i < (bmodel.k.length -1)) {
				result += ", ";
			}
		}
		result += "}\n";
		
		result += getIndent(indent+1) + "T set of tasks : {";  
		for (int i = 0; i < bmodel.t.length; i++) {
			result += bmodel.t[i];
			if (i < (bmodel.t.length -1)) {
				result += ", ";
			}
		}	
		result += "}\n";
		
		result += getIndent(indent+1) + "F set of foregrounded beliefs and tasks: {";
		for (int i = 0; i < bmodel.f.length; i++) {
			result += bmodel.f[i];
			if (i < (bmodel.f.length -1)) {
				result += ", ";
			}
		}	
		result += "}";
		
		return result;
	}
	
	/**
	 * Returns a string-formatted version of the formula, indented by depth
	 * 
	 * TODO: remove this hardcoded conversion between features and strongly typed belief formulae
	 * 
	 * @param formula the formula
	 * @param depth the indent to apply
	 * @return the string
	 */

	public static String getFormulaPrettyPrint(UncertainSuperFormula formula, int depth) {

		String result = formula.id;


		// If the formula is a complex formula, loop on its constituents and
		// call getFormulaPrettyPrint recursively
		if (formula instanceof ComplexFormula) {

			if (showProbabilitiesInPrettyPrint && 
					(formula.id.contains("unionconf-") || formula.id.contains("form-"))) {
				result +=  " ["  + formula.prob + "]";
			}

			result += " ^ \n" +  getIndent(depth) + "(";

			String operator = getOperatorPrettyPrint((ComplexFormula)formula);

			// loop on the formula constituents
			for (int i = 0; i < ((ComplexFormula)formula).formulae.length ; i++) {
				UncertainSuperFormula subformula = (UncertainSuperFormula) ((ComplexFormula)formula).formulae[i];

				result += getFormulaPrettyPrint(subformula, depth + 1);

				if (i < (((ComplexFormula)formula).formulae.length - 1)) {
					result += " " + operator + "\n" + getIndent(depth) + " ";
				}
			}
			result += ")";
		}

		// if the formula is a simple colour property
		else if (formula instanceof ColorProperty) {
			result += " ^ <Colour> " + ((ColorProperty)formula).colorValue;
			if (formula instanceof UncertainSuperFormula) {
				result += " " + getProbabilityValuePrettyPrint((UncertainSuperFormula)formula);
			}
		}

		// if the formula is a simple shape property
		else if (formula instanceof ShapeProperty) {
			result += " ^ <Shape> " + ((ShapeProperty)formula).shapeValue;
			if (formula instanceof UncertainSuperFormula) {
				result += " " + getProbabilityValuePrettyPrint((UncertainSuperFormula)formula);
			}
		}
		else if (formula instanceof ObjectTypeProperty) {
			result += " ^ <ObjectType> " + ((ObjectTypeProperty)formula).typeValue;	
			if (formula instanceof UncertainSuperFormula) {
				result += " " + getProbabilityValuePrettyPrint((UncertainSuperFormula)formula);
			}
	}

		// if the formula is a simple graspable property
		else if (formula instanceof GraspableProperty) {
			result += " ^ <Graspable> " + ((GraspableProperty)formula).graspableValue;	
			if (formula instanceof UncertainSuperFormula) {
				result += " " + getProbabilityValuePrettyPrint((UncertainSuperFormula)formula);
			}
		}

		// if the formula is a simple location property
		else if (formula instanceof LocationProperty) {
			result += " ^ <Location> " + ((LocationProperty)formula).location;	
			if (formula instanceof UncertainSuperFormula) {
				result += " " + getProbabilityValuePrettyPrint((UncertainSuperFormula)formula);
			}
		}

		// if the formula is a simple linguistic label property
		else if (formula instanceof LinguisticLabelProperty) {
			result += " ^ <LingLabel> " + ((LinguisticLabelProperty)formula).label;	
			if (formula instanceof UncertainSuperFormula) {
				result += " " + getProbabilityValuePrettyPrint((UncertainSuperFormula)formula);
			}
		}

		// if the formula is a simple linguistic attribute property
		else if (formula instanceof LinguisticAttributeProperty) {
			result += " ^ <LingAttribute> " + ((LinguisticAttributeProperty)formula).attribute;	
			if (formula instanceof UncertainSuperFormula) {
				result += " " + getProbabilityValuePrettyPrint((UncertainSuperFormula)formula);
			}
		}

		// if the formula is a simple linguistic attribute property
	/**	else if (formula instanceof BoundPhantomProxyProperty) {
			result += " ^ <BoundPhantom> " + ((BoundPhantomProxyProperty)formula).boundProxy;	
			if (formula instanceof UncertainSuperFormula) {
				result += " " + getProbabilityValuePrettyPrint((UncertainSuperFormula)formula);
			}
		} */
		
		else if (formula instanceof UnionRefProperty) {
			result += " ^ <UnionRef> " + ((UnionRefProperty)formula).unionRef;	
			if (formula instanceof UncertainSuperFormula) {
				result += " " + getProbabilityValuePrettyPrint((UncertainSuperFormula)formula);
			}
		}
		
		else if (formula instanceof SaliencyProperty) {
			result += " ^ <Saliency> " + ((SaliencyProperty)formula).sal;	
			if (formula instanceof UncertainSuperFormula) {
				result += " " + getProbabilityValuePrettyPrint((UncertainSuperFormula)formula);
			}
		} 
		
		else if (formula instanceof ProximityProperty) {
			result += " ^ <Proximity> " + ((ProximityProperty)formula).prox;	
			if (formula instanceof UncertainSuperFormula) {
				result += " " + getProbabilityValuePrettyPrint((UncertainSuperFormula)formula);
			}
		}

	/**	if (formula instanceof ContinualFormula) {
			result += " " + getContinualStatusPrettyPrint((ContinualFormula) formula);
		} */

		return result;
	}


	// ================================================================= 
	// UTILITY METHODS   
	// ================================================================= 

	/**
	 * Returns a string containing nbIndents spaces
	 * 
	 * @param nbIndents nb of spaces
	 * @return
	 */
	private static String getIndent(int nbIndents) {
		String str = "";
		for (int i = 0 ; i < nbIndents; i++) {
			str += "     ";
		}
		return str;
	}

	/**
	 * Logging utility
	 * @param str the string to output
	 */
	private static void log(String str) {
		if (LOGGING)
			System.out.println("[BeliefModelUtils] "  + str);
	}
}
