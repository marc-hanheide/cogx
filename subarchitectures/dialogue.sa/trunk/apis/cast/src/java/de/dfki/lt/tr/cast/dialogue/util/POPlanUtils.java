package de.dfki.lt.tr.cast.dialogue.util;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

import de.dfki.lt.tr.beliefs.slice.logicalcontent.BooleanFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ElementaryFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.FloatFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.PointerFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;
import de.dfki.lt.tr.planverb.history.Step;

import autogen.Planner.Action;
import autogen.Planner.Link;
import autogen.Planner.POPlan;

public class POPlanUtils {
	
	public static String POPlanToString (POPlan _poPlan) {
		
		List<Step<String>> listSteps = POPlanUtils.extractSteps(_poPlan);
		StringBuilder action_sb = new StringBuilder(22 * _poPlan.actions.length);
		for (Step<String> s : listSteps) {
			action_sb.append(s.getStep() + "\n");
		}
		
		List<String> listLinks = POPlanUtils.extractLinks(_poPlan);
		StringBuilder link_sb = new StringBuilder(42 * _poPlan.links.length);
		for (String s : listLinks) {
			link_sb.append(s + "\n");
		}

		StringBuilder return_sb = new StringBuilder(40 + action_sb.length() + link_sb.length());
		return_sb.append("processAddedPOPlan() called: \n");
		return_sb.append("Task ID = ");
		return_sb.append(_poPlan.taskID);
		return_sb.append("\nStatus name = ");
		return_sb.append(_poPlan.status.name());
		return_sb.append("\nActions: ");
		return_sb.append(action_sb);
		return_sb.append("\nLinks: \n");
		return_sb.append(link_sb);
		
		return return_sb.toString();
	}
	
	/**
	 * This is a helper method for hacking around the planner bug that sometimes
	 * causes ElementaryFormulas (ElFo) appearing as arguments where PointerFormulas (PoFo) 
	 * are expected.
	 * 
	 * It expects as input the prop(osition) of the ElFo, and returns a String rendering
	 * correspoding to a WMA derived from a PoFo.
	 * 
	 * E.g., it turns 'place_2__b' into '2:B@spatial.sa'
	 * NB This is an ugly hack!
	 * 
	 * @param prop - prop(osition) of the ElFo (e.g. place_2__b)
	 * @return a String correspoding to a WMA (e.g. 2:B@spatial.sa)
	 */
	public static String transformElFoPropToPoFoWMA(String prop) {
		String pointerID = prop.replace("place_", "");
		pointerID = pointerID.replace("__", ":");
		pointerID = pointerID.toUpperCase();
		return pointerID +  "@" + "spatial.sa";
	}
	
	
	public static List<Step<String>> extractSteps (POPlan _poPlan) {
		List<Step<String>> actionList = new LinkedList<Step<String>>();

		for (int i = 0; i < _poPlan.actions.length; i++) {
			dFormula[] allArgs = _poPlan.actions[i].allArguments;
			String args = "";
			for (int j = 0; j < allArgs.length; j++) {
				dFormula _df = allArgs[j];
				if (_df instanceof PointerFormula) {
					if (((PointerFormula)_df).pointer != null) {
						String wmp = ((PointerFormula)_df).pointer.id +  "@" + ((PointerFormula)_df).pointer.subarchitecture;
						args = args + " " + wmp;
					} 
				} else if (_df instanceof ElementaryFormula) {
					String origProp = ((ElementaryFormula)_df).prop;
					if (origProp != null) {
						if (origProp.startsWith("place")) {
							String wmp = POPlanUtils.transformElFoPropToPoFoWMA(origProp);
							args = args + " " + wmp;
						} else {
							args = args + " " + origProp;
						}
					} 
				} else {
					// TODO handle this case
					// log("current argument with index " + j + " is neither of type PointerFormular nor ElementaryFormula, but of type " + _df.ice_id());
				}
			}
			String currStepLine = new Integer(i).toString()+ ": " + _poPlan.actions[i].status.name()  + " " + _poPlan.actions[i].name + args;
			actionList.add(new Step<String>(currStepLine.trim()));
		}
		return actionList;
	}
	
	public static List<String> extractLinks (POPlan _poPlan) {
		List<String> linksList = new ArrayList<String>();
		
		StringBuilder link_sb = new StringBuilder(100);
		
		for (Link _ln : _poPlan.links) {
			link_sb.append(_ln.src + " " + _ln.dest + " " + _ln.type.name() + " " + _ln.reason.name);
			
			for (dFormula _df : _ln.reason.arguments) {
				
				if (_df instanceof PointerFormula) {
					if (((PointerFormula)_df).pointer != null) {
						link_sb.append(" " + ((PointerFormula)_df).pointer.id);
						link_sb.append("@");
						link_sb.append(((PointerFormula)_df).pointer.subarchitecture);
					}	
				} else if (_df instanceof ElementaryFormula) {
					link_sb.append(" " + transformElFoPropToPoFoWMA(((ElementaryFormula)_df).prop));
					
				}
				else {
					link_sb.append("\nArgument has a special type.");
					link_sb.append("\n" + _df.getClass().toString());
				}
			}
			
			for (dFormula _df : _ln.reason.modalArguments) {
				if (_df instanceof PointerFormula) {
					link_sb.append(" MODALITY: " + _ln.reason.modality);
					if (((PointerFormula)_df).pointer != null) {
						link_sb.append(" " + ((PointerFormula)_df).pointer.id);
						link_sb.append("@");
						link_sb.append(((PointerFormula)_df).pointer.subarchitecture);
					}
				} else if (_df instanceof ElementaryFormula) {
					link_sb.append(" MODALITY: " + _ln.reason.modality);
					
				} else if (_df instanceof BooleanFormula) {
					link_sb.append(" MODALITY: " + _ln.reason.modality);
					link_sb.append(" " + ((BooleanFormula)_df).val);
				}
				else {
					link_sb.append("\nModalArgument has a special type.");
					link_sb.append("\n" + _df.getClass().toString());
				}
			}
			
			if (_ln.reason.value instanceof BooleanFormula) {
				link_sb.append(" VALUE: " + ((BooleanFormula)_ln.reason.value).val);
				
			} else if (_ln.reason.value instanceof PointerFormula) {
				if (((PointerFormula)_ln.reason.value).pointer != null) {
					link_sb.append(" VALUE: " + ((PointerFormula)_ln.reason.value).pointer.id);
					link_sb.append("@");
					link_sb.append(((PointerFormula)_ln.reason.value).pointer.subarchitecture);
				}
			} else if (_ln.reason.value instanceof ElementaryFormula) {
				link_sb.append(" VALUE: " + ((ElementaryFormula)_ln.reason.value).prop);
				
			} else if (_ln.reason.value instanceof FloatFormula) {
				link_sb.append(" VALUE: " + ((FloatFormula)_ln.reason.value).val);
				
			} else {
				link_sb.append("\nArgument has a special type.");
				link_sb.append("\n" + _ln.reason.value.getClass().toString());
			}
			
			linksList.add(link_sb.toString());
			link_sb.setLength(0);
		}
		return linksList;
	}
}
