package de.dfki.lt.tr.cast.dialogue.util;

import java.util.ArrayList;
import java.util.List;

import de.dfki.lt.tr.beliefs.slice.logicalcontent.BooleanFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ElementaryFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.FloatFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.PointerFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.dFormula;

import autogen.Planner.Action;
import autogen.Planner.Link;
import autogen.Planner.POPlan;

public class POPlanUtils {
	
	public static String POPlanToString (POPlan _poPlan) {
		
		StringBuilder action_sb = new StringBuilder(22 * _poPlan.actions.length);
		for (Action _ac : _poPlan.actions) {
			action_sb.append("\nAction fullName: ");
			action_sb.append(_ac.fullName);
			action_sb.append(" status: ");
			action_sb.append(_ac.status);
		}
		
		List<String> listLinks = new ArrayList<String>();
		
		listLinks = POPlanUtils.extractLinks(_poPlan);
		
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
					link_sb.append(" " + ((ElementaryFormula)_df).prop);
					
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
