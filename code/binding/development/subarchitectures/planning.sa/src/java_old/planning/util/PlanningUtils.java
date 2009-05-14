package planning.util;

/**
 * 
 */

import planning.autogen.PlanningProblem;
import Planner.Fact;
import Planner.ObjectDeclaration;
import Planner.PlanningTask;

/**
 * @author nah
 */
public class PlanningUtils {

	/**
	 * @param _planningProblem
	 * @return
	 */
	public static String toString(PlanningProblem _planningProblem) {
		String out = "PlanningProblem:\n";

		out += _planningProblem.m_maplGoal;
		out += "\nObjects:\n";
		for (int i = 0; i < _planningProblem.m_objectList.length; i++) {
			out += _planningProblem.m_objectList[i].name + " : "
					+ _planningProblem.m_objectList[i].type + "\n";
		}
		out += "\nFacts:\n";
		for (int i = 0; i < _planningProblem.m_factList.length; i++) {
			out += _planningProblem.m_factList[i] + "\n";
		}

		return out;
	}

	// public static String toString(PlanningState _state) {
	// String out = "PlanningState:\n";
	//
	// out += "\nObjects:\n";
	// for (int i = 0; i < _state.m_objects.length; i++) {
	// out += _state.m_objects[i].name + " : " + _state.m_objects[i].type
	// + "\n";
	// }
	// out += "\nFacts:\n";
	// for (int i = 0; i < _state.m_facts.length; i++) {
	// out += _state.m_facts[i] + "\n";
	// }
	//
	// return out;
	// }
	//
	// /**
	// * @param _plan
	// * @return
	// */
	// public static String toString(Plan _plan) {
	// String out = "Plan:\n";
	// for (int i = 0; i < _plan.m_plan.length; i++) {
	// out += i + ": " + _plan.m_plan[i] + "\n";
	// }
	// return out;
	// }

	/**
	 * @param _decl
	 * @return
	 */
	public static String toString(ObjectDeclaration _decl) {
		return "(" + _decl.name + " - " + _decl.type + ")";
	}

	/**
	 * @param _decl
	 * @return
	 */
	public static String toString(Fact _fact) {
		StringBuffer sb = new StringBuffer("(");
		sb.append(_fact.name);
		sb.append(" ");
		for (String arg : _fact.arguments) {
			sb.append(arg);
			sb.append(" ");
		}
		sb.append(": ");
		sb.append(_fact.value);
		sb.append(")");
		return sb.toString();
	}

	public static String toString(PlanningTask _task) {

		StringBuffer sb = new StringBuffer("(");
		sb.append(_task.task_id);
		sb.append("\n");
		sb.append(_task.domain_fn);
		sb.append("\n");
		sb.append(_task.domain_name);
		sb.append("\n");
		sb.append(_task.planning_agent);
		sb.append("\n");
		sb.append("GOALS:\n");
		for (String arg : _task.goals) {
			sb.append(arg);
			sb.append("\n");
		}
		sb.append("OBJECTS:\n");

		for (ObjectDeclaration arg : _task.objects) {
			sb.append(PlanningUtils.toString(arg));
			sb.append("\n");
		}
		sb.append("FACTS:\n");

		for (Fact arg : _task.facts) {
			sb.append(PlanningUtils.toString(arg));
			sb.append("\n");
		}

		return sb.toString();
	}

	// /**
	// * @param _state
	// * @return
	// */
	// public static String toString(TemporaryPlanningState _state) {
	// return toString(_state.toPlanningState());
	// }

}
