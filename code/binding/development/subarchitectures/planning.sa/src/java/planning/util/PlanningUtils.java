package planning.util;

/**
 * 
 */

import java.util.ArrayList;

import planning.autogen.PlanningProblem;
import planning.autogen.PlanningStateLists;
import Planner.Command;
import Planner.Fact;
import Planner.GroundAction;
import Planner.ModalityEnum;
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

	public static Fact newFact(String _name, String[] _args, String _value) {
		return new Fact(ModalityEnum.NO_MODALITY, "", _name, _args, _value);
	}

	public static Fact newFact(String _name, String[] _args) {
		return new Fact(ModalityEnum.NO_MODALITY, "", _name, _args, "true");
	}

	public static boolean sameFactDifferentValue(Fact _f1, Fact _f2) {
		// are the names and arg list lenghts the same
		if ((!_f1.name.equals(_f2.name))
				|| (_f1.arguments.length != _f2.arguments.length)) {
			return false;
		}

		// are the arguments the same?
		for (int i = 0; i < _f1.arguments.length; i++) {
			if (!_f1.arguments[i].equals(_f2.arguments[i])) {
				return false;
			}
		}

		// are thej values the same?
		return !_f1.value.equals(_f2.value);

	}

	public static Fact[] changeState(Fact[] _state, Fact[] _changes) {
		ArrayList<Fact> newState = new ArrayList<Fact>(_state.length);
		boolean changed = false;
		for (Fact stateFact : _state) {
			changed = false;

			for (Fact changeFact : _changes) {
				if (sameFactDifferentValue(stateFact, changeFact)) {
					newState.add(changeFact);
					// System.out.println("replaced: " + toString(stateFact) + "
					// with " + toString(changeFact));
					changed = true;
					break;
				}
			}

			if (!changed) {
				// System.out.println("kept: " + toString(stateFact));
				newState.add(stateFact);
			}
		}

		return newState.toArray(new Fact[newState.size()]);

	}

	public static String toString(PlanningStateLists _state) {
		String out = "PlanningStateLists:\n";
		out += "\nObjects:\n";
		for (int i = 0; i < _state.m_objects.length; i++) {
			out += toString(_state.m_objects[i]) + "\n";
		}
		out += "\nFacts:\n";
		for (int i = 0; i < _state.m_facts.length; i++) {
			out += toString(_state.m_facts[i]) + "\n";
		}

		return out;
	}
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

		if (_fact.modality == ModalityEnum.K_MODALITY) {
			StringBuffer ksb = new StringBuffer("(K ");
			ksb.append(sb);
			ksb.append(")");
			return ksb.toString();
		}
		else {

			return sb.toString();
		}
	}

	public static String toString(PlanningTask _task) {

		StringBuffer sb = new StringBuffer("(ID: '");
		sb.append(_task.task_id);
		sb.append("'\n");
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

	public static String toString(Command _cmd) {
		StringBuffer sb = new StringBuffer("(");
		sb.append(toString(_cmd.mapl_action));
		for (String arg : _cmd.cmd_args) {
			sb.append(arg);
			sb.append(" ");
		}
		sb.append(")");
		return sb.toString();
	}

	private static String toString(GroundAction _action) {
		StringBuffer sb = new StringBuffer("(");
		sb.append(_action.name);
		sb.append(" ");
		for (String arg : _action.args) {
			sb.append(arg);
			sb.append(" ");
		}
		sb.append(")");
		return sb.toString();
	}

	public static boolean equals(Fact _f1, Fact _f2) {
		if ((!_f1.name.equals(_f2.name))
				|| (_f1.arguments.length != _f2.arguments.length)) {
			return false;
		}

		// are the arguments the same?
		for (int i = 0; i < _f1.arguments.length; i++) {
			if (!_f1.arguments[i].equals(_f2.arguments[i])) {
				return false;
			}
		}

		// are the values the same?
		return _f1.value.equals(_f2.value);

	}

	public static boolean equals(ObjectDeclaration _o1, ObjectDeclaration _o2) {
		if (_o1 == _o2) {
			return true;
		}
		else {
			if (_o1.name.equals(_o2.name)) {
				return _o1.type.equals(_o2.type);
			}
			else {
				return false;
			}
		}
	}

	// /**
	// * @param _state
	// * @return
	// */
	// public static String toString(TemporaryPlanningState _state) {
	// return toString(_state.toPlanningState());
	// }

}
