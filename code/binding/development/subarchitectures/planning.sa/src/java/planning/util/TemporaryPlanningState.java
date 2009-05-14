/**
 * 
 */
package planning.util;

import java.util.ArrayList;
import java.util.Set;
import java.util.TreeSet;

import planning.autogen.PlanningStateLists;
import Planner.Fact;
import Planner.ObjectDeclaration;

/**
 * helper struct used to hold lists of things for planning. Based on array lists
 * to avoid the horrors of arrays for idl. Later will be written to wm as a
 * {@link TemporaryPlanningState} struct.
 * 
 * @author nah
 */
public class TemporaryPlanningState {

	// make sure these are synced just in case
	public Set<Fact> m_factList = new TreeSet<Fact>(new FactComparator());
	public Set<ObjectDeclaration> m_objectList = new TreeSet<ObjectDeclaration>(
			new ObjectDeclarationComparator());

	public TemporaryPlanningState(PlanningStateLists _additionalState) {
		this();
		extend(_additionalState);
	}

	public TemporaryPlanningState() {
	}

	public PlanningStateLists toPlanningState() {
		PlanningStateLists state = new PlanningStateLists();

		state.m_facts = new Fact[m_factList.size()];
		m_factList.toArray(state.m_facts);

		state.m_objects = new ObjectDeclaration[m_objectList.size()];
		m_objectList.toArray(state.m_objects);

		return state;
	}

	/**
	 * Extend the given state with the contents of this state
	 * 
	 * @param _state
	 */
	public void toPlanningState(PlanningStateLists _state) {

		{// copy facts to temp
			ArrayList<Fact> tmpFacts = new ArrayList<Fact>();
			for (Fact string : _state.m_facts) {
				tmpFacts.add(string);
			}
			tmpFacts.addAll(m_factList);
			_state.m_facts = new Fact[tmpFacts.size()];
			tmpFacts.toArray(_state.m_facts);
		}
		{
			ArrayList<ObjectDeclaration> tmpDecls = new ArrayList<ObjectDeclaration>();
			for (ObjectDeclaration decl : _state.m_objects) {
				tmpDecls.add(decl);
			}
			tmpDecls.addAll(m_objectList);
			_state.m_objects = new ObjectDeclaration[tmpDecls.size()];
			tmpDecls.toArray(_state.m_objects);
		}
	}

	/**
	 * Extend this staet with the contents of the given state.
	 * 
	 * @param _state
	 */
	public void extend(PlanningStateLists _state) {
		for (ObjectDeclaration decl : _state.m_objects) {
			m_objectList.add(decl);
		}
		for (Fact fact : _state.m_facts) {
			m_factList.add(fact);
		}
	}

	public void extend(ObjectDeclaration[] _objects, Fact[] _facts) {
		for (ObjectDeclaration decl : _objects) {
			m_objectList.add(decl);
		}
		for (Fact fact : _facts) {
			m_factList.add(fact);
		}
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see java.lang.Object#toString()
	 */
	@Override
	public String toString() {
		return PlanningUtils.toString(toPlanningState());
	}

}
