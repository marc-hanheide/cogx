package george.execution.components;

import autogen.Planner.Action;
import cast.CASTException;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ElementaryFormula;
import execution.components.BeliefBasedPlanExecutionMediator;
import execution.slice.ActionExecutionException;
import execution.slice.actions.AskForColour;
import execution.slice.actions.AskForIdentity;
import execution.slice.actions.AskForObjectWithFeatureValue;
import execution.slice.actions.AskForShape;
import execution.slice.actions.AskPolarColour;
import execution.slice.actions.AskPolarIdentity;
import execution.slice.actions.AskPolarShape;
import execution.slice.actions.LearnColour;
import execution.slice.actions.LearnIdentity;
import execution.slice.actions.LearnShape;
import execution.slice.actions.UnlearnColour;
import execution.slice.actions.UnlearnIdentity;
import execution.slice.actions.UnlearnShape;
import execution.util.ActionConverter;

// HACK Alen

import cast.cdl.WorkingMemoryAddress;
import cast.SubarchitectureComponentException;
import execution.util.SleepyThread;
import motivation.slice.PlanProxy;
import autogen.Planner.PlanningTask;
import autogen.Planner.Completion;
import execution.util.SerialPlanExecutor;
import cast.core.CASTUtils;
import cast.DoesNotExistOnWMException;
import cast.PermissionException;
import cast.ConsistencyException;
import cast.UnknownSubarchitectureException;
import eu.cogx.beliefs.slice.GroundedBelief;

import nu.xom.Document;
import nu.xom.Element;
import nu.xom.Node;
import nu.xom.Nodes;
import castutils.castextensions.IceXMLSerializer;
import VisionData.VisualConceptModelStatus;
/**
 * Execution mediator for George year 2.
 * 
 * @author nah
 * 
 */
public class GeorgeExecutionMediator extends BeliefBasedPlanExecutionMediator
		implements ActionConverter {

	public GeorgeExecutionMediator() {
	}

	/**
	 * Does the system specific work of converting a planning action into real
	 * system stuff.
	 * 
	 * @param _plannedAction
	 * @return
	 * @throws CASTException
	 */
	public execution.slice.Action toSystemAction(Action _plannedAction)
			throws CASTException {

		if (_plannedAction.name.equals("ask-for-an-objects-color-general")) {

			assert _plannedAction.arguments.length == 2 : "ask-for-an-objects-color-general is expected to be of arity 2";
			return createSingleBeliefAction(AskForColour.class,
					_plannedAction.arguments[1]);
		} else if (_plannedAction.name
				.equals("ask-for-an-objects-shape-general")) {

			assert _plannedAction.arguments.length == 2 : "ask-for-an-objects-shape-general is expected to be of arity 2";
			return createSingleBeliefAction(AskForShape.class,
					_plannedAction.arguments[1]);
		} else if (_plannedAction.name
				.equals("ask-for-an-objects-ident-general")) {

			assert _plannedAction.arguments.length == 2 : "ask-for-an-objects-ident-general is expected to be of arity 2";
			return createSingleBeliefAction(AskForIdentity.class,
					_plannedAction.arguments[1]);
		} else if (_plannedAction.name.equals("ask-for-an-objects-color-polar")) {

			assert _plannedAction.arguments.length == 3 : "ask-for-an-objects-color-polar is expected to be of arity 2";
			return createBeliefPlusStringAction(AskPolarColour.class,
					_plannedAction.arguments[1], _plannedAction.arguments[2]);
		} else if (_plannedAction.name.equals("ask-for-an-objects-shape-polar")) {

			assert _plannedAction.arguments.length == 3 : "ask-for-an-objects-shape-polar is expected to be of arity 2";
			return createBeliefPlusStringAction(AskPolarShape.class,
					_plannedAction.arguments[1], _plannedAction.arguments[2]);
		} else if (_plannedAction.name.equals("ask-for-an-objects-ident-polar")) {

			assert _plannedAction.arguments.length == 3 : "ask-for-an-objects-ident-polar is expected to be of arity 2";
			return createBeliefPlusStringAction(AskPolarIdentity.class,
					_plannedAction.arguments[1], _plannedAction.arguments[2]);
		} else if (_plannedAction.name.equals("learn-color")) {

			assert _plannedAction.arguments.length == 3 : "learn-color is expected to be of arity 3";
			return createBeliefPlusStringAction(LearnColour.class,
					_plannedAction.arguments[1], _plannedAction.arguments[2]);

		} else if (_plannedAction.name.equals("learn-shape")) {

			assert _plannedAction.arguments.length == 3 : "learn-shape is expected to be of arity 3";
			return createBeliefPlusStringAction(LearnShape.class,
					_plannedAction.arguments[1], _plannedAction.arguments[2]);

		} else if (_plannedAction.name.equals("learn-ident")) {

			assert _plannedAction.arguments.length == 3 : "learn-ident is expected to be of arity 3";
			return createBeliefPlusStringAction(LearnIdentity.class,
					_plannedAction.arguments[1], _plannedAction.arguments[2]);

		} else if (_plannedAction.name.equals("unlearn-color")) {

			assert _plannedAction.arguments.length == 3 : "unlearn-color is expected to be of arity 3";
			return createBeliefPlusStringAction(UnlearnColour.class,
					_plannedAction.arguments[1], _plannedAction.arguments[2]);
			
		} else if (_plannedAction.name.equals("unlearn-shape")) {
			assert _plannedAction.arguments.length == 3 : "unlearn-shape is expected to be of arity 3";
			return createBeliefPlusStringAction(UnlearnShape.class,
					_plannedAction.arguments[1], _plannedAction.arguments[2]);

		} else if (_plannedAction.name.equals("unlearn-ident")) {
			assert _plannedAction.arguments.length == 3 : "unlearn-ident is expected to be of arity 3";
			return createBeliefPlusStringAction(UnlearnIdentity.class,
					_plannedAction.arguments[1], _plannedAction.arguments[2]);
		}
		else if (_plannedAction.name.equals("ask-for-and-object-with-color")) {
			assert _plannedAction.arguments.length == 3 : "ask-for-and-object-with-color is expected to be of arity 3";
			return createAskForAction("color",
					(ElementaryFormula) _plannedAction.arguments[1]);
		} else if (_plannedAction.name.equals("ask-for-and-object-with-shape")) {
			assert _plannedAction.arguments.length == 3 : "ask-for-and-object-with-shape is expected to be of arity 3";
			return createAskForAction("shape",
					(ElementaryFormula) _plannedAction.arguments[1]);
		} else if (_plannedAction.name.equals("ask-for-and-object-with-ident")) {
			assert _plannedAction.arguments.length == 3 : "ask-for-and-object-with-ident is expected to be of arity 3";
			return createAskForAction("ident",
					(ElementaryFormula) _plannedAction.arguments[1]);
		}

		throw new ActionExecutionException("No conversion available for: "
				+ _plannedAction.fullName);
	}

	/**
	 * @param _plannedAction
	 * @return
	 * @throws CASTException
	 */
	private AskForObjectWithFeatureValue createAskForAction(String _feature,
			ElementaryFormula _value) throws CASTException {
		AskForObjectWithFeatureValue action = newActionInstance(AskForObjectWithFeatureValue.class);
		action.feature = _feature;
		action.value = stringFromElementaryFormula(_value);
		return action;
	}

	@Override
	public ActionConverter getActionConverter() {

		return this;
	}

	@Override
	public boolean isPaused() {
		// TODO Auto-generated method stub
		return false;
	}

	// HACK by Alen
	@Override	
	protected void newPlanProxy(WorkingMemoryAddress _planProxyAddr)
			throws SubarchitectureComponentException {
		log("newPlanProxy so creating new plan executor");

		PlanProxy pp = getMemoryEntry(_planProxyAddr, PlanProxy.class);
		PlanningTask pt = getMemoryEntry(pp.planAddress, PlanningTask.class);

		assert (pt.planningStatus == Completion.SUCCEEDED);

		// create and launch an executor
		SerialPlanExecutor executor = new SerialPlanExecutor(this,
				_planProxyAddr, getActionConverter());
		executor.startExecution();
		
		//HACK by Alen
		for(int i=0; i<pt.plan.length; i++)
			if (pt.plan[i].name.equals("ask-for-and-object-with-color"))
				new Ask4Interrupter(new WorkingMemoryAddress(pt.firstActionID,
					pp.planAddress.subarchitecture), "color").start();
			else if (pt.plan[i].name.equals("ask-for-and-object-with-shape"))
				new Ask4Interrupter(new WorkingMemoryAddress(pt.firstActionID,
					pp.planAddress.subarchitecture), "shape").start();

	}

	
	private class Ask4Interrupter extends SleepyThread {
		WorkingMemoryAddress m_actionAddress;
		String m_concept;

		public Ask4Interrupter(WorkingMemoryAddress _aa, String _c) {
			super(4000);
			m_actionAddress = _aa;
			m_concept = _c;
		}

		@Override
		public void doSomething() {
			try {
				println("action at: "
						+ CASTUtils.toString(m_actionAddress));
				lockComponent();
				Action ask4action = getMemoryEntry(m_actionAddress, Action.class);
				if(ask4action.name.equals("ask-for-and-object-with-" + m_concept)) {
					sleepComponent(1000);
					WorkingMemoryAddress beliefAddr = addressFromFormula(ask4action.arguments[2]);
					GroundedBelief gbelief = getMemoryEntry(beliefAddr, GroundedBelief.class);
					Document xmlDoc = IceXMLSerializer.toXomDom(gbelief);
					Nodes id =  xmlDoc.query("*/*/ancestors/*/address/id");
					Nodes sa =  xmlDoc.query("*/*/ancestors/*/address/subarchitecture");
					String val = xmlDoc.query("*/*/*/entry/*/*/*/*/*/prop").get(1).getValue();
					
					WorkingMemoryAddress msAddr = new WorkingMemoryAddress(id.get(0).getValue(), sa.get(0).getValue());
					VisualConceptModelStatus ms = getMemoryEntry(msAddr, VisualConceptModelStatus.class);
					
					for (int i = 0; i < ms.labels.length; i++) 
						if (ms.labels[i].equals(val))
							ms.askedFor[i]=true;
					
					overwriteWorkingMemory(msAddr, ms);
					
					ask4action.status = Completion.SUCCEEDED;
//					overwriteWorkingMemory(m_actionAddress, ask4action);
					deleteFromWorkingMemory(m_planAddress);
				}
//				else
//					new Ask4Interrupter(m_actionAddress, m_concept).start();
					
				unlockComponent();

			} catch (DoesNotExistOnWMException e) {
				e.printStackTrace();
			} catch (PermissionException e) {
				e.printStackTrace();
			} catch (ConsistencyException e) {
				e.printStackTrace();
			} catch (UnknownSubarchitectureException e) {
				e.printStackTrace();
			}
		}
	}
}
