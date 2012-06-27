package george.execution.components;

import autogen.Planner.Action;
import cast.CASTException;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ElementaryFormula;
import execution.components.BeliefBasedPlanExecutionMediator;
import execution.slice.ActionExecutionException;
import execution.slice.ConfidenceLevel;
import execution.slice.actions.AnswerOpenQuestion;
import execution.slice.actions.AnswerPolarQuestion;
import execution.slice.actions.ArmToHomePos;
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
import execution.slice.actions.PointToObject;
import execution.slice.actions.UnlearnColour;
import execution.slice.actions.UnlearnIdentity;
import execution.slice.actions.UnlearnShape;
import execution.slice.actions.VerifyReference;
import execution.slice.actions.VerifyReferenceByFeatureValue;
import execution.slice.actions.george.yr3.AnalyzeProtoObject;
import execution.slice.actions.george.yr3.MoveToViewCone;
import execution.util.ActionConverter;

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

		if (_plannedAction.name.equals("move-to-viewcone")) {
			assert _plannedAction.arguments.length == 3 : "move-to-viewcone  is expected to be of arity 3";
			return createSingleBeliefAction(MoveToViewCone.class,
					_plannedAction.arguments[2]);
		} else if (_plannedAction.name.equals("analyze-proto-object")) {
			assert _plannedAction.arguments.length == 3 : "analyze-proto-object  is expected to be of arity 3";
			return createSingleBeliefAction(AnalyzeProtoObject.class,
					_plannedAction.arguments[2]);
		} else if (_plannedAction.name.equals("point-to-object")) {

			assert _plannedAction.arguments.length == 2 : "point-to-object is expected to be of arity 2";
			return createSingleBeliefAction(PointToObject.class,
					_plannedAction.arguments[1]);

		} else if (_plannedAction.name.equals("retract-arm")) {
			assert _plannedAction.arguments.length == 1 : "retract-arm is expected to be of arity 1";
			return newActionInstance(ArmToHomePos.class);

		} else if (_plannedAction.name
				.equals("ask-for-an-objects-color-general")) {

			assert _plannedAction.arguments.length == 2 : "ask-for-an-objects-color-general is expected to be of arity 2";
			return createSingleBeliefAction(AskForColour.class,
					_plannedAction.arguments[1]);
		} else if (_plannedAction.name
				.equals("ask-for-an-objects-shape-general")) {

			assert _plannedAction.arguments.length == 2 : "ask-for-an-objects-shape-general is expected to be of arity 2";
			return createSingleBeliefAction(AskForShape.class,
					_plannedAction.arguments[1]);
		} else if (_plannedAction.name
				.equals("ask-for-an-objects-objecttype-general")) {

			assert _plannedAction.arguments.length == 2 : "ask-for-an-objects-objecttype-general is expected to be of arity 2";
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
		} else if (_plannedAction.name.equals("ask-for-an-objects-objecttype-polar")) {

			assert _plannedAction.arguments.length == 3 : "ask-for-an-objects-objecttype-polar is expected to be of arity 2";
			return createBeliefPlusStringAction(AskPolarIdentity.class,
					_plannedAction.arguments[1], _plannedAction.arguments[2]);
		} else if (_plannedAction.name.equals("learn-color")) {

			assert _plannedAction.arguments.length == 3 : "learn-color is expected to be of arity 3";
			println("created a learn-color action");
			return createBeliefPlusStringAction(LearnColour.class,
					_plannedAction.arguments[1], _plannedAction.arguments[2]);

		} else if (_plannedAction.name.equals("learn-shape")) {

			assert _plannedAction.arguments.length == 3 : "learn-shape is expected to be of arity 3";
			return createBeliefPlusStringAction(LearnShape.class,
					_plannedAction.arguments[1], _plannedAction.arguments[2]);

		} else if (_plannedAction.name.equals("learn-objecttype")) {

			assert _plannedAction.arguments.length == 3 : "learn-objecttype is expected to be of arity 3";
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

		} else if (_plannedAction.name.equals("unlearn-objecttype")) {
			assert _plannedAction.arguments.length == 3 : "unlearn-objecttype is expected to be of arity 3";
			return createBeliefPlusStringAction(UnlearnIdentity.class,
					_plannedAction.arguments[1], _plannedAction.arguments[2]);
		} else if (_plannedAction.name.equals("ask-for-and-object-with-color")) {
			assert _plannedAction.arguments.length == 2 : "ask-for-and-object-with-color is expected to be of arity 2";
			return createAskForAction("color",
					(ElementaryFormula) _plannedAction.arguments[1]);
		} else if (_plannedAction.name.equals("ask-for-and-object-with-shape")) {
			assert _plannedAction.arguments.length == 2 : "ask-for-and-object-with-shape is expected to be of arity 2";
			return createAskForAction("shape",
					(ElementaryFormula) _plannedAction.arguments[1]);
		} else if (_plannedAction.name.equals("ask-for-and-object-with-objecttype")) {
			assert _plannedAction.arguments.length == 2 : "ask-for-and-object-with-objecttype is expected to be of arity 2";
			return createAskForAction("objecttype",
					(ElementaryFormula) _plannedAction.arguments[1]);
		} else if (_plannedAction.name
				.equals("answer-global-color-question-convinced")) {
			return createAnswerOpenAction(_plannedAction, "color",
					ConfidenceLevel.CONFIDENT);
		} else if (_plannedAction.name
				.equals("answer-global-color-question-believing")) {
			return createAnswerOpenAction(_plannedAction, "color",
					ConfidenceLevel.UNSURE);
		} else if (_plannedAction.name
				.equals("answer-global-color-question-unknown")) {
			return createAnswerOpenAction(_plannedAction, "color",
					ConfidenceLevel.UNKNOWN);
		} else if (_plannedAction.name
				.equals("answer-global-shape-question-convinced")) {
			return createAnswerOpenAction(_plannedAction, "shape",
					ConfidenceLevel.CONFIDENT);
		} else if (_plannedAction.name
				.equals("answer-global-shape-question-believing")) {
			return createAnswerOpenAction(_plannedAction, "shape",
					ConfidenceLevel.UNSURE);
		} else if (_plannedAction.name
				.equals("answer-global-shape-question-unknown")) {
			return createAnswerOpenAction(_plannedAction, "shape",
					ConfidenceLevel.UNKNOWN);
		} else if (_plannedAction.name
				.equals("answer-global-objecttype-question-convinced")) {
			return createAnswerOpenAction(_plannedAction, "objecttype",
					ConfidenceLevel.CONFIDENT);
		} else if (_plannedAction.name
				.equals("answer-global-objecttype-question-believing")) {
			return createAnswerOpenAction(_plannedAction, "objecttype",
					ConfidenceLevel.UNSURE);
		} else if (_plannedAction.name
				.equals("answer-global-objecttype-question-unknown")) {
			return createAnswerOpenAction(_plannedAction, "objecttype",
					ConfidenceLevel.UNKNOWN);
		} else if (_plannedAction.name
				.equals("answer-polar-color-question-positively")) {
			return createAnswerPolarAction(_plannedAction, "color",
					ConfidenceLevel.CONFIDENT);
		} else if (_plannedAction.name
				.equals("answer-polar-color-question-believing")) {
			return createAnswerPolarAction(_plannedAction, "color",
					ConfidenceLevel.UNSURE);
		} else if (_plannedAction.name
				.equals("answer-polar-color-question-negatively")) {
			return createAnswerPolarAction(_plannedAction, "color",
					ConfidenceLevel.UNKNOWN);
		} else if (_plannedAction.name
				.equals("answer-polar-shape-question-positively")) {
			return createAnswerPolarAction(_plannedAction, "shape",
					ConfidenceLevel.CONFIDENT);
		} else if (_plannedAction.name
				.equals("answer-polar-shape-question-believing")) {
			return createAnswerPolarAction(_plannedAction, "shape",
					ConfidenceLevel.UNSURE);
		} else if (_plannedAction.name
				.equals("answer-polar-shape-question-negatively")) {
			return createAnswerPolarAction(_plannedAction, "shape",
					ConfidenceLevel.UNKNOWN);
		} else if (_plannedAction.name
				.equals("answer-polar-objecttype-question-positively")) {
			return createAnswerPolarAction(_plannedAction, "objecttype",
					ConfidenceLevel.CONFIDENT);
		} else if (_plannedAction.name
				.equals("answer-polar-objecttype-question-believing")) {
			return createAnswerPolarAction(_plannedAction, "objecttype",
					ConfidenceLevel.UNSURE);
		} else if (_plannedAction.name
				.equals("answer-polar-objecttype-question-negatively")) {
			return createAnswerPolarAction(_plannedAction, "objecttype",
					ConfidenceLevel.UNKNOWN);
		} else if (_plannedAction.name.equals("verify-reference")) {
			assert _plannedAction.arguments.length == 2 : "verify-reference is expected to be of arity 2";
			return createSingleBeliefAction(VerifyReference.class,
					_plannedAction.arguments[1]);
		} else if (_plannedAction.name
				.equals("verify-reference-by-describing-its-color")) {
			assert _plannedAction.arguments.length == 3 : "verify-reference-by-describing-its-color is expected to be of arity 3";			
			return createBeliefFeatureValueAction(VerifyReferenceByFeatureValue.class, _plannedAction.arguments[1], "color", _plannedAction.arguments[2]);			
		}
		else if (_plannedAction.name
				.equals("verify-reference-by-describing-its-shape")) {
			assert _plannedAction.arguments.length == 3 : "verify-reference-by-describing-its-shape is expected to be of arity 3";			
			return createBeliefFeatureValueAction(VerifyReferenceByFeatureValue.class, _plannedAction.arguments[1], "shape", _plannedAction.arguments[2]);			
		}
		else if (_plannedAction.name
				.equals("verify-reference-by-describing-its-objecttype")) {
			assert _plannedAction.arguments.length == 3 : "verify-reference-by-describing-its-objecttype is expected to be of arity 3";			
			return createBeliefFeatureValueAction(VerifyReferenceByFeatureValue.class, _plannedAction.arguments[1], "objecttype", _plannedAction.arguments[2]);			
		}

		throw new ActionExecutionException("No conversion available for: "
				+ _plannedAction.fullName);
	}

	private AnswerOpenQuestion createAnswerOpenAction(Action _plannedAction,
			String _feature, ConfidenceLevel _confidence) throws CASTException {
		assert _plannedAction.arguments.length == 3 : "question answer actions are  expected to be of arity 3";
		AnswerOpenQuestion cmd = createBeliefFeatureValueAction(
				AnswerOpenQuestion.class, _plannedAction.arguments[1],
				_feature, (ElementaryFormula) _plannedAction.arguments[2]);
		cmd.confidence = _confidence;
		return cmd;
	}

	private AnswerPolarQuestion createAnswerPolarAction(Action _plannedAction,
			String _feature, ConfidenceLevel _confidence) throws CASTException {
		assert _plannedAction.arguments.length == 3 : "question answer actions are  expected to be of arity 3";
		AnswerPolarQuestion cmd = createBeliefFeatureValueAction(
				AnswerPolarQuestion.class, _plannedAction.arguments[1],
				_feature, (ElementaryFormula) _plannedAction.arguments[2]);
		cmd.confidence = _confidence;
		return cmd;
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
		return false;
	}

}
