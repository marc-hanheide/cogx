/**
 * 
 */
package motivation.components.abstr;

import motivation.idl.AcknowledgeMotive;
import motivation.idl.AnswerSet;
import motivation.idl.GainAttentionMotive;
import planning.autogen.Action;
import planning.autogen.PlanningStatus;
import BindingFeatures.Colour;
import BindingFeatures.Concept;
import BindingFeatures.RelationLabel;
import BindingFeatures.Shape;
import BindingQueries.FeatureAssertion;
import BindingQueries.FeatureRequest;
import balt.core.data.BALTType;
import cast.architecture.abstr.ChangeFilterFactory;
import cast.architecture.abstr.WorkingMemoryChangeReceiver;
import cast.architecture.subarchitecture.ConsistencyException;
import cast.architecture.subarchitecture.DoesNotExistOnWMException;
import cast.architecture.subarchitecture.ManagedProcess;
import cast.architecture.subarchitecture.SubarchitectureProcessException;
import cast.cdl.TriBool;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTUtils;

/**
 * 
 * Abstract class that loads planned actions and can respond to them when
 * complete.
 * 
 * @author nah
 * 
 */
public abstract class AbstractActionResponder extends ManagedProcess {

	/**
	 * @param _id
	 */
	public AbstractActionResponder(String _id) {
		super(_id);
	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.architecture.subarchitecture.ManagedProcess#taskAdopted(java.lang.String)
	 */
	@Override
	protected void taskAdopted(String _taskID) {

	}

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.architecture.subarchitecture.ManagedProcess#taskRejected(java.lang.String)
	 */
	@Override
	protected void taskRejected(String _taskID) {

	}

	@Override
	public void start() {
		super.start();

		try {
			addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(
					Action.class, WorkingMemoryOperation.ADD),
					new WorkingMemoryChangeReceiver() {
						public void workingMemoryChanged(
								WorkingMemoryChange _wmc) {
							newAction(_wmc.m_address.m_id);
						}
					});
		}
		catch (SubarchitectureProcessException e) {
			e.printStackTrace();
		}
	}

	/**
	 * Handle a new action added to our wm.
	 * 
	 * @param _actionID
	 */
	private void newAction(String _actionID) {
		try {
			// load the action
			Action action = (Action) getWorkingMemoryEntry(_actionID).getData();

			String actionType = action.m_action.m_type;

			// dumb case statement for now (or forever)
			if (actionType.equals(CASTUtils.typeName(AcknowledgeMotive.class))) {
				AcknowledgeMotive ackMotive = (AcknowledgeMotive) getWorkingMemoryEntry(
						action.m_action.m_address).getData();
				acknowledgeMotive(_actionID, ackMotive);
			}
			else if (actionType
					.equals(CASTUtils.typeName(FeatureRequest.class))) {
				FeatureRequest featReq = (FeatureRequest) getWorkingMemoryEntry(
						action.m_action.m_address).getData();
				featureRequest(_actionID, featReq);
			}
			else if (actionType.equals(CASTUtils
					.typeName(FeatureAssertion.class))) {
				FeatureAssertion featAss = (FeatureAssertion) getWorkingMemoryEntry(
						action.m_action.m_address).getData();
				featureAssertion(_actionID, featAss);
			}
			else if (actionType.equals(CASTUtils.typeName(AnswerSet.class))) {
				AnswerSet as = (AnswerSet) getWorkingMemoryEntry(
						action.m_action.m_address).getData();
				answerSet(_actionID, as);
			}
			else if (actionType.equals(CASTUtils
					.typeName(GainAttentionMotive.class))) {
				GainAttentionMotive attentionReq = (GainAttentionMotive) getWorkingMemoryEntry(
						action.m_action.m_address).getData();
				gainAttentionMotive(_actionID, attentionReq);
			}

			else {
				println("AbstractActionResponder not yet able to handle action type: "
						+ actionType);
				responseComplete(_actionID, TriBool.triFalse);
			}

		}
		catch (SubarchitectureProcessException e) {
			e.printStackTrace();
		}
	}

	protected abstract void answerSet(String _actionID, AnswerSet _as)
		throws SubarchitectureProcessException;
	
	/**
	 * Handle an GainAttentionMotive action. This is send when the planner
	 * requires to gain the attention of a person. The m_personId field of the
	 * GainAttentionMotive struct specifies the person to be trapped. The
	 * m_source field points to the binding proxies on motive wm that were used
	 * to create the motive.
	 * 
	 * @author Geert-Jan M. Kruijff (gj@dfki.de)
	 * @version 080828
	 * 
	 * @param _actionID
	 *            Unique ID for the action.
	 * @param _attMotive
	 *            The struct describing the gain attention motive.
	 * @throws SubarchitectureProcessException
	 */
	protected abstract void gainAttentionMotive(String _actionID,
			GainAttentionMotive _attMotive)
			throws SubarchitectureProcessException;

	private void featureRequest(String _actionID, FeatureRequest _featReq)
			throws SubarchitectureProcessException {
		// no address means it's a factual req
		if (_featReq.m_request.m_featurePointer.m_address.length() == 0) {

			Class<?> reqClass = BALTType
					.classFromType(_featReq.m_request.m_featurePointer.m_type);

			factualQuery(_actionID, _featReq.m_request.m_proxyID, reqClass,
					_featReq.m_fromSA);
		}
		else if (_featReq.m_request.m_featurePointer.m_type.equals(CASTUtils
				.typeName(RelationLabel.class))) {
			println("noop for relation label query");
			responseComplete(_actionID, TriBool.triFalse);

			String featureID = _featReq.m_request.m_featurePointer.m_address;

			// TODO, replace with proper binding config
			String featureSA = "binding.sa";

			// load relation label feature
			RelationLabel relationLabel = (RelationLabel) getWorkingMemoryEntry(
					featureID, featureSA).getData();

			factualRelationQuery(_actionID, _featReq.m_request.m_proxyID,
					relationLabel, _featReq.m_fromSA);

		}
		// otherwise it's polar
		else {
			throw new RuntimeException(
					"Someone needs to decode the polar query");
		}
	}

	/**
	 * Handle a factual query about a relation. E.g. "Where is the book". The
	 * relation label will have the label commonly used in binding (e.g.
	 * position or pos for position), and the proxy id will point to the known
	 * element of the relation. At the moment it's not clear whether this is
	 * "to" "from" or either... sorry! The proxyID will be a proxy for this
	 * subarch if one is available, else it will be from the asking SA.
	 * 
	 * @param _actionID
	 *            Unique ID for the action
	 * @param _proxyID
	 *            The proxy which is being asked about
	 * @param _relationLabel
	 *            The label for the relation feature
	 * @param _fromSA
	 *            The subarchitecture doing the asking
	 * @throws SubarchitectureProcessException 
	 * 
	 */
	protected abstract void factualRelationQuery(String _actionID,
			String _proxyID, RelationLabel _relationLabel, String _fromSA) throws SubarchitectureProcessException;

	private void featureAssertion(String _actionID, FeatureAssertion _featAss)
			throws SubarchitectureProcessException {

		// need to load the feature. this should be binding sa as that's the
		// only place feature pointers should point;)
		String bindingSA = _featAss.m_fromSA;

		// just check it's something we can deal with
		Class<?> reqClass = BALTType
				.classFromType(_featAss.m_assertion.m_featurePointer.m_type);

		// TODO make less ugly perhaps
		if (reqClass.equals(Colour.class)) {
			Colour feature = (Colour) getWorkingMemoryEntry(
					_featAss.m_assertion.m_featurePointer.m_address, bindingSA)
					.getData();
			factualAssertion(_actionID, _featAss.m_assertion.m_proxyID, feature);

		}
		else if (reqClass.equals(Shape.class)) {
			Shape feature = (Shape) getWorkingMemoryEntry(
					_featAss.m_assertion.m_featurePointer.m_address, bindingSA)
					.getData();
			factualAssertion(_actionID, _featAss.m_assertion.m_proxyID, feature);
		}
		else if (reqClass.equals(Concept.class)) {
			Concept feature = (Concept) getWorkingMemoryEntry(
					_featAss.m_assertion.m_featurePointer.m_address, bindingSA)
					.getData();
			factualAssertion(_actionID, _featAss.m_assertion.m_proxyID, feature);
		}

		else {
			throw new RuntimeException(
					"Someone needs to write the loading lines for feature: "
							+ reqClass);
		}

	}
	/**
	 * 
	 * Handle a factual assertion. This is sent is answer to a request for a
	 * binding feature for a particular proxy. The planner will use a proxy from
	 * this SA for the query.
	 * 
	 * @param _actionID
	 *            Unique ID for the action
	 * @param _proxyID
	 *            The proxy which is being asked about
	 * @param _feature
	 *            The binding feature being asserted for the proxy.
	 * @throws SubarchitectureProcessException
	 */
	protected abstract <FeatureT> void factualAssertion(String _actionID,
			String _proxyID, FeatureT _feature)
			throws SubarchitectureProcessException;

	/**
	 * 
	 * Handle a factual query. This is sent when the planner needs a value for a
	 * binding feature for a particular proxy. The planner will attempt to use a
	 * proxy from this SA for the query, but if no such proxy exists then the
	 * proxy from the sa that generated the request will be used.
	 * 
	 * @param _actionID
	 *            Unique ID for the action
	 * @param _proxyID
	 *            The proxy which is being asked about
	 * @param _queryClass
	 *            The class of the binding feature which is being asked about
	 * @param _fromSA
	 *            The subarchitecture which originally generated the query.
	 * @throws SubarchitectureProcessException
	 */
	protected abstract void factualQuery(String _actionID, String _proxyID,
			Class<?> _queryClass, String _fromSA)
			throws SubarchitectureProcessException;

	/**
	 * Handle an AcknowledgeMotive action. This is send when the planner and
	 * motive system first works on a motive. The m_ack field of the
	 * AcknowledgeMotive struct says whether the motive was accepted or
	 * rejected. The m_source field points to the binding proxies on motive wm
	 * that were used to create the motive.
	 * 
	 * @param _actionID
	 *            Unique ID for the action.
	 * @param _ackMotive
	 *            The struct describing the acknowledgement.
	 * @throws SubarchitectureProcessException
	 */
	protected abstract void acknowledgeMotive(String _actionID,
			AcknowledgeMotive _ackMotive)
			throws SubarchitectureProcessException;

	/**
	 * Signal the planner that we've completed handling the action.
	 * 
	 * @param _actionID
	 *            The unique ID for the action.
	 * @param _success
	 *            Whether the actions was handled successfully or not.
	 * @throws DoesNotExistOnWMException
	 * @throws ConsistencyException
	 * @throws SubarchitectureProcessException
	 */
	protected void responseComplete(String _actionID, TriBool _success)
			throws DoesNotExistOnWMException, ConsistencyException,
			SubarchitectureProcessException {
		Action action = (Action) getWorkingMemoryEntry(_actionID).getData();
		// now let the planner know we've handled the ack without problems
		action.m_succeeded = _success;
		// and that we'ere finised
		action.m_status = PlanningStatus.COMPLETE;
		overwriteWorkingMemory(_actionID, action);
		log("Action complete: " + CASTUtils.toString(_success) + " "
				+ _actionID);
	}

}
