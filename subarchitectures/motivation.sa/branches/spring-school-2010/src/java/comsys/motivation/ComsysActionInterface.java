package comsys.motivation;

import java.awt.FlowLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.Map;

import javax.swing.JButton;
import javax.swing.JDialog;
import javax.swing.JTextArea;
import javax.swing.JTextField;

import beliefmodels.autogen.beliefs.Belief;
import beliefmodels.autogen.distribs.BasicProbDistribution;
import beliefmodels.autogen.distribs.CondIndependentDistribs;
import beliefmodels.autogen.distribs.DistributionValues;
import beliefmodels.autogen.distribs.FeatureValueProbPair;
import beliefmodels.autogen.distribs.FeatureValues;
import beliefmodels.autogen.distribs.ProbDistribution;
import beliefmodels.autogen.featurecontent.FeatureValue;
import beliefmodels.autogen.featurecontent.StringValue;
import beliefmodels.utils.FeatureContentUtils;
import binder.arch.BindingWorkingMemory;
import cast.architecture.ManagedComponent;
import castutils.facades.BinderFacade;
import castutils.viewer.plugins.BeliefInfo;
import execution.slice.Action;
import execution.slice.TriBool;
import execution.slice.actions.ComsysQueryFeature;
import execution.util.ActionExecutor;
import execution.util.ActionExecutorFactory;
import execution.util.LocalActionStateManager;
import execution.util.NonBlockingActionExecutor;

public class ComsysActionInterface extends ManagedComponent {

	private LocalActionStateManager m_actionStateManager;
	private final BinderFacade m_binderFacade;

	@Override
	protected void configure(Map<String, String> _config) {
	}

	public ComsysActionInterface() {
		m_binderFacade = new BinderFacade(this);
	}

	private class FeatureQueryActionExecutor extends NonBlockingActionExecutor {

		private String m_featureID;
		private String m_beliefID;

		@Override
		public boolean accept(Action _action) {
			ComsysQueryFeature act = (ComsysQueryFeature) _action;
			m_beliefID = act.beliefID;
			m_featureID = act.featureID;
			return true;
		}

		@Override
		public void executeAction() {

			final Belief belief = m_binderFacade.getBelief(m_beliefID);

			final JDialog dialog = new JDialog();
			dialog.setLayout(new FlowLayout());
			String blurb = "what is the value of feature " + m_featureID
					+ " for belief " + m_beliefID;

			Map<String, ProbDistribution> distribs = ((CondIndependentDistribs) belief.content).distribs;
			for (String feature : distribs.keySet()) {
				blurb += "\n -> it already has " + feature + " with value ";
				FeatureValues values = (FeatureValues) ((BasicProbDistribution) distribs
						.get(feature)).values;
				blurb += BeliefInfo.toString(values.values.get(0).val);
			}

			JTextArea textArea = new JTextArea(blurb);

			dialog.add(textArea);
			final JTextField textfield = new JTextField(30);
			dialog.add(textfield);
			JButton goButton = new JButton("Answer");

			ActionListener submit = new ActionListener() {

				@Override
				public void actionPerformed(ActionEvent _e) {
					dialog.setVisible(false);
					String featureValue = textfield.getText();
					if (featureValue.isEmpty()) {
						executionComplete(TriBool.TRIFALSE);
					} else {
						try {

							FeatureContentUtils
									.addAnotherValueInBelief(
											belief,
											m_featureID,
											new FeatureValueProbPair(
													new beliefmodels.autogen.featurecontent.StringValue(
															featureValue), 1f));

							overwriteWorkingMemory(belief.id,
									BindingWorkingMemory.BINDER_SA, belief);

							executionComplete(TriBool.TRITRUE);

						} catch (Exception e) {
							logException(e);
							executionComplete(TriBool.TRIFALSE);
						}

					}
				}
			};

			goButton.addActionListener(submit);
			textfield.addActionListener(submit);

			JButton dontKnowButton = new JButton("Don't know");
			dontKnowButton.addActionListener(new ActionListener() {

				@Override
				public void actionPerformed(ActionEvent _e) {
					dialog.setVisible(false);
					executionComplete(TriBool.TRIFALSE);

				}
			});

			dialog.add(goButton);
			dialog.add(dontKnowButton);
			dialog.pack();
			dialog.setVisible(true);
		}

		@Override
		public void stopExecution() {
			// TODO Auto-generated method stub

		}

	}

	@Override
	protected void start() {
		m_binderFacade.start();
		m_actionStateManager = new LocalActionStateManager(this);
		m_actionStateManager.registerActionType(ComsysQueryFeature.class,
				new ActionExecutorFactory() {
					@Override
					public ActionExecutor getActionExecutor() {
						return new FeatureQueryActionExecutor();
					}
				});
	}

}
