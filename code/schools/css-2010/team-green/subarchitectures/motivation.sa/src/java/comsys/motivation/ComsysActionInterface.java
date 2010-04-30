package comsys.motivation;

import java.awt.FlowLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.Map;

import javax.swing.JButton;
import javax.swing.JDialog;
import javax.swing.JTextArea;
import javax.swing.JTextField;

import beliefmodels.arch.BeliefException;
import beliefmodels.autogen.beliefs.Belief;
import beliefmodels.autogen.distribs.BasicProbDistribution;
import beliefmodels.autogen.distribs.CondIndependentDistribs;
import beliefmodels.autogen.distribs.FeatureValueProbPair;
import beliefmodels.autogen.distribs.FeatureValues;
import beliefmodels.autogen.distribs.ProbDistribution;
import beliefmodels.autogen.featurecontent.FeatureValue;
import beliefmodels.utils.FeatureContentUtils;
import binder.arch.BindingWorkingMemory;
import cast.CASTException;
import cast.ConsistencyException;
import cast.DoesNotExistOnWMException;
import cast.PermissionException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ManagedComponent;
import castutils.facades.BinderFacade;
import castutils.viewer.plugins.BeliefInfo;
import execution.slice.Action;
import execution.slice.TriBool;
import execution.slice.actions.ComsysQueryFeature;
import execution.slice.actions.ComsysTestFeatureValue;
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
		private String m_question;

		@Override
		public boolean accept(Action _action) {
			ComsysQueryFeature act = (ComsysQueryFeature) _action;
			m_beliefID = act.beliefID;
			m_featureID = act.featureID;
			m_question = act.question;
			return true;
		}

		@Override
		public void executeAction() {

			final Belief belief = m_binderFacade.getBelief(m_beliefID);

			final JDialog dialog = new JDialog();
			dialog.setLayout(new FlowLayout());
			String blurb;
			if (m_question.isEmpty()) {
			blurb = "what is the value of feature " + m_featureID
					+ " for belief " + m_beliefID + " of type " + belief.type
					+ "?";
			try { 
				Runtime rt = Runtime.getRuntime(); 
			    	Process p = rt.exec("espeak -s110" + "'Human._" + blurb + "_You_have_ten_seconds_to_comply.'");

			    	//p.waitFor();
			}
			catch(Exception e) { 
			    System.out.println(e.getMessage()); 
			}


			Map<String, ProbDistribution> distribs = ((CondIndependentDistribs) belief.content).distribs;
			for (String feature : distribs.keySet()) {
				blurb += "\n -> it already has " + feature + " with value ";
				FeatureValues values = (FeatureValues) ((BasicProbDistribution) distribs
						.get(feature)).values;
				blurb += BeliefInfo.toString(values.values.get(0).val);
			}
			}
			else {
				blurb=m_question;
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
                                        String featureValue = "NO_RECORD";
                                        try {
                                            FeatureContentUtils.addAnotherValueInBelief(belief, m_featureID, 
                                                                        new FeatureValueProbPair( new beliefmodels.autogen.featurecontent.StringValue(featureValue), 1f));

					    overwriteWorkingMemory(belief.id, BindingWorkingMemory.BINDER_SA, belief);
                                            executionComplete(TriBool.TRITRUE);

                                        } catch (Exception e) {
                                            logException(e);
                                            executionComplete(TriBool.TRIFALSE);
                                        }
					executionComplete(TriBool.TRITRUE);
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

	private class TestFeatureValueActionExecutor extends
			NonBlockingActionExecutor {

		private String m_featureType;
		private String m_beliefID;
		private FeatureValue m_featureValue;

		@Override
		public boolean accept(Action _action) {
			ComsysTestFeatureValue act = (ComsysTestFeatureValue) _action;
			m_beliefID = act.beliefID;
			m_featureType = act.featureType;
			m_featureValue = act.featureValue;
			return true;
		}

		@Override
		public void executeAction() {

			final Belief belief = m_binderFacade.getBelief(m_beliefID);

			final JDialog dialog = new JDialog();
			dialog.setLayout(new FlowLayout());
			String blurb = "Does belief " + m_beliefID + " of type " + belief.type
					+ " have a feature of type " + m_featureType
					+ " with value " + BeliefInfo.toString(m_featureValue)
					+ "?";

			Map<String, ProbDistribution> distribs = ((CondIndependentDistribs) belief.content).distribs;
			for (String feature : distribs.keySet()) {
				blurb += "\n -> it already has " + feature + " with value ";
				FeatureValues values = (FeatureValues) ((BasicProbDistribution) distribs
						.get(feature)).values;
				blurb += BeliefInfo.toString(values.values.get(0).val);
			}

			JTextArea textArea = new JTextArea(blurb);

			dialog.add(textArea);

			JButton yesButton = new JButton("Yes");
			JButton noButton = new JButton("No");
			JButton dontKnowButton = new JButton("Don't Know");

			yesButton.addActionListener(new ActionListener() {

				@Override
				public void actionPerformed(ActionEvent _e) {
					dialog.setVisible(false);
					try {
						addFeatureWithProb(belief, 1f);
						executionComplete(TriBool.TRITRUE);
					} catch (Exception e) {
						logException(e);
						executionComplete(TriBool.TRIFALSE);
					}
				}

			});

			noButton.addActionListener(new ActionListener() {

				@Override
				public void actionPerformed(ActionEvent _e) {
					dialog.setVisible(false);
					try {
						addFeatureWithProb(belief, 0f);
						executionComplete(TriBool.TRITRUE);
					} catch (Exception e) {
						logException(e);
						executionComplete(TriBool.TRIFALSE);
					}
				}

			});

			dontKnowButton.addActionListener(new ActionListener() {

				@Override
				public void actionPerformed(ActionEvent _e) {
					dialog.setVisible(false);
					executionComplete(TriBool.TRITRUE);
				}
			});

			dialog.add(yesButton);
			dialog.add(noButton);
			dialog.add(dontKnowButton);
			dialog.pack();
			dialog.setVisible(true);
		}

		/**
		 * @param belief
		 * @param _prob
		 * @throws BeliefException
		 * @throws UnknownSubarchitectureException
		 * @throws ConsistencyException
		 * @throws PermissionException
		 * @throws DoesNotExistOnWMException
		 */
		private void addFeatureWithProb(final Belief belief, float _prob)
				throws BeliefException, DoesNotExistOnWMException,
				PermissionException, ConsistencyException,
				UnknownSubarchitectureException {

			FeatureContentUtils.addAnotherValueInBelief(belief, m_featureType,
					new FeatureValueProbPair(m_featureValue, _prob));

			overwriteWorkingMemory(belief.id, BindingWorkingMemory.BINDER_SA,
					belief);

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
		m_actionStateManager.registerActionType(ComsysTestFeatureValue.class,
				new ActionExecutorFactory() {
					@Override
					public ActionExecutor getActionExecutor() {
						return new TestFeatureValueActionExecutor();
					}
				});
	}

}
