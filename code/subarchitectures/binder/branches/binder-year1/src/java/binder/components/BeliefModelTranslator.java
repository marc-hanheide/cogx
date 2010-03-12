package binder.components;

import java.util.Enumeration;
import java.util.HashMap;
import java.util.Vector;

import beliefmodels.adl.Agent;
import beliefmodels.adl.AgentStatus;
import beliefmodels.adl.Belief;
import beliefmodels.adl.BeliefModel;
import beliefmodels.adl.Formula;
import beliefmodels.adl.Perspective;
import beliefmodels.adl.PrivateAgentStatus;
import beliefmodels.adl.SpatialInterval;
import beliefmodels.adl.SpatioTemporalFrame;
import beliefmodels.adl.SpatioTemporalModel;
import beliefmodels.adl.TemporalInterval;
import beliefmodels.domainmodel.cogx.ComplexFormula;
import beliefmodels.domainmodel.cogx.GroundedBelief;
import beliefmodels.domainmodel.cogx.LogicalOp;
import beliefmodels.domainmodel.cogx.Saliency;
import beliefmodels.domainmodel.cogx.SaliencyProperty;
import beliefmodels.domainmodel.cogx.SuperFormula;
import beliefmodels.domainmodel.cogx.UncertainSuperFormula;
import binder.autogen.core.Feature;
import binder.autogen.core.Union;
import binder.autogen.core.UnionConfiguration;
import binder.autogen.featvalues.AddressValue;
import binder.autogen.specialentities.PhantomProxy;
import binder.utils.BeliefModelUtils;
import binder.utils.BinderUtils;
import binder.utils.GenericUtils;
import cast.DoesNotExistOnWMException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.CASTTime;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTData;

/**
 * Translate the union configurations on the binder working memory into belief models
 * 
 * @author Pierre Lison
 * @version 15/09/2009 (version 21/09/2009)
 */
public class BeliefModelTranslator extends ManagedComponent {


	public final Agent robotAgent = new Agent("robot");
	public final Agent humanAgent = new Agent("human");
	public SpatioTemporalFrame curFrame = new SpatioTemporalFrame();

	public final float SALIENCY_THRESHOLD = 0.5f;

	HashMap<String, Belief> currentBeliefs = new HashMap<String, Belief>() ;
	
	// =================================================================
	// INITIALISATION
	// =================================================================

 
	public void initializeAgentAndFrame () {
		robotAgent.id = "robot";

		String id = "here-and-now";
		Agent[] ags = new Agent[1];
		ags[0] = robotAgent;
		Perspective persp = new Perspective("robotperspective", ags);
		SpatialInterval spatialint = new SpatialInterval("here");
		TemporalInterval tempint = new TemporalInterval("now", getCASTTime().toString(), getCASTTime().toString());
		
		curFrame = new SpatioTemporalFrame(id, spatialint, tempint, persp);
	}


	/**
	 * Add a change filter for union configurations on the binder working memory
	 * 
	 * TODO: update from unionconfiguration --> alternativeunionconfigurations
	 */
	@Override
	public void start() {

		// if the set of possible union configurations has been updated, update the
		// monitor accordingly

		// TODO: extend to handle ambiguities
		// TODO: implement a garbase collector for outdated beliefs (which are not supported in the current config)
		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(UnionConfiguration.class,
				WorkingMemoryOperation.WILDCARD), new WorkingMemoryChangeReceiver() {

			public void workingMemoryChanged(WorkingMemoryChange _wmc) {
				try {
					log("-------- START BELIEF MODEL TRANSLATION ----------");
					// Retrieve the union configurations
					// EASY CASE: assuming no ambiguity
					UnionConfiguration uc = 
						getMemoryEntry(_wmc.address, UnionConfiguration.class);

					// Translate the union configuration into a belief model
					Vector<Belief> beliefs = extractBeliefs(uc);

					// add non-binder beliefs
					beliefs.addAll(getNonBinderBeliefs());
										
					for (Enumeration<Belief> e = beliefs.elements(); e.hasMoreElements(); ) {
						Belief curB = e.nextElement();
						if (isUpdateNeeded(curB)) {
							updateBeliefInWM(curB);
							currentBeliefs.put(curB.id, curB);
							log("Updated belief : \n" + BeliefModelUtils.getBeliefPrettyPrint(curB, 1));
							log("----------------------");
						}
						else {
							log("belief update not necessary");
						}
					}

					
					CASTData<Belief>[] beliefsInWM = getWorkingMemoryEntries(Binder.BINDER_SA, Belief.class);
					for (int i = 0 ; i < beliefsInWM.length ; i++) {
						Belief b = beliefsInWM[i].getData();
						if (!containsBelief(beliefs, b)) {
							log("Belief to delete:" + BeliefModelUtils.getBeliefPrettyPrint(b, 1));
							deleteFromWorkingMemory(beliefsInWM[i].getID(), Binder.BINDER_SA);
						}
					}
					

					String id = "";
					CASTData<BeliefModel>[] bmodels = getWorkingMemoryEntries(Binder.BINDER_SA, BeliefModel.class);
					if (bmodels.length > 0) {
						id = bmodels[0].getData().id;
					}
					else {
						id = newDataID();
					}
					BeliefModel bmodel = constructBeliefModel (beliefs, id);
					updateBeliefModelInWM(bmodel);
					
					log("Updated belief model: \n" + BeliefModelUtils.getBeliefModelPrettyPrint(bmodel, 1));
					log("-------- STOP BELIEF MODEL TRANSLATION ----------");

				}
				catch (Exception e) {
					e.printStackTrace();
				}
			} 
		});

		initializeAgentAndFrame();
		
		log("Belief model translator successfully initialized");
	}



	private boolean containsBelief (Vector<Belief> beliefs, Belief belief) {
		for (int i = 0 ; i < beliefs.size(); i++) {
			Belief curBelief = beliefs.elementAt(i);
			if (curBelief.id.equals(belief.id)) {
				return true;
			}
		}
		return false;
	}

	// =================================================================
	// METHODS FOR TRANSLATING UNION CONFIGURATIONS INTO BELIEF MODELS
	// =================================================================


	
	private String[] constructForeground (Vector<Belief> allBeliefs) {
		
		Vector<String> foregroundedBeliefsV = new Vector<String>();
		
		for (int i = 0; i < allBeliefs.size(); i++) {
			Belief curB = allBeliefs.elementAt(i);
			Saliency saliency = BeliefModelUtils.getSaliencyValue (curB);
			
			if (saliency.equals(Saliency.high)) {
				foregroundedBeliefsV.add(curB.id);
			}
		}
		
		String[] foregroundedBeliefs = new String [foregroundedBeliefsV.size()];
		foregroundedBeliefs = foregroundedBeliefsV.toArray(foregroundedBeliefs);
		
		return foregroundedBeliefs;
	}
	
	
	private Vector<Belief> getNonBinderBeliefs() {
		
		log("now getting non-binder beliefs...");
		Vector<Belief> nonbinderbeliefs = new Vector<Belief>();
		
		try {
			CASTData<Belief>[] beliefs = getWorkingMemoryEntries (Binder.BINDER_SA, Belief.class);
			
			for (int i = 0; i < beliefs.length ; i++ ) {
				Belief belief = beliefs[i].getData();
				if (belief.id.contains("cca")) {
					nonbinderbeliefs.add(belief);
				}
			}
			
			CASTData<GroundedBelief>[] gbeliefs = getWorkingMemoryEntries (Binder.BINDER_SA, GroundedBelief.class);
			
			for (int i = 0; i < gbeliefs.length ; i++ ) {
				Belief gbelief = gbeliefs[i].getData();
				if (gbelief.id.contains("cca")) {
					nonbinderbeliefs.add(gbelief);
				}
			}	
		}
		
		catch (Exception e) {
			e.printStackTrace();
		}
		
		return nonbinderbeliefs;
	}
	

	/**
	 * Translate a set of alternative feature values into a belief model superformula
	 * 
	 * @param feat a feature (as defined in BinderEssentials)
	 * @return a SuperFormula
	 */
	private SuperFormula getFeatureValuesAsFormula (Feature feat) {

		// The formula 
		SuperFormula formula;

		// If there a single feature value for the feature, create a simple property
		if (feat.alternativeValues.length == 1) {
			formula = BeliefModelUtils.createNewProperty(feat.featlabel, feat.alternativeValues[0]);	
		}

		// Else, create a complex formula includeing a set of disjunctive 
		// (mutually exclusive) properties
		else if (feat.alternativeValues.length >1) {

			SuperFormula[] featvalsArray = new UncertainSuperFormula[feat.alternativeValues.length];

			for (int k = 0 ; k < feat.alternativeValues.length ;k++) {
				featvalsArray[k] = BeliefModelUtils.createNewProperty(feat.featlabel, feat.alternativeValues[k]);
				featvalsArray[k].id = "featval-" + (k+1);
			}

			formula = new ComplexFormula(newDataID(),1.0f, LogicalOp.xor, featvalsArray);
		}

		else {
			formula = new UncertainSuperFormula(newDataID(), 1.0f);
			log("WARNING: feature does not contain any feature value");
		}
		return formula;
	}



	public BeliefModel constructBeliefModel (Vector<Belief> beliefs, String id) {
		
		SpatioTemporalModel s = new SpatioTemporalModel();		
		s.frames = new SpatioTemporalFrame[1];
		s.frames[0] = curFrame;
		
		String[] k = new String[beliefs.size()];

		for (int i = 0 ; i < beliefs.size(); i++) {
			k[i] = beliefs.elementAt(i).id;
		}

		Agent[] a = new Agent[] { robotAgent, humanAgent };
		String[] t = new String[0];
		String[] f = constructForeground(beliefs);

		BeliefModel bmodel = new BeliefModel(id, a, s, k, t, f);
		return bmodel;
	}


 
	private boolean containsOnlyOnePhantomProxy (Union union) {
		return ((union.includedProxies.length == 1) && (union.includedProxies[0] instanceof PhantomProxy));
	}
	
	/**
	 * Translate the union configuration into a belief model
	 * 
	 * @param config the union configuration
	 * @return the super formula
	 */
	public Vector<Belief> extractBeliefs (UnionConfiguration config) {

		Vector<Belief> beliefs = new Vector<Belief>();

		for (int i = 0 ; i < config.includedUnions.length ; i++) {
			Belief newBelief = translateIntoBelief (config.includedUnions[i]);

			if (!containsOnlyOnePhantomProxy(config.includedUnions[i])) {
				beliefs.add(newBelief);
			}
		}	

		log("Belief set successfully built!");
		log("Size: " + beliefs.size());
		
		return beliefs;
	}


	/**
	 * Translate the union configuration into a belief model
	 * 
	 * @param config the union configuration
	 * @return the super formula
	 */
	public Belief translateIntoBelief (Union union) {

		PrivateAgentStatus status = new PrivateAgentStatus();
		status.ag = robotAgent;
		String id = "b-"+union.entityID;

		ComplexFormula formula = new ComplexFormula(); 

		formula.id = "form-"+union.entityID;
		formula.op = LogicalOp.and;
		formula.prob = union.probExists;

		formula.formulae = new SuperFormula[union.features.length + 1];
		for (int j = 0 ; j < union.features.length ; j++) {

			Feature feat = union.features[j];
			formula.formulae[j] = getFeatureValuesAsFormula(feat);
			formula.formulae[j].id = "f"+(j+1);

		}
		formula.formulae[union.features.length] = 
			BeliefModelUtils.createNewProperty("unionRef", new AddressValue(1.0f, union.timeStamp, union.entityID));
		formula.formulae[union.features.length].id = "f" + (union.features.length+1);
		
		
		Belief belief = new Belief(id, curFrame, status, formula, union.timeStamp);
		
		return belief;
	}


	// =================================================================
	// METHODS FOR INSERTING/MODIFYING/DELETING FORMULAE IN THE WM
	// =================================================================

	
	protected boolean isUpdateNeeded (Belief belief) {
		if (currentBeliefs.containsKey(belief.id)) {
			Belief curBelief = currentBeliefs.get(belief.id);

			if (GenericUtils.isMoreRecent(belief.timeStamp, curBelief.timeStamp)) {
				return true;
			}
			else {
				return false;
			}		
		}
		return true;
	}
	
	protected void updateBeliefInWM (Belief belief) {
	
		try {
			if (!existsOnWorkingMemory(belief.id, Binder.BINDER_SA)) {
				addBeliefToWM(belief);
				log("belief " + belief.id + " not currently in working memory, added");
			}
			else {
				overwriteBeliefInWM(belief);
				log("belief " + belief.id + " already in working memory, bein overwritten");			
			}
		}

		catch (Exception e) {
			e.printStackTrace();
		}
	}

	protected void updateBeliefModelInWM (BeliefModel bmodel) {

		try {
			if (!existsOnWorkingMemory(bmodel.id, Binder.BINDER_SA)) {
				addBeliefModelToWM(bmodel);
			}
			else {
				overwriteBeliefModelInWM(bmodel);
			}

		}
		catch (Exception e) {
			e.printStackTrace();
		}
	}


	/**
	 * Insert the formula in the binder working memory
	 * 
	 * @param formula
	 *            the belief model formula
	 */

	protected void addBeliefToWM (Belief belief) {

		try {
			addToWorkingMemory(belief.id, Binder.BINDER_SA, belief);
			log("new belief succesfully added to the binder working memory");

		} catch (Exception e) {
			e.printStackTrace();
		}
	}


	/**
	 * Overwrite an existing formula with a new one (the new formula needs to have
	 * the same id as the existing one)
	 * 
	 * @param formula
	 *            the belief model formula
	 */

	protected void overwriteBeliefInWM (Belief belief) {

		try {
			overwriteWorkingMemory(belief.id, Binder.BINDER_SA, belief);
			log("existing belief succesfully modified in the binder working memory");

		} catch (DoesNotExistOnWMException e) {
			log("Sorry, the belief does not exist in the binder working memory");
		} catch (Exception e) {
			e.printStackTrace();
		}
	}


	/**
	 * Insert the formula in the binder working memory
	 * 
	 * @param formula
	 *            the belief model formula
	 */

	protected void addBeliefModelToWM (BeliefModel bmodel) {

		try {
			addToWorkingMemory(bmodel.id, Binder.BINDER_SA, bmodel);
			log("new belief model succesfully added to the binder working memory");

		} catch (Exception e) {
			e.printStackTrace();
		}
	}


	/**
	 * Overwrite an existing formula with a new one (the new formula needs to have
	 * the same id as the existing one)
	 * 
	 * @param formula
	 *            the belief model formula
	 */
 
	protected void overwriteBeliefModelInWM (BeliefModel bmodel) {

		try {
			overwriteWorkingMemory(bmodel.id, Binder.BINDER_SA, bmodel);
			log("existing belief model succesfully modified in the binder working memory");

		} catch (DoesNotExistOnWMException e) {
			log("Sorry, the belief model does not exist in the binder working memory");
		} catch (Exception e) {
			e.printStackTrace();
		}
	}

	/**
	 * Delete an existing formula
	 * 
	 * @param formula
	 *            the belief model formula
	 */

	protected void deleteFormulaInWM (SuperFormula formula) {

		try {
			deleteFromWorkingMemory(formula.id, Binder.BINDER_SA);
			log("existing belief model formula  succesfully deleted from the binder working memory");

		} catch (DoesNotExistOnWMException e) {
			log("Sorry, the belief model formula  does not exist in the binder working memory");
		} catch (Exception e) {
			e.printStackTrace();
		}
	}


}
