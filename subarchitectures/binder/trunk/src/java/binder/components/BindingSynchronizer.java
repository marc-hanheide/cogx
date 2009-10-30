


package binder.components;


import beliefmodels.adl.Belief;
import beliefmodels.adl.BeliefModel;
import beliefmodels.domainmodel.cogx.ColorProperty;
import beliefmodels.domainmodel.cogx.ComplexFormula;
import beliefmodels.domainmodel.cogx.GraspableProperty;
import beliefmodels.domainmodel.cogx.GroundedBelief;
import beliefmodels.domainmodel.cogx.ObjectType;
import beliefmodels.domainmodel.cogx.ObjectTypeProperty;
import beliefmodels.domainmodel.cogx.ShapeProperty;
import beliefmodels.domainmodel.cogx.LocationProperty;
import beliefmodels.domainmodel.cogx.SuperFormula;
import beliefmodels.domainmodel.cogx.UnionRefProperty;
import binder.autogen.core.AlternativeUnionConfigurations;
import binder.autogen.core.Feature;
import binder.autogen.core.FeatureValue;
import binder.autogen.core.ProbabilityDistribution;
import binder.autogen.core.Proxy;
import binder.autogen.core.Union;
import binder.autogen.core.UnionConfiguration;
import binder.autogen.distributions.discrete.DiscreteProbabilityDistribution;
import binder.autogen.featvalues.StringValue;
import binder.constructors.DistributionGeneration;
import binder.constructors.ProxyConstructor;
import binder.utils.BeliefModelUtils;
import binder.utils.BinderUtils;
import binder.utils.FeatureValueUtils;
import cast.ConsistencyException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTData;


public class BindingSynchronizer extends ManagedComponent {

	
	
	/**
	 * Initialisation of the monitor
	 * 
	 * TODO: extend this to AlternativeUnionConfigurations
	 */
	@Override
	public void start() {

		addChangeFilter(ChangeFilterFactory.createGlobalTypeFilter(Belief.class,
				WorkingMemoryOperation.WILDCARD), new WorkingMemoryChangeReceiver() {

			public void workingMemoryChanged(WorkingMemoryChange _wmc) {
				try {
					if (existsOnWorkingMemory(_wmc.address)) {
					Belief b = getMemoryEntry(_wmc.address, Belief.class);
									
					if (_wmc.src.equals("fakeverification")) {
						String unionRefId = getUnionRedId(b);
						if (!unionRefId.equals("")) {
							Union u = getUnionInCurrentConfig(unionRefId);
							if (u != null) {
								updateProxiesWithBeliefInfo (b, u);
							}
						}
					}
					}
				}
				catch (Exception e) {
					e.printStackTrace();
				}
			} 
		});
		
		log("Binding Synchronizer successfully started");
	}
	
	
	
	
	private void updateProxiesWithBeliefInfo (Belief belief, Union u) {
		ObjectTypeProperty typeProp = getObjectTypeProperty(belief);
		if (typeProp != null) {
			StringValue newVal = 
				ProxyConstructor.createStringValue (""+ typeProp.typeValue, typeProp.prob);
			modifyProxiesWithBeliefInfo (u, belief, "obj_label", newVal);
		}
		ColorProperty colorProp = getColorProperty(belief);
		if (colorProp != null) {
			StringValue newVal = 
				ProxyConstructor.createStringValue (""+ colorProp.colorValue, colorProp.prob);
			modifyProxiesWithBeliefInfo (u, belief, "colour", newVal);
		}
		ShapeProperty shapeProp = getShapeProperty(belief);
		if (shapeProp != null) {
			StringValue newVal = 
				ProxyConstructor.createStringValue (""+ shapeProp.shapeValue, shapeProp.prob);
			modifyProxiesWithBeliefInfo (u, belief, "shape", newVal);
		}
		LocationProperty locationProp = getLocationProperty(belief);
		if (locationProp != null) {
			StringValue newVal = 
				ProxyConstructor.createStringValue (""+ locationProp.location, locationProp.prob);
			modifyProxiesWithBeliefInfo (u, belief, "location", newVal);
		}		
		GraspableProperty graspableProp = getGraspableProperty(belief);
		if (graspableProp != null) {
			StringValue newVal = 
				ProxyConstructor.createStringValue (""+ graspableProp.graspableValue, graspableProp.prob);
			modifyProxiesWithBeliefInfo (u, belief, "graspable", newVal);
		}	
	}
	
	
	private Union getUnionInCurrentConfig (String unionRef) {
		try {
		CASTData<UnionConfiguration>[] ucs = getWorkingMemoryEntries(UnionConfiguration.class);
		if (ucs.length > 0) {
			for (int i = 0 ; i < ucs[0].getData().includedUnions.length ; i++) {
				Union u = ucs[0].getData().includedUnions[i];
				if (u.entityID.equals(unionRef)) {
					return u;
				}
			}	
		}
		}
		catch (Exception e) {
			e.printStackTrace();
		}
		return null;
	}
	
	
	
	private void modifyProxiesWithBeliefInfo (Union u, Belief b, String featlabel, StringValue replacingValue) {

		Proxy modifiedProxy = null;
		for (int j = 0 ; j < u.includedProxies.length ; j++) {
			Proxy p = u.includedProxies[j];
			for (int i = 0 ; i < p.features.length ; i++) {
				if (p.features[i].featlabel.equals(featlabel)) {		
						p.features[i].alternativeValues = new FeatureValue[1];
						p.features[i].alternativeValues[0] = replacingValue;
						log("UPDATED OBJ_LABEL: " + ""+ FeatureValueUtils.toString((replacingValue)));
						modifiedProxy = p;
				}				
			}
		}
		
		if (modifiedProxy == null) {
			Feature feat = ProxyConstructor.createFeatureWithUniqueFeatureValue(featlabel, replacingValue);
			modifiedProxy = u.includedProxies[0];
			ProxyConstructor.addFeatureToProxy(modifiedProxy, feat);
		}
		
			Proxy newProxy = ProxyConstructor.createNewProxy
				(modifiedProxy.origin, modifiedProxy.entityID, modifiedProxy.probExists, modifiedProxy.features);
			
			ProxyConstructor.setTimeStamp(newProxy, getCASTTime());
			
			try {
				log("Overwriting proxy with ID: " + newProxy.entityID);
			//	startVersioning(newProxy.entityID);
				overwriteWorkingMemory(newProxy.entityID, Binder.BINDER_SA, newProxy);
				log("Overwrite successfull!");
			}
			catch (Exception e) {
				try {
					e.printStackTrace();
					log("Problem, trying again...");
					Proxy updatedProxy = getMemoryEntry(newProxy.entityID, Binder.BINDER_SA, Proxy.class);
					overwriteWorkingMemory(newProxy.entityID, Binder.BINDER_SA, newProxy);
				}
				catch (Exception e2) {
					try {
		//			e2.printStackTrace();
					log("trying to brute force method: delete and add");
					deleteFromWorkingMemory(newProxy.entityID);
					addToWorkingMemory(newProxy.entityID, Binder.BINDER_SA, newProxy);
					}
					catch (Exception e3) {
						log("giving up...");
						e3.printStackTrace();
					}
				}
		} 
	}
	
	
	private ObjectTypeProperty getObjectTypeProperty (Belief b) {
		if (b.phi != null && b.phi instanceof ComplexFormula) {
			for (int i = 0 ; i < ((ComplexFormula)b.phi).formulae.length ; i++) {
				SuperFormula form =  ((ComplexFormula)b.phi).formulae[i];
				if (form instanceof ObjectTypeProperty) {
					return  ((ObjectTypeProperty)form);
				}
			}
			}
			return null;
	}
	
	
	private ShapeProperty getShapeProperty (Belief b) {
		if (b.phi != null && b.phi instanceof ComplexFormula) {
			for (int i = 0 ; i < ((ComplexFormula)b.phi).formulae.length ; i++) {
				SuperFormula form =  ((ComplexFormula)b.phi).formulae[i];
				if (form instanceof ShapeProperty) {
					return  ((ShapeProperty)form);
				}
			}
			}
			return null;
	}
	
	private LocationProperty getLocationProperty (Belief b) {
		if (b.phi != null && b.phi instanceof ComplexFormula) {
			for (int i = 0 ; i < ((ComplexFormula)b.phi).formulae.length ; i++) {
				SuperFormula form =  ((ComplexFormula)b.phi).formulae[i];
				if (form instanceof LocationProperty) {
					return  ((LocationProperty)form);
				}
			}
			}
			return null;
	}
	

	private GraspableProperty getGraspableProperty (Belief b) {
		if (b.phi != null && b.phi instanceof ComplexFormula) {
			for (int i = 0 ; i < ((ComplexFormula)b.phi).formulae.length ; i++) {
				SuperFormula form =  ((ComplexFormula)b.phi).formulae[i];
				if (form instanceof GraspableProperty) {
					return  ((GraspableProperty)form);
				}
			}
			}
			return null;
	}
	

	private ColorProperty getColorProperty (Belief b) {
		if (b.phi != null && b.phi instanceof ComplexFormula) {
			for (int i = 0 ; i < ((ComplexFormula)b.phi).formulae.length ; i++) {
				SuperFormula form =  ((ComplexFormula)b.phi).formulae[i];
				if (form instanceof ColorProperty) {
					return  ((ColorProperty)form);
				}
			}
			}
			return null;
	}
	
	
	private String getUnionRedId (Belief b) {
		
		if (b.phi != null && b.phi instanceof ComplexFormula) {
		for (int i = 0 ; i < ((ComplexFormula)b.phi).formulae.length ; i++) {
			SuperFormula form =  ((ComplexFormula)b.phi).formulae[i];
			if (form instanceof UnionRefProperty) {
				return  ((UnionRefProperty)form).unionRef ;
			}
		}
		}
		return "";
	}
	
}
