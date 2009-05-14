	/**
 * 
 */
package motivation.util;

import motivation.idl.FeatureGenerationCompetence;
import BindingFeatures.RelationLabel;
import BindingFeaturesCommon.TemporalFrameType;
import cast.architecture.subarchitecture.AlreadyExistsOnWMException;
import cast.architecture.subarchitecture.ManagedProcess;
import cast.architecture.subarchitecture.SubarchitectureProcessException;
import cast.core.CASTUtils;

/**
 * 
 * Class used to wrap helper methods for registering the competence to generate
 * features.
 * 
 * @author nah
 * 
 */
public class CompetenceRegistration {

	/**
	 * Write a struct to wm that informs the rest of the system that this
	 * component/subarchitecture can handle generate this type of feature. This
	 * will be used to determine where the planner will send feature actions
	 * 
	 * @param <FeatureT>
	 * @param _component
	 * @param _cls
	 * @throws AlreadyExistsOnWMException
	 * @throws SubarchitectureProcessException
	 */
	public static <FeatureT> void registerFeatureGenerationCompetence(
			ManagedProcess _component, Class<FeatureT> _cls)
			throws AlreadyExistsOnWMException, SubarchitectureProcessException {
		
		
		if(_cls.equals(RelationLabel.class)) {
			throw new RuntimeException("RelationLabel compentences should be registered with the method that takes a label argument");
		}
		
		String type = CASTUtils.typeName(_cls);
		FeatureGenerationCompetence reg = new FeatureGenerationCompetence(false, type,
				_component.getProcessIdentifier(), _component
						.getSubarchitectureID());
		_component.addToWorkingMemory(_component.newDataID(), reg);
		_component.log("registered feature competence: " + type);
	}

	/**
	 * Write a struct to wm that informs the rest of the system that this
	 * component/subarchitecture can handle generate relations with this label. This
	 * will be used to determine where the planner will send feature actions
	 * 
	 * @param _component 
	 * @param _label The label of the relation that will be added.
	 * @param _tf Temporal frame of relation supported: NA if this is not appropriate.
	 * @throws AlreadyExistsOnWMException
	 * @throws SubarchitectureProcessException
	 */
	public static void registerFeatureGenerationCompetence(
			ManagedProcess _component, String _label, TemporalFrameType _tf)
			throws AlreadyExistsOnWMException, SubarchitectureProcessException {
		
		if(_tf == TemporalFrameType.PERCEIVED) {
			_label= "perceived-" + _label;
		}
		else if(_tf == TemporalFrameType.ASSERTED) {
			_label= "asserted-" + _label;
		}
		else if(_tf == TemporalFrameType.TYPICAL) {
			_label="asserted-" + _label;
		}
		
		FeatureGenerationCompetence reg = new FeatureGenerationCompetence(true, _label,
				_component.getProcessIdentifier(), _component
						.getSubarchitectureID());
		_component.addToWorkingMemory(_component.newDataID(), reg);
		_component.log("registered feature competence: RelationLabel=" + _label);
	}

	
}
