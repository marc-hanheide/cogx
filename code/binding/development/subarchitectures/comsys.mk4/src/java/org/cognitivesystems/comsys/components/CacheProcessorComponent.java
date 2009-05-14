package org.cognitivesystems.comsys.components;

import org.cognitivesystems.comsys.data.ProcessingData;
import org.cognitivesystems.comsys.general.ComsysException;

public abstract class CacheProcessorComponent extends PackedLFProcessorComponent {

	   // =================================================================
    // CONSTRUCTOR METHODS
    // =================================================================

	/** The unary constructor 
	
		@param _id The id of the process
	*/ 
	
	public CacheProcessorComponent (String _id) { 
		super(_id);
	} // end constructor

	
	/**
	The method <i>executeProcessingTask</i> is the method called when a PackedLF processing task 
	has been accepted. 
	
	@param  pd The Cache data to be processed
	@throws ComsysException Thrown if something goes wrong 
*/ 
	@Override
	abstract void executeProcessingTask(ProcessingData pd) throws ComsysException ;
	
	
	
}
