package celmarchitecture.global;

import java.util.Map;

/**
 *  If C-ELM processes are run across several subarchitectures they need
 *  to know the names of the latter to successfully communicate. 
 *  SANames provides a unified configuration scheme and global defaults
 *  for this.
 */
public class SANames {

    public static final String configKeyWriterSA       = "--writer-sa";
    public static final String configKeyRecognitionSA  = "--recognition-sa";
    public static final String configKeyRecollectionSA = "--recollection-sa";
    public static final String configKeySimulationSA   = "--simulation-sa";
    public static final String configKeyLocationConvSA = "--loc-conv-sa";
    
    // default setup for singleSA mode...
    public String writerSA                             = "celm.sa"; // "celmwriter.sa";
    public String recognitionSA                        = "celm.sa"; // "eventrecognition.sa";
    public String recollectionSA                       = "celm.sa"; // "recollection.sa";
    public String simulationSA                         = "celm.sa"; // "simulator.sa";
    public String locationConvSA                       = "celm.sa"; // "locationConversion.sa";
    
    public void configure(Map<String, String> config) {
	
	if (config.containsKey(configKeyWriterSA)) 
	    writerSA = config.get(configKeyWriterSA);
	
	if (config.containsKey(configKeyRecognitionSA)) 
	    recognitionSA = config.get(configKeyRecognitionSA);
	
	if (config.containsKey(configKeyRecollectionSA)) 
	    recollectionSA = config.get(configKeyRecollectionSA);
	
	if (config.containsKey(configKeySimulationSA)) 
	    simulationSA = config.get(configKeySimulationSA);
	
	if (config.containsKey(configKeyLocationConvSA)) 
	    locationConvSA = config.get(configKeyLocationConvSA);    
	
    }
}
