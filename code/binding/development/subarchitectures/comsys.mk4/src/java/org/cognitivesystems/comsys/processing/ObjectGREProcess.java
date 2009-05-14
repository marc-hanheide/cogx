//=================================================================
// Copyright (C) 2005-2008 Geert-Jan M. Kruijff
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation; either version 2.1 of
// the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
// 02111-1307, USA.
//=================================================================

//=================================================================
// PACKAGE DEFINITION 
//=================================================================

package org.cognitivesystems.comsys.processing;

//=================================================================
// IMPORTS
//=================================================================



//-----------------------------------------------------------------
// BINDING Imports
//-----------------------------------------------------------------

import BindingData.BindingProxy;
import BindingData.BindingUnion;
import BindingData.FeaturePointer;
import BindingFeatures.Concept;
import BindingFeatures.Colour;
import BindingFeatures.DebugString;
import BindingFeatures.Name;
import BindingFeatures.Shape;
import BindingFeatures.Size;
import binding.common.BindingComponentException;
import binding.util.BindingUtils;

//-----------------------------------------------------------------
// CAST Imports
//-----------------------------------------------------------------

import cast.architecture.abstr.WorkingMemoryChangeReceiver;
import cast.architecture.subarchitecture.SubarchitectureProcessException;
import cast.cdl.WorkingMemoryChange;
import cast.core.CASTUtils;
import cast.core.data.CASTData;

//-----------------------------------------------------------------
// COMSYS Imports
//-----------------------------------------------------------------

import org.cognitivesystems.comsys.processing.GREAttributeTemplateFiller.GREAttribute;
import org.cognitivesystems.comsys.processing.GREAttributeTemplateFiller;
import org.cognitivesystems.comsys.processing.saliency.SalienceModel;


//-----------------------------------------------------------------
// JAVA Imports
//-----------------------------------------------------------------

import java.util.Properties;

//=================================================================
// JAVADOC CLASS DOCUMENTATION 
//=================================================================

/** 
 The class <b>ObjectGREProcess</b> implements a process for 
 producing referring expressions to objects. We use a version of
 Dale & Reiter's incremental algorithm to create logical forms, 
 expressing the content for referring to an object. The process is
 informed about the object through a pointer to a proxy on binding
 working memory. The process retrieves the union for this proxy, to 
 establish which features are available for referring to the object. 
  
 @start	  080902	
 @version 080903
 @author  Geert-Jan M. Kruijff
 */ 

//=================================================================
// CLASS DEFINITION 
//=================================================================

public class ObjectGREProcess 
	extends AbstractGREProcess
{

	//=================================================================
	// CLASS-GLOBAL DATA STRUCTURES 
	//=================================================================
	
	
	/** logging flag, default is false, set by the command line option --log */
	boolean m_logging; 
	
	/** root feature used for computing the distractor set, default is Concept, set by the command line option --distractorFeature */	
	GREAttribute m_distractorFeature; 
	
	/** distance in salience between the intended referent and any distractor. default is 10, set by the command line option --pronominalizationDistance */ 
	int m_pronominalizationDistance;
	
	/** current index for generating nominal variable names. default is 66, set by the command line option --nomIndexStart */
	int m_nomIndex; 
	
	String bindingSA; 

	
	//=================================================================
	// CONSTRUCTOR METHODS
	//=================================================================

	public ObjectGREProcess (String _id) { 
		super(_id);
		init();
	} // end constructor
	
	/**
	Initialization method for the internal global variables for this class. 
	*/ 
	
	private void init () { 
		m_logging = false; 
	} // end init
	
	//=================================================================
	// COMPUTATION METHODS
	//=================================================================
	
	/** 
	 The method <i>createProfile</i> takes an array of features stored 
	 in a union, and fills an attribute template with the values for 
	 these features in so far as present in the template. 
	 
	 @param unionFeatures					The array of features
	 @returns GREAttributeTemplateFiller	The profile
	 */ 
	
	public GREAttributeTemplateFiller createProfile (FeaturePointer[] unionFeatures) { 
		GREAttributeTemplateFiller profile = new GREAttributeTemplateFiller();
		try { 
			for (FeaturePointer ftPt : unionFeatures ) {
				// check for Concept
				if (ftPt.m_type.equals(CASTUtils.typeName(Concept.class))) {
					String type = ((Concept) getWorkingMemoryEntry(ftPt.m_address,getBindingSA()).getData()).m_concept;
					profile.setAttributeValue(GREAttribute.TYPE, type);
				} else if (ftPt.m_type.equals(CASTUtils.typeName(Colour.class))) {
					String colour = ((Colour) getWorkingMemoryEntry(ftPt.m_address,getBindingSA()).getData()).m_colour;
					profile.setAttributeValue(GREAttribute.COLOUR, colour);
				} else if (ftPt.m_type.equals(CASTUtils.typeName(Shape.class))) {
					String shape = ((Shape) getWorkingMemoryEntry(ftPt.m_address,getBindingSA()).getData()).m_shape;
					profile.setAttributeValue(GREAttribute.SHAPE, shape);
				} else if (ftPt.m_type.equals(CASTUtils.typeName(Size.class))) {
					String size = ((Size) getWorkingMemoryEntry(ftPt.m_address,getBindingSA()).getData()).m_size;
					profile.setAttributeValue(GREAttribute.SIZE, size);
				} else if (ftPt.m_type.equals(CASTUtils.typeName(Name.class))) {
					String name = ((Name) getWorkingMemoryEntry(ftPt.m_address,getBindingSA()).getData()).m_name;
					profile.setAttributeValue(GREAttribute.NAME, name);
				} else {
					log("Attribute type ["+ftPt.m_type+"] not dealt with in creating profile for intended referent");
				} // end if..else
			}	// end for over feature pointers
		} catch (BindingComponentException bce) { 
			System.err.println(bce.getMessage());
		} catch (SubarchitectureProcessException spe) { 
			System.err.println(spe.getMessage());			
		} // end try..catch
		return profile; 
	} // end createProfile	
	
	/**
	 The method <i>getIntendedDiscRef</i> examines the (comsys) proxy for the intended referent, and returns informatio 
	 about the discourse referent stored in that proxy. A <tt>null</tt> value is returned if no referent id can be 
	 established (accompanied by an error message on System.err). 
	
	 @param _intdRefProxy	The comsys proxy for the intended referent
	 @returns String		The discourse referent for the intended referent
	*/ 
	
	public String getIntendedDiscRef (BindingProxy _intdRefProxy) { 
		String resultDiscRef = null; 
		try { 
			for (DebugString _ds : BindingUtils.getBindingFeatures(this, _intdRefProxy, DebugString.class)) {
				String dsString = _ds.m_debugString;
				int colonPos = dsString.indexOf(":");
				if (colonPos != -1) {
					resultDiscRef = dsString.substring(colonPos+2,dsString.length()); 
				} else { 
					System.err.println("ERROR: In ObjectGREProcess unable to determine discourse referent for intended referent in ["+dsString+"]");
				} // end 
			} // end for
		} catch (BindingComponentException bce) { 
			System.err.println(bce.getMessage());
		} catch (SubarchitectureProcessException spe) { 
			System.err.println(spe.getMessage());			
		} // end try..catch
		// Return the result
		return resultDiscRef; 
	} // end getIntendedDiscRef	
	
	/**
	The method <i>processGRETask</i> processes the task for generating
	a referring expression. 
	 
	@param _myWrappedTask	The task with information for the process
	*/ 
	
	@Override
	public void processGRETask(GRETaskWrapper _myWrappedTask) {
		log("Starting GRE process");
		// Read in the intended referent proxy
		log("Trying to get binding proxy for intended referent ...");
		BindingProxy _intdRefProxy = null;
		String _proxyID = this.decodeProxyID(_myWrappedTask.getIntendedRefProxyAdress().m_id);
		try {
			_intdRefProxy = BindingUtils.getProxy(this, bindingSA, _proxyID); // this will throw an error if failing; passed upwards
		} catch (SubarchitectureProcessException e) {
			log("ERROR while trying to retrieve binding proxy for intended referent with proxy ID ["+_proxyID+"]");
			e.printStackTrace();
		} // end try..catch 	
		// Try to access the union to get features for identifying the referent
		log("Trying to get binding union for intended referent ...");		
		BindingUnion _intdRefUnion = null;
		try {
			// _intdRefUnion = this.getUnion(_intdRefProxy.m_unionID);
			_intdRefUnion = BindingUtils.getUnion(this, bindingSA, _intdRefProxy.m_unionID); // this will throw an error if failing; passed upwards
		} catch (SubarchitectureProcessException e) {
			log("ERROR while trying to retrieve binding union based on proxy for intended referent");
			e.printStackTrace();
		} // end try..catch
		// Get the features of the union
		FeaturePointer[] _unionFts = _intdRefUnion.m_unionFeatures;		
		// Create the profile for the intended referent
		GREAttributeTemplateFiller profileTemplate = createProfile (_unionFts);
		// Get the discourse referent for the intended referent
		String intdRefDiscRef = getIntendedDiscRef (_intdRefProxy);
		log("Discourse referent for the binding proxy is ["+intdRefDiscRef+"] ...");		
		// Initialize the algorithm for generating a logical form with content for the referring expression
		ObjectGREAlgorithm greAlgo = new ObjectGREAlgorithm();
		greAlgo.setLogging(m_logging);
		greAlgo.setPronominalizationDistance(m_pronominalizationDistance); 
		greAlgo.setDistractorFeature(m_distractorFeature);
		greAlgo.setNominalStartIndex(m_nomIndex);
		try { 
			CASTData[] Data = getWorkingMemoryEntries(SalienceModel.class, 0);
			SalienceModel salience = (SalienceModel) Data[0].getData() ;			
			greAlgo.setSalienceModel(salience);
		} catch (SubarchitectureProcessException spe) { 	
			log("Unable to load salience model in Object GRE process: "+spe.getMessage());
		} // end try..catch	
		// Generate the logical form 
		//String resultRefEx = "@"+encodeProxyID(_proxyID)+":"+greAlgo.generateRFX(profileTemplate, intdRefDiscRef); 
		String resultRefEx = greAlgo.generateRFX(profileTemplate, intdRefDiscRef); 
		log("Returning resulting RFX String: ["+resultRefEx+"]");
		// Update the nominal variable index
		m_nomIndex = greAlgo.getNomIndex();
		// Return the logical form; also sets the m_done flag
		_myWrappedTask.setResultLF(resultRefEx);
		returnRefExLF(_myWrappedTask);		
		return; 
	} // end processGRETask

	//=================================================================
	// CONFIGURATION METHOD
	//=================================================================
	
    public void configure(Properties _config) {
    	super.configure(_config);
        if (_config.containsKey("--log")) {
            String loggingValue = _config.getProperty("--log");
			if (loggingValue.equals("") || loggingValue.equals("true")) { m_logging = true; }
        } // end if.. check for logging
		
		if (_config.containsKey("--distractorFeature")) {
			String feature = _config.getProperty("--distractorFeature");						 
			if (feature.equals("Concept"))		{ m_distractorFeature = GREAttribute.TYPE; } 
			else if (feature.equals("Colour"))	{ m_distractorFeature = GREAttribute.COLOUR; } 
			else if (feature.equals("Shape"))	{ m_distractorFeature = GREAttribute.SHAPE; } 			
			else if (feature.equals("Size"))	{ m_distractorFeature = GREAttribute.SIZE; } 						
			else if (feature.equals("Number"))	{ m_distractorFeature = GREAttribute.NUMBER_TAG; } 
			else if (feature.equals("Name"))	{ m_distractorFeature = GREAttribute.NAME; } 	
			else { m_distractorFeature = GREAttribute.TYPE; }
		} else { 
			m_distractorFeature = GREAttribute.TYPE;
        } // end if.. check for logging	
		
		if (_config.containsKey("--pronominalizationDistance")) { 
			m_pronominalizationDistance = new Integer(_config.getProperty("--pronominalizationDistance")).intValue();
		} else { 
			m_pronominalizationDistance = 10; 
		} // end if.. check for distance measure for pronominalization	
		
		if (_config.containsKey("--nomIndexStart")) { 
			m_nomIndex = new Integer(_config.getProperty("--nomIndexStart")).intValue();
		} else { 
			m_nomIndex = 66; 
		} // end if.. check for start value for indices for nominal variables
		
		if (_config.containsKey("--bsa")) {
			this.bindingSA = _config.getProperty("--bsa");
		} else {
			throw new RuntimeException("You must specify the binding SA ID: --bsa <SA-ID>!");
		}		
		
		
	} // end configure
	
	//=================================================================
	// MISC METHODS
	//=================================================================

	protected String encodeProxyID (String proxyID) { 	
		return proxyID.replaceAll(":","_");
	} 
	
	protected String decodeProxyID (String proxyID) { 
		return proxyID.replaceAll("_",":");
	} 
	
	public void log(String m) { if (m_logging) { System.out.println("[ObjectGREProcess] "+m); } }
	
	
	
} // end class
