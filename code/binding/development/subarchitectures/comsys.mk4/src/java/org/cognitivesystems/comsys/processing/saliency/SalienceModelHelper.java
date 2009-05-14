package org.cognitivesystems.comsys.processing.saliency;

import java.io.BufferedReader;
import java.io.FileReader;
import java.util.Hashtable;
import java.util.Iterator;
import java.util.ListIterator;
import java.util.Properties;
import java.util.Vector;
import java.util.Enumeration;

import org.cognitivesystems.comsys.autogen.ComsysEssentials.*;
import org.cognitivesystems.comsys.data.ProcessingData;
import org.cognitivesystems.comsys.general.ComsysUtils;
import org.cognitivesystems.comsys.general.SDRSUtils;
import org.cognitivesystems.repr.lf.autogen.LFPacking.PackedNominal;
import org.cognitivesystems.repr.lf.autogen.LFPacking.PackingNode;

import balt.core.processes.FrameworkProcess.ProcessStatus;
import BindingData.BindingProxy;
import BindingData.FeaturePointer;
import BindingFeatures.Concept;
import BindingFeatures.Location;
import BindingFeatures.RelationLabel;
import BindingFeatures.SourceID;
import BindingFeatures.Colour;
import cast.core.data.CASTData;
import cast.architecture.abstr.WorkingMemoryChangeReceiver;
import cast.architecture.subarchitecture.ManagedProcess;
import cast.architecture.subarchitecture.SubarchitectureProcessException;
import cast.cdl.*;

public class SalienceModelHelper extends ManagedProcess {

	// ----------------------------------------------------------------
	// INFORMATION PROCESSING DATA STRUCTURES
	// ----------------------------------------------------------------

	// Hashtable used to record the tasks we want to carry out. For each
	// taskID we store a Vector with the data it is to work on
	private Hashtable<String, ProcessingData> m_proposedProcessing;

	// Hashtable linking data IDs to goal IDs
	protected Hashtable<String, String> m_dataToProcessingGoalMap;

	// Hashtable linking task IDs to task types
	protected Hashtable<String, String> m_taskToTaskTypeMap;

	// Vector with objects to be processed,
	// can be ComSys:PhonString,...
	protected Vector<ProcessingData> m_dataObjects;

	// Identifiers for ProcessData objects
	private int pdIdCounter;
	
	boolean processVisualSaliency = false;
	boolean processDiscourseSaliency = false;
	String fakesaliencyFilename; 
	
	// =================================================================
	// CONSTRUCTOR METHODS
	// =================================================================

	/**
	 * @param _id
	 */
	public SalienceModelHelper (String _id) {
		super(_id);
		init();
	} // end constructor

	private void init() {
		// set the ontology for this method
		m_bLogOutput = true;
		// general information processing structures
		m_proposedProcessing = new Hashtable<String, ProcessingData>();
		m_dataToProcessingGoalMap = new Hashtable<String, String>();
		m_taskToTaskTypeMap = new Hashtable<String, String>();
		m_dataObjects = new Vector<ProcessingData>();
		pdIdCounter = 0;
		// synthesis

		// nah: making all the comsys queue changes... don't want to
		// miss a thing
		m_queueBehaviour = WorkingMemoryChangeQueueBehaviour.QUEUE;

	} // end init

	
	public void activateLogging(boolean activation) {
		m_bLogOutput = activation;
	}
	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.architecture.abstr.WorkingMemoryReaderProcess#start()
	 */
	@Override
	public void start() {
		super.start();

		try {
			/**	addChangeFilter(ComsysOntology.RECOGRESULT_TYPE,
					WorkingMemoryOperation.ADD, true,
					new WorkingMemoryChangeReceiver() {

				public void workingMemoryChanged(
						WorkingMemoryChange _wmc) {
					newRecogResultReceived(_wmc);
				}
			});*/
		}
		catch (Exception e) {
			e.printStackTrace();
		}
	}
	
	@Override
	public void configure(Properties _config) {
		//		_config.list(System.out);
		super.configure(_config);
		
		if (_config.containsKey("--visual")) {
			processVisualSaliency = true;
		}
		if (_config.containsKey("--discourse")) {
			processDiscourseSaliency = true;
		}
		if (_config.containsKey("--fakesaliency")) {
		 fakesaliencyFilename = _config.getProperty("--fakesaliency");
		}
	}
	
	// =================================================================
	// CAST TASK METHODS
	// =================================================================

	/**
	 * The method <i>taskAdopted</i> processes a dialogue production
	 * task once the task manager has informed the component it can run.
	 * The method pushes the processing data for a given task onto the
	 * m_dataObjects queue, so that the runComponent() method can spot
	 * something needs to be done. The method does not distinguish
	 * between different types of tasks.
	 * <p>
	 * This method does <b>not</b> inform the task manager, whether the
	 * task has been succesfully completed. This notification happens in
	 * the <i>runComponent</i> method.
	 * 
	 * @see #runComponent
	 * @see cast.architecture.subarchitecture.ManagedProcess#taskAdopted(java.lang.String)
	 */
	@Override
	protected void taskAdopted(String _goalID) {

		// get the data we stored for this goal
		ProcessingData pd = m_proposedProcessing.remove(_goalID);
		if (pd != null) {
			// add the data item to the data objects queue
			m_dataObjects.addElement(pd);
			// get the identifier of the processing data object
			String pdID = pd.getID();
			// link the data ID to the goal ID, for future reference
			// on task completion (done in runComponent)
			m_dataToProcessingGoalMap.put(pdID, _goalID);
		}
		else {
			log("ERROR: Goal without data: " + _goalID);
		} // end if..else
	} // end taskAdopted

	/**
	 * The method <i>taskRejected</i> removes a rejected task from the
	 * list of proposed processing tasks.
	 * 
	 * @see cast.architecture.subarchitecture.ManagedProcess#taskRejected(java.lang.String)
	 */
	@Override
	protected void taskRejected(String _goalID) {

		log("WARNING: The goal with ID [" + _goalID
				+ "] has been rejected.");
		m_proposedProcessing.remove(_goalID);
	} // end taskRejected


	public Vector<SalientEntity> extractSalientEntities() {

		Vector<SalientEntity> salientEntities = new Vector<SalientEntity>();

		if (fakesaliencyFilename != null) {
			salientEntities.addAll(extractFakeSalientEntities(fakesaliencyFilename));
		}
		else {
		if (processVisualSaliency)
			salientEntities.addAll(extractVisualSalientEntities());
		if (processDiscourseSaliency)
			salientEntities.addAll(extractDiscourseSalientEntities());	
		}
		return salientEntities;

	}

	public Vector<SalientEntity> extractFakeSalientEntities(String fakesaliencyFilename) {
		Vector<SalientEntity> entities = new Vector<SalientEntity>();
		String text = readFile(fakesaliencyFilename);
		String[] split = text.split("\n");
		for (int i = 0; i < split.length ; i++) {
			String line = split[i];
			log ("Entity added: " + line);
			SalientEntity entity = new VisualSalientEntity(line);
			entity.score = 1.0f;
			entities.add(entity);
		}
		return entities;
	}
	

	public String readFile(String fileName) {
		String result = "";

		try {
			if (fileName != null) {
				BufferedReader inFile;
				inFile = new BufferedReader(new FileReader(fileName));

				String line = inFile.readLine();
				while (line != null) {
					result += line + "\n";
					line = inFile.readLine();
				}
				log ("File " + fileName + " successfully read");
			}
			else {
				log("WARNING: file " + fileName + " does not exist, unable to read it");
			}
		}
		catch (Exception e) {
			e.printStackTrace();
		}
		return result;
	}
	
	public Vector<SalientEntity> extractDiscourseSalientEntities() {
		Vector<SalientEntity> salientEntities = new Vector<SalientEntity>();
		
		try {
			CASTData[] Data = getWorkingMemoryEntries(SDRS.class, 0);
			if (Data.length > 0) {
			SDRS sdrs = (SDRS) Data[0].getData() ;
			SDRSFormula curFormula = SDRSUtils.getFormula(sdrs, sdrs.LAST);
			int recencyCount = 0;
			while (curFormula != null && !curFormula.label.equals("none")) {		
				if (curFormula != null && SDRSUtils.getFormulaType
						(curFormula) == SDRSUtils.SDRS_DISCRIM_PLF) {
					PackedLFs plf = curFormula.type.plf();
					for (int j=0; j < plf.packedLF.pNodes.length ; j++) {
						PackingNode node = plf.packedLF.pNodes[j];
						for (int k=0; k < node.packedNoms.length ; k++) {
							PackedNominal nom = node.packedNoms[k];
							if (nom.prop.prop != "") {
								DiscourseSalientEntity object = new DiscourseSalientEntity(nom.prop.prop, nom.nomVar, recencyCount);
								salientEntities.add(object);
						//		log("Discourse salient entity added: " + object.getConcept());
							}
						}
					}
				}
				curFormula = SDRSUtils.getFormula(sdrs, curFormula.tprec);
				recencyCount++;
			}
			}
		}
		catch (Exception e) {
			e.printStackTrace();
		}

		
		return salientEntities;
	}
	

	public Vector<SalientEntity> extractVisualSalientEntities() {

		Vector<SalientEntity> SalientEntities = new Vector<SalientEntity>();

		try {

			CASTData<?>[] sourceIds = getWorkingMemoryEntries(SourceID.class, 0);
			for (int i=0; i < sourceIds.length; i++) {
				SourceID sourceId = (SourceID)sourceIds[i].getData();

				if (sourceId.m_sourceID.equals("vision.sa")) {

					String ProxyID = sourceId.m_parent.m_immediateProxyID;
					CASTData<?> proxy =  getWorkingMemoryEntry(ProxyID, "binding.sa"); 
					BindingProxy proxyData = (BindingProxy)proxy.getData();

					FeaturePointer[] features = proxyData.m_proxyFeatures;
					
					for (int j=0;j < features.length ; j++) {
						String type = features[j].m_type;
						String addressPointer = features[j].m_address;	

						if (type.contains("Concept")) {
							CASTData<?> ad =  getWorkingMemoryEntry(addressPointer, "binding.sa"); 
							Concept c = (Concept) ad.getData();
							//			log("Concept: " + c.m_concept); 
							
							VisualSalientEntity object = new VisualSalientEntity(c.m_concept);
							
							log("new visual object added: " + object.getConcept());
							String UnitedProxy = "";
							for (int l=0; l < sourceIds.length ; l++) {
								SourceID sourceId2 = (SourceID)sourceIds[l].getData();
								String ProxyID2 = sourceId2.m_parent.m_immediateProxyID;
								if (!ProxyID2.equals(ProxyID)) {
									CASTData<?> proxy2 =  getWorkingMemoryEntry(ProxyID2, "binding.sa"); 
									BindingProxy proxyData2 = (BindingProxy)proxy2.getData();
									if (proxyData2.m_unionID.equals(proxyData.m_unionID)) {
										UnitedProxy = ProxyID2;
									}
								}
							}

							for (int l=0; l < sourceIds.length ; l++) {
								SourceID sourceId3 = (SourceID)sourceIds[l].getData();
								String ProxyID3 = sourceId3.m_parent.m_immediateProxyID;
								if (!ProxyID3.equals(ProxyID) && !ProxyID3.equals(UnitedProxy)) {
									CASTData<?> proxy3 =  getWorkingMemoryEntry(ProxyID3, "binding.sa"); 
									BindingProxy proxyData3 = (BindingProxy)proxy3.getData();
									boolean isPosRelation = false;

									for (int k=0; k < proxyData3.m_outPorts.m_ports.length; k++) {
										if (proxyData3.m_outPorts.m_ports[k].m_proxyID.equals(UnitedProxy)) {     		    
											FeaturePointer[] features3 = proxyData3.m_proxyFeatures;
											for (int m=0; m < features3.length ; m++) {
												String type3 = features3[m].m_type;
												String addressPointer3 = features3[m].m_address;	
												if (type3.contains("RelationLabel")) {
													CASTData<?> ad3 =  getWorkingMemoryEntry(addressPointer3, "binding.sa"); 
													RelationLabel c3 = (RelationLabel) ad3.getData();
													if (c3.m_label.equals("pos")) {
														isPosRelation = true;
													}
												}
											}
										}
									}


									if (isPosRelation) {
										for (int k=0; k < proxyData3.m_outPorts.m_ports.length; k++) {
											for (int p=0; p < sourceIds.length ; p++) {
												SourceID sourceId4 = (SourceID)sourceIds[p].getData();
												String ProxyID4 = sourceId4.m_parent.m_immediateProxyID;
												CASTData<?> proxy4 =  getWorkingMemoryEntry(ProxyID4, "binding.sa"); 
												BindingProxy proxyData4 = (BindingProxy)proxy4.getData();
												if (ProxyID4.equals(proxyData3.m_outPorts.m_ports[k].m_proxyID)) {
													FeaturePointer[] features4 = proxyData4.m_proxyFeatures;
													for (int m=0; m < features4.length ; m++) {
														String type4 = features4[m].m_type;
														String addressPointer4 = features4[m].m_address;	
														if (type4.contains("Location")) {
															CASTData<?> ad4 =  getWorkingMemoryEntry(addressPointer4, "binding.sa"); 
															Location vc4 = (Location) ad4.getData();
															//		log("Coordinates: " + vc4.m_location.m_x + " " + 
															//				vc4.m_location.m_y + " " + 
															//				vc4.m_location.m_z);

															object.setLocation(vc4.m_location);
														}
													}
												}
											}
										}
									}
								}
							}
							if (object.isWellFormed()) {
								SalientEntities.add(object);
							}
						}
					}
				}
			}
		}
		catch (Exception e) {
			e.printStackTrace();
		}

		return SalientEntities ;
	}
	

	@Override
	public void runComponent() {
		log("Run SalienceModelHelper...");

		sleepProcess(200);

		SalienceModel salienceModel ;
		
		int curNumberOfEntities = 0;
		
		while (m_status == ProcessStatus.RUN) {

			sleepProcess(500);
			Vector<SalientEntity> objects = extractSalientEntities();
			
			if (objects.size() > curNumberOfEntities) {
				
			curNumberOfEntities = objects.size();
			
			salienceModel = new SalienceModel() ;
			salienceModel.addSalientObjects(objects);
			log(salienceModel.toString());
			// Insert result into the working memory
			try {
				lockProcess();
				CASTData[] Data = getWorkingMemoryEntries(SalienceModel.class, 0);
				if (Data.length > 0) {		
					overwriteWorkingMemory(Data[0].getID(), salienceModel, OperationMode.BLOCKING);
				}
				else {
					addToWorkingMemory(newDataID(), salienceModel, OperationMode.BLOCKING);
				}
				unlockProcess();
			}
			catch (Exception e) {
				System.out.println(e);
				e.printStackTrace();
			}
			}
		}
	}
	
}

