// =================================================================
// Copyright (C) 2010 DFKI GmbH Talking Robots 
// Pierre Lison (plison@dfki.de)
//                                                                                                                          
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License 
// as published by the Free Software Foundation; either version 2.1 of
// the License, or (at your option) any later version.
//                                                                                                                          
// This library is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// Lesser General Public License for more details.
//                                                                                                                          
// You should have received a copy of the GNU Lesser General Public
// License along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
// 02111-1307, USA.
// =================================================================

// =================================================================
// PACKAGE DEFINITION 
package de.dfki.lt.tr.cast.dialogue;

//=================================================================
// IMPORTS

// Java
import java.util.Enumeration;
import java.util.Hashtable;
import java.util.Properties;
import java.util.Vector;
import java.util.Iterator;


// Dialogue API 
import de.dfki.lt.tr.cast.ProcessingData;
import de.dfki.lt.tr.dialogue.asr.WeightedWord;
import de.dfki.lt.tr.dialogue.asr.WeightedWordClass;
import de.dfki.lt.tr.dialogue.asr.WeightedWordClasses;
import de.dfki.lt.tr.dialogue.asr.WordsActivationNetwork;
import de.dfki.lt.tr.dialogue.ref.SalientEntity;


// CAST
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryChangeQueueBehaviour;

import cast.architecture.WorkingMemoryChangeReceiver;

import cast.cdl.*;
import cast.core.CASTData;
import cast.architecture.ChangeFilterFactory;

/**
 * Component for context-sensitive adaptation of a language model for ASR
 * 
 * @author 	Pierre Lison (plison@dfki.de)
 * @version	100608
 */


public class LanguageModelHelper extends ManagedComponent {

	WeightedWordClasses wordClasses ;
	WeightedWordClasses contextDependentWords;
	WordsActivationNetwork wordsActivation;
	String outputFileName = "";

	float alpha;

	float minWeight;
	float maxWeight;
	
	
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

	// =================================================================
	// CONSTRUCTOR METHODS
	// =================================================================

	/**
	 * @param _id
	 */
	public LanguageModelHelper (String _id) {
		init();
	} // end constructor

	private void init() {
		// general information processing structures
		m_proposedProcessing = new Hashtable<String, ProcessingData>();
		m_dataToProcessingGoalMap = new Hashtable<String, String>();
		m_taskToTaskTypeMap = new Hashtable<String, String>();
		m_dataObjects = new Vector<ProcessingData>();
		m_queueBehaviour = WorkingMemoryChangeQueueBehaviour.QUEUE;

	} // end init

	/*
	 * (non-Javadoc)
	 * 
	 * @see cast.architecture.abstr.WorkingMemoryReaderProcess#start()
	 */
	@Override
	public void start() {
		super.start();

		try {
			 addChangeFilter(
	            		ChangeFilterFactory.createLocalTypeFilter(SalienceModel.class,  WorkingMemoryOperation.ADD),
	                new WorkingMemoryChangeReceiver() {

	                    public void workingMemoryChanged(
	                            WorkingMemoryChange _wmc) {
	                        newSalienceModelReceived(_wmc);
	                    }
	                });
		}
		catch (Exception e) {
			e.printStackTrace();
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


	public void configure(Properties _config) {
		//		_config.list(System.out);
		if (_config.containsKey("--wordClasses")) {
			String src = _config.getProperty("--wordClasses");
			wordClasses = new WeightedWordClasses(src);
		//	wordClasses2 = new WeightedWordClasses(src.replace("smallvoc", "bigvoc"));
		}
		else {
			log("ERROR, no filename for the word classes has been specified");
		}

		if (_config.containsKey("--contextDependentWords")) {
			String src = _config.getProperty("--contextDependentWords");
			contextDependentWords = new WeightedWordClasses(src);
		}

		if (_config.containsKey("--wordsActivationNetwork")) {
			String src = _config.getProperty("--wordsActivationNetwork");
			wordsActivation = new WordsActivationNetwork(src);
		}

		if (_config.containsKey("--outputWordClasses")) {
			String src = _config.getProperty("--outputWordClasses");
			outputFileName = src;
		}

		if (_config.containsKey("--alpha")) {
			String src = _config.getProperty("--alpha");
			alpha = new Float(src);
		}

		if (_config.containsKey("--minWeight")) {
			String src = _config.getProperty("--minWeight");
			minWeight = new Float(src);
		}
		
	}


	public void newSalienceModelReceived(WorkingMemoryChange wmc) {
		// if we have a logical form collection string,
		try {
			// get the id of the working memory entry
			String id = wmc.address.id;
			// get the data from working memory and store it
			// with its id
			CASTData data = getWorkingMemoryEntry(id);
			SalienceModel salienceModel = (SalienceModel) data.getData() ;
			log("Salience model received");
			wordClasses.reset();
		//	wordClasses2.reset();

			
			alpha = 0.10f;
			minWeight = 0.01f;
			maxWeight = 6.0f;

			modifyWordClassesAccordingToSalienceModel(salienceModel);

			wordClasses.writeToOutputFileOneClass(outputFileName.replace(".grammar", "") + "_OBJ_SG_alpha10_min01_max6.txt", "OBJ_SG_LC");
			wordClasses.writeToOutputFileOneClass(outputFileName.replace(".grammar", "") + "_OBJ_PL_alpha10_min01_max6.txt", "OBJ_PL_LC");
			wordClasses.writeToOutputFileOneClass(outputFileName.replace(".grammar", "") + "_ADJUNCT_alpha10_min01_max6.txt", "ADJUNCT_LC");
			wordClasses.writeToOutputFileOneClass(outputFileName.replace(".grammar", "") + "_ACTION_TRANS_alpha10_min01_max6.txt", "ACTION_TRANS_LC");
		}
		catch (Exception e) {
			e.printStackTrace();
		}
	}

	public void modifyWordClassesAccordingToSalienceModel(SalienceModel salienceModel) {

		Hashtable<SalientEntity, Float> distribution = salienceModel.getDistribution() ;

		Vector<String> activated = new Vector<String>();
		
		for (Enumeration<SalientEntity> e = distribution.keys() ; e.hasMoreElements();) {
			SalientEntity entity = e.nextElement() ;

			// we should activate the properties too
			WeightedWordClasses activations = 
				wordsActivation.getActivation(entity.getConcept());

			if (activations != null) {
				for (Iterator<String> f = activations.getClassNames().iterator() ; f.hasNext() ;) {
					String className = f.next() ;
					
					WeightedWordClass initialWordClass = wordClasses.getClassContent(className);
					if (initialWordClass != null) {
					WeightedWordClass activatedWordClass = activations.getClassContent(className);

					for (Enumeration<WeightedWord> g = activatedWordClass.getWeightedWords() ; g.hasMoreElements() ;) {
						WeightedWord activatedWWord = g.nextElement();
						WeightedWord initWWord = initialWordClass.getWeightedWord(activatedWWord.getWord());
						float weight = 0.0f;
						if (initWWord != null) {
							weight = (((float)initWWord.getWeight() / initialWordClass.size()) + alpha ) * initialWordClass.size();
							if (weight > maxWeight)
								weight = maxWeight;
							initWWord.modifyWeight(weight);
							activated.add(activatedWWord.getWord());
						}
					}

					WeightedWordClass contextDependentWordClass = 
						contextDependentWords.getClassContent(className);

					float alpha2 = ((float) activatedWordClass.size() / 
							(contextDependentWordClass.size() - activatedWordClass.size())) * alpha ;
					for (Enumeration<WeightedWord> g = contextDependentWordClass.getWeightedWords() ; g.hasMoreElements() ;) {
						WeightedWord contextDepWWord = g.nextElement() ;
						WeightedWord initWWord = initialWordClass.getWeightedWord(contextDepWWord.getWord());
						if (initWWord != null) {
							if (!activated.contains(contextDepWWord.getWord())) {

								float weight = (((float)initWWord.getWeight() / initialWordClass.size()) - alpha2) * initialWordClass.size();
									if (weight > minWeight) {
									initWWord.modifyWeight(minWeight);
								}
								else {
									initWWord.modifyWeight(minWeight);
								}
							}
						}
					}
					}
					else {
						log("WARNING: class " + className + " does not exist in the wordclasses file");
					}
				}
			}
		}
	}

	public WeightedWordClasses modifyWordClassWeights(
			WeightedWordClasses wcdist, 
			String[] activatedLexemes, 
			String[] desactivatedLexemes) {

		return wcdist;
	}

}
