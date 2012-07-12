package de.dfki.lt.tr.cast.dialogue.planverb;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.io.Writer;
import java.net.UnknownHostException;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Scanner;

import cast.DoesNotExistOnWMException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryReaderComponent;
import cast.cdl.WorkingMemoryAddress;
import castutils.castextensions.IceXMLSerializer;
import castutils.castextensions.WMView;

import de.dfki.lt.tr.cast.dialogue.GBeliefMemory;
import de.dfki.lt.tr.cast.dialogue.realisation.LFRealiser;
import de.dfki.lt.tr.cast.dialogue.realisation.NoRealiser;
import de.dfki.lt.tr.cast.dialogue.realisation.RealisationClient;
import de.dfki.lt.tr.cast.dialogue.realisation.TarotCCGRealiser;
import de.dfki.lt.tr.planverb.generation.Message;
import de.dfki.lt.tr.planverb.generation.ProtoLFMessage;
import de.dfki.lt.tr.planverb.generation.RhetoricalMarkerMessage;
import de.dfki.lt.tr.planverb.generation.StringMessage;
import de.dfki.lt.tr.planverb.planning.pddl.PDDLContentDeterminator;
import de.dfki.lt.tr.planverb.planning.pddl.PDDLDomainModel;
import de.dfki.lt.tr.planverb.planning.pddl.PDDLHistory;
import de.dfki.tarot.cogx.WMAddress;
import de.dfki.tarot.nlp.lf.BasicLogicalForm;
import de.dfki.tarot.util.BuildException;
import de.dfki.tarot.util.ParseException;
import eu.cogx.beliefs.slice.GroundedBelief;

public class PlanVerbalizer {
	
	private final PDDLContentDeterminator m_contentDeterminator;
	private LFRealiser m_realiser;
	protected ManagedComponent m_castComponent;
	
	private HashMap<String, String> m_preLexicalSub  = new HashMap<String, String>();
//	private HashMap<String, String> m_postLexicalSub = new HashMap<String, String>();
	
	private final static String DEFAULTNGRAMFILE   = "subarchitectures/dialogue.sa/resources/grammars/openccg/moloko.v6/ngram-corpus.txt";
	private final static String DEFAULTGRAMMARFILE = "subarchitectures/dialogue.sa/resources/grammars/openccg/moloko.v6/grammar.xml";
	
	public boolean debug_lf_out = false;
	
	public GBeliefMemory m_gbmemory = new GBeliefMemory();
	
	final protected WMView<GroundedBelief> view = WMView.create(this.m_castComponent, GroundedBelief.class);
	
	private final boolean m_fullConstructor; 
	

	/**
	 * Creates and initializes a new PlanVerbalizer object.
	 * After initialization, it can be queried subsequently for verbalizations of event structures.
	 * The domains and grammar etc. it operates on remain static.
	 * 
	 * This constructor is used for the interactive CAST system.
	 * 
	 * @param annotatedDomainFile
	 * @param pddlDomainFile
	 * @param grammarFile -- or "" -> not needed for the realiserver // or use default file 
	 * @param ngramFile  -- or "" -> not needed for the realiserver // or use default file
	 * @param hostname -- or "" -> try default hostname (localhost) or fallback to internal tarot realiser
	 * @param port -- or null -> try default port (4444) or fallback to internal tarot realiser
	 * @throws IOException
	 */
	public PlanVerbalizer(String annotatedDomainFile, String pddlDomainFile,  String grammarFile, String ngramFile, String hostname, Integer port, WorkingMemoryReaderComponent component) throws IOException  {
		// init CAST component for WM access and logging
		m_castComponent = (ManagedComponent)component;
		
		log("PlanVerbalizer constructor called with annotatedDomainFile = " + annotatedDomainFile +
				" pddlDomainFile = " + pddlDomainFile + " grammarFile = " + grammarFile);
		
		// initialize planner-related stuff
		File adf = new File(annotatedDomainFile);
		File pddf = new File(pddlDomainFile);
		PDDLDomainModel domainModel = new PDDLDomainModel(adf, pddf);
		m_contentDeterminator = new PDDLContentDeterminator(domainModel);

		m_fullConstructor = true;
	
		try {
			m_realiser = new RealisationClient(hostname, port);
		} catch (UnknownHostException e) {
			logException(e);
			m_realiser = new NoRealiser();
			log("Using NoRealiser!");
//			log("trying to use object-based tarot realiser instead of the realiserver.");
//			if (grammarFile.equals("")) grammarFile = DEFAULTGRAMMARFILE;
//			if (ngramFile.equals("")) ngramFile = DEFAULTNGRAMFILE;
//			m_realiser = new TarotCCGRealiser(grammarFile, ngramFile);
		} catch (IOException e) {
			logException(e);
			m_realiser = new NoRealiser();
			log("Using NoRealiser!");
//			log("trying to use object-based tarot realiser instead of the realiserver.");
//			if (grammarFile.equals("")) grammarFile = DEFAULTGRAMMARFILE;
//			if (ngramFile.equals("")) ngramFile = DEFAULTNGRAMFILE;
//			m_realiser = new TarotCCGRealiser(grammarFile, ngramFile);
		}
		
		// initialize lexicon substitutions for the time being...
		initLexicalSubstitutions();
		log("finished PlanVerbalizer constructor");
	}
	
	/**
	 * Creates and initializes a new PlanVerbalizer object.
	 * After initialization, it can be queried subsequently for verbalizations of event structures.
	 * The domains and grammar etc. it operates on remain static.
	 * 
	 * This constructor is used for the stand-alone system.
	 * 
	 * @param annotatedDomainFile
	 * @param pddlDomainFile
	 * @param grammarFile -- or "" -> not needed for the realiserver // or use default file 
	 * @param ngramFile  -- or "" -> not needed for the realiserver // or use default file
	 * @param hostname -- or "" -> try default hostname (localhost) or fallback to internal tarot realiser
	 * @param port -- or null -> try default port (4444) or fallback to internal tarot realiser
	 * @param gbmemoryFile
	 * @throws IOException
	 */
	public PlanVerbalizer(String annotatedDomainFile, String pddlDomainFile,  String grammarFile, String ngramFile, String hostname, Integer port, String gbmemoryFile) throws IOException  {
		// init CAST component for WM access and logging
		m_castComponent = null;

		// Load gbmemory from file
		File f = new File(gbmemoryFile);
		readGBMemoryFromFile(f);
		
		log(m_gbmemory.getTimeStampMap());
		
		log("PlanVerbalizer constructor called with annotatedDomainFile = " + annotatedDomainFile +
				"\npddlDomainFile = " + pddlDomainFile + "\ngrammarFile = " + grammarFile + 
				"\nngamFile = " + ngramFile + "\nhostname = " + hostname + 
				"\nport = " + port + "\ngbmemoryFile = " + gbmemoryFile);
		
		// initialize planner-related stuff
		File adf = new File(annotatedDomainFile);
		File pddf = new File(pddlDomainFile);
		PDDLDomainModel domainModel = new PDDLDomainModel(adf, pddf);
		m_contentDeterminator = new PDDLContentDeterminator(domainModel);

		m_fullConstructor = true;
		
		try {
			m_realiser = new RealisationClient(hostname, port);
		} catch (UnknownHostException e) {
			logException(e);
			log("trying to use object-based tarot realiser instead of the realiserver.");
			if (grammarFile.equals("")) grammarFile = DEFAULTGRAMMARFILE;
			if (ngramFile.equals("")) ngramFile = DEFAULTNGRAMFILE;
			m_realiser = new TarotCCGRealiser(grammarFile, ngramFile);
		} catch (IOException e) {
			logException(e);
			log("trying to use object-based tarot realiser instead of the realiserver.");
			if (grammarFile.equals("")) grammarFile = DEFAULTGRAMMARFILE;
			if (ngramFile.equals("")) ngramFile = DEFAULTNGRAMFILE;
			m_realiser = new TarotCCGRealiser(grammarFile, ngramFile);
		}
		
		// initialize lexicon substitutions for the time being...
		initLexicalSubstitutions();
		log("finished PlanVerbalizer constructor");
	}
	
	/**
	 * This constructor can be used for a standalone verbalization of already determined and finalized messages
	 * (i.e. when reading them from file) via
	 * verbalizeMessageFile(File msgListFile)
	 *  
	 * @param grammarFile
	 * @param ngramFile
	 * @param hostname
	 * @param port
	 * @throws IOException
	 */
	public PlanVerbalizer(String grammarFile, String ngramFile, String hostname, Integer port) throws IOException  {
		m_fullConstructor = false;
		m_contentDeterminator = null;
		
		try {
			m_realiser = new RealisationClient(hostname, port);
		} catch (UnknownHostException e) {
			logException(e);
			log("trying to use object-based tarot realiser instead of the realiserver.");
			if (grammarFile.equals("")) grammarFile = DEFAULTGRAMMARFILE;
			if (ngramFile.equals("")) ngramFile = DEFAULTNGRAMFILE;
			m_realiser = new TarotCCGRealiser(grammarFile, ngramFile);
		} catch (IOException e) {
			logException(e);
			log("trying to use object-based tarot realiser instead of the realiserver.");
			if (grammarFile.equals("")) grammarFile = DEFAULTGRAMMARFILE;
			if (ngramFile.equals("")) ngramFile = DEFAULTNGRAMFILE;
			m_realiser = new TarotCCGRealiser(grammarFile, ngramFile);
		}

		// initialize lexicon substitutions for the time being...
		initLexicalSubstitutions();
		log("finished PlanVerbalizer constructor");
	}


	
	/**
	 * add missing lexical items to this method!
	 * caution: please add blanks before and after each word!
	 */
	private void initLexicalSubstitutions() {
		m_preLexicalSub.put("ascription ^ be", "ascription ^ be ^ <Mood>ind ^ <Tense>past");
		m_preLexicalSub.put("<ExecutionStatus>FAILED", "<Mood>ind ^ <Polarity>neg ^ <Modifier>(could1_0:modal ^ could)");
		m_preLexicalSub.put("m-cause", "m-condition");
		m_preLexicalSub.put(":category ^ meetingroom", ":e-place ^ meetingroom");
		m_preLexicalSub.put("container", "shelf");
//		m_preLexicalSub.put("vision.sa\":thing ^ thing", "vision.sa\":person ^ PERSON");
	}
	
	/**
	 * This is the core method for realizing a report of a given history.
	 * Will call verbalizeHistoryStepOne and verbalizeHistoryStepTwo in row.
	 * 
	 * @param h
	 * @return
	 */
	public String verbalizeHistory(PDDLHistory h) {
		if (!m_fullConstructor) return "verbalizeHistory only possible with situated context and event models (GBeliefs!)";
		return verbalizeHistoryStepTwo(verbalizeHistoryStepOne(h));
	}
	
	
	/**
	 * Step one of verbalization includes
	 * determine messages
	 * process and finalize messages
	 * write message list to file
	 * 
	 * @param h
	 * @return
	 */
	public List<Message> verbalizeHistoryStepOne(PDDLHistory h) {
		if (!m_fullConstructor) {
			log("verbalizeHistory only possible with situated context and event models (GBeliefs!)");
			return null;
		}

		log("verbalizeHistory 1: determining and finalizing messages...");
		List<Message> finalMessages = processAndFinalizeMessages(determineMessages(h));
		log("verbalizeHistory 2: saving messages to file...");
		writeMessagesToFile(finalMessages, h.getTID());
		return finalMessages;
	}
		
	/**
	 * Step two of verbalization includes
	 * realize messages (through openccg)
	 * aggregate text
	 * 
	 * @param finalMessages
	 * @return
	 */
	public String verbalizeHistoryStepTwo(List<Message> finalMessages) {	
		log("verbalizeHistory 3: realising messages via openccg...");
		String rawText = realizeMessages(finalMessages);
		log("verbalizeHistory 4: saving raw text to file...");
		writeTextToFile(rawText, "PEV-rawtext.xml");
		log("verbalizeHistory 5: aggregating realised text string...");
		String outText = PEVUtils.aggregateStrings(rawText);
		log("verbalizeHistory 6: returning generated text...");
		return outText;
	}
	
	
	
	/**
	 * Verbalizing messages that were saved to file.
	 * This replaces step one of verbalization,
	 * and calls step two of verbalization only.
	 * 
	 * @param f
	 * @return
	 */
	public String verbalizeMessageFile(File msgListFile) {
		List<Message> finalMessages = readMessageListFromFile(msgListFile);
		return verbalizeHistoryStepTwo(finalMessages);
	}
	
	/**
	 * Determines the messages for a given history.
	 * 
	 * @param h -- history data structure
	 * @return
	 */
	private List<Message> determineMessages(PDDLHistory h) {
		List<Message> messages = m_contentDeterminator.determineMessages(h);	
		log("contentDeterminator returned " + messages.size() + " messages for the History.");
		return messages;
	}
	
	
	/**
	 * Method to finalize protoLF Messages into situated, complete LFs.
	 * This is basically the context-situated part of the NLG chain. 
	 * 
	 * @param protoMessages
	 * @return
	 */
	private List<Message> processAndFinalizeMessages(List<Message> protoMessages) {
		LinkedList<Message> finalMessages = new LinkedList<Message>();
		for (Message msg : protoMessages) {
			StringBuilder log_sb = new StringBuilder();
			if (msg instanceof ProtoLFMessage) {
				log_sb.append("Current Message is a ProtoLFMessage:\n" + ((ProtoLFMessage) msg).getProtoLF());
				BasicLogicalForm protoLF = ((ProtoLFMessage) msg).getProtoLF();
//				if (protoLF.toString().contains("assume") && protoLF.toString().contains("that")) {
//					log_sb.append("\n Error: I got as 'assume that' protoLF -- ignoring it!");
//					log(log_sb.toString());
//					return "";
//				}
				log("Current Message is a ProtoLFMessage:\n" + ((ProtoLFMessage) msg).getProtoLF());

				// fixing malformed WMAs before GRE
				try {
					protoLF = PEVUtils.preProcessWMAs(protoLF);
					log_sb.append("\n WMA substitution before realization yielded: \n" + protoLF.toString());
				} catch (BuildException e) {
					logException(e);
				} catch (ParseException e) {
					logException(e);
				}
				
				// do GRE 
				GBeliefGRE greModule = new GBeliefGRE(this);
				protoLF = greModule.doGRE(protoLF);
				log_sb.append("\n doGRE() yielded: \n" + protoLF.toString());

				// perform lexical substitution before realization
				try {
					protoLF = PEVUtils.preProcessLexiconSubstitution(protoLF, m_preLexicalSub);
					log_sb.append("\n lexical substitution before realization yielded: \n" + protoLF.toString());
				} catch (BuildException e) {
					logException(e);
				} catch (ParseException e) {
					logException(e);
				}
 
				// make missing parts consistent (e.g. subj agreement)
				BasicLogicalForm finalLF = PEVUtils.finalizeProtoLF(protoLF);
				log_sb.append("\n finalizeProtoLF() yielded: \n" + finalLF.toString());
				
				Message finalPLFMsg = new ProtoLFMessage(finalLF);
				finalMessages.add(finalPLFMsg);

			} else if (msg instanceof StringMessage) {
				finalMessages.add(msg);
			}
	    	log("**** " + log_sb.toString());
        }
    	return finalMessages;
	}
	
	
	/**
	 * method to realize a list of Messages 
	 * (incl ProtoLFMessages and StringMessages) into a surface text.
	 * 
	 * @param messages
	 * @return
	 */
	private String realizeMessages(List<Message> messages) {
		StringBuilder output_sb = new StringBuilder();

		// realize each message
		for (Message msg : messages) {
			StringBuilder log_sb = new StringBuilder();
			if (msg instanceof ProtoLFMessage) {

				BasicLogicalForm finalLF = ((ProtoLFMessage) msg).getProtoLF();
				
				// surface realization
				// perform lexical re-substitution after realization, before appending to the verbal report
				String realization = m_realiser.realiseLF(finalLF);
				log_sb.append("\n realizeLF() yielded: \n" + realization);

				if (!realization.equals("")) {
					output_sb.append(realization + "\n");
				} else {
					if (debug_lf_out) {
						log_sb.append("\n appending original LF to output text.");
						output_sb.append(finalLF.toString() + "\n");
					} else {
						log_sb.append("\n no realisation found for LF " + finalLF.toString());
						output_sb.append("\n");
					}
				}
			} else if (msg instanceof StringMessage) {
				String outputText = ((StringMessage) msg).getText();
				log_sb.append("\n appending current Message, which is a StringMessage: \n " + outputText);
				// rhetorical markers are prepended to the subsequent sentence:
				if (msg instanceof RhetoricalMarkerMessage) {
					output_sb.append(outputText + " "); // no full stop + line break
				} else output_sb.append(outputText + "\n");
			}
	    	log("**** " + log_sb.toString());
        }
    	return output_sb.toString();
	}
	  


	/**
	 * Accessor for the GBelief memory model.
	 * Works both for interactive mode as well as stand-alone mode.
	 * 
	 * @param referentWMA
	 * @return
	 * @throws DoesNotExistOnWMException
	 * @throws UnknownSubarchitectureException
	 */
	protected GroundedBelief getGBelief(WMAddress referentWMA) throws DoesNotExistOnWMException, UnknownSubarchitectureException {
		log("called getGBelief("+referentWMA.toString()+")");
		GroundedBelief gbWME;
		String[] beliefTempIndexWMA = referentWMA.id().split(",");
		if (beliefTempIndexWMA.length>1) {

			WorkingMemoryAddress beliefWMA = new WorkingMemoryAddress(beliefTempIndexWMA[1], referentWMA.subarchitecture());
			String[] beliefTempIndices = beliefTempIndexWMA[0].split("_");
			int beliefTaskID = Integer.parseInt(beliefTempIndices[0]);
			int beliefPOPlanID = Integer.parseInt(beliefTempIndices[1]);

			gbWME = m_gbmemory.getValidGBelief(beliefWMA, beliefTaskID, beliefPOPlanID);
		} else {
			gbWME = m_gbmemory.getLastValidGBelief(new WorkingMemoryAddress(referentWMA.id(), referentWMA.subarchitecture()));
		}
		return gbWME;
	}

	/**
	 * Read the GBeliefMemory object from the given file
	 * 
	 * @param gbmemoryFile
	 * 				the file to read from
	 */
	private void readGBMemoryFromFile(File gbmemoryFile) {
		
		try {
			StringBuilder text = new StringBuilder();
		    String NL = System.getProperty("line.separator");
		    Scanner scanner = new Scanner(new FileInputStream(gbmemoryFile));
		    try {
		      while (scanner.hasNextLine()){
		        text.append(scanner.nextLine() + NL);
		      }
		    }
		    finally{
		      scanner.close();
		    }
		    m_gbmemory = IceXMLSerializer.fromXMLString(text.toString(), GBeliefMemory.class);
			
		} catch (FileNotFoundException e) {
			e.printStackTrace();
		}
		 catch (IOException e) {
			e.printStackTrace();
		}
	}
	
	
	/**
	 * Read the GBeliefMemory object from the given file
	 * 
	 * @param gbmemoryFile
	 * 				the file to read from
	 */
	private List<Message> readMessageListFromFile(File msgListFile) {
		
		try {
			StringBuilder text = new StringBuilder();
		    String NL = System.getProperty("line.separator");
		    Scanner scanner = new Scanner(new FileInputStream(msgListFile));
		    try {
		      while (scanner.hasNextLine()){
		        text.append(scanner.nextLine() + NL);
		      }
		    }
		    finally{
		      scanner.close();
		    }
		    List<Message> msgList = IceXMLSerializer.fromXMLString(text.toString(), List.class);
		    return msgList;
			
		} catch (FileNotFoundException e) {
			e.printStackTrace();
		}
		 catch (IOException e) {
			e.printStackTrace();
		}
		return new LinkedList<Message>();
	}

	
	/**
	 * Write the text (String object) to the given file
	 */
	private void writeTextToFile(String text, String filename) {
		log("Writing TextToFile ...");

		File file;
		file = new File(filename);

		try {
			String msgListXMLString = IceXMLSerializer.toXMLString(text);

			Writer out = new OutputStreamWriter(new FileOutputStream(file));
			try {
				out.write(msgListXMLString);
				log("successfully saved text to " + filename);
			}
			finally {
				out.close();
			}			
		} catch (FileNotFoundException e) {
			logException(e);
			e.printStackTrace();
		}
		catch (IOException e) {
			logException(e);
			e.printStackTrace();
		}
	}
	
	/**
	 * Write the List<Message> object to the given file
	 */
	private void writeMessagesToFile(List<Message> messages, int taskID) {
		log("Writing MessagesToFile ...");

		String fileName = "PlanVerbMessages-" + taskID + ".xml";
		File file = new File(fileName);

		try {
			String msgListXMLString = IceXMLSerializer.toXMLString(messages);

			Writer out = new OutputStreamWriter(new FileOutputStream(file));
			try {
				out.write(msgListXMLString);
				log("successfully saved " + fileName);
			}
			finally {
				out.close();
			}			
		} catch (FileNotFoundException e) {
			logException(e);
			e.printStackTrace();
		}
		catch (IOException e) {
			logException(e);
			e.printStackTrace();
		}
	}

	
	
	/**
	 * tries to use the CAST logger
	 * if unavailable, uses std out
	 * 
	 * @param s - log String
	 */
	protected void log(String s) {
		if (this.m_castComponent!=null) m_castComponent.log(s);
		else System.out.println(s);
	}

	/**
	 * tries to use the CAST logger
	 * if unavailable, uses std out
	 * 
	 * @param s - log String
	 */
	protected void logException(Throwable e) {
		if (m_castComponent!=null) m_castComponent.logException(e);
		else System.out.println(e.getLocalizedMessage());
	}

}
