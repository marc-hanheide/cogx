





package comsys.utils;

import comsys.datastructs.comsysEssentials.*;
import comsys.lf.utils.LFUtils;

import java.util.*;


public class EventStructureUtils {
	
	/**
	 * Add a stateDiscRefRelation to a state
	 * @param state
	 * @param stateDiscRefRelation
	 */
	public static State addStateDiscRefRelation (State st, StateDiscRefRelation stateDiscRefRelation) {
		State state = st;
		ArrayList<StateDiscRefRelation> relations = new ArrayList<StateDiscRefRelation>();
		if (st.participants != null) { 
			relations = new ArrayList<StateDiscRefRelation> (Arrays.asList(state.participants));
			relations.add(stateDiscRefRelation);
			state.participants = relations.toArray(state.participants) ;
		} 
		else {
			state.participants = new StateDiscRefRelation[1];
			state.participants[0] = stateDiscRefRelation;
		}
		return state;
	} 
	
	/**
	 * Add an eventDiscRefRelation to an event
	 * @param event
	 * @param eventDiscRefRelation
	 * @return Event				The updated event
	 */
	public static Event addEventDiscRefRelation (Event ev, EventDiscRefRelation eventDiscRefRelation) {
		Event event = ev;
		ArrayList<EventDiscRefRelation> relations = new ArrayList<EventDiscRefRelation>();
		if (ev.participants != null) { 
			relations = new ArrayList<EventDiscRefRelation> (Arrays.asList(event.participants));
			relations.add(eventDiscRefRelation);
			event.participants = relations.toArray(event.participants) ; 
		} 
		else {
			event.participants = new EventDiscRefRelation[1];
			event.participants[0] = eventDiscRefRelation;
		}
		return event;
	}
	
	/**
	 * Add an aspectualRelation to a nucleus
	 * @param nucleus
	 * @param aspectualRelation
	 */
	public static Nucleus addAspectualRelation (Nucleus nuc, AspectualRelation aspectualRelation) {
		Nucleus nucleus = nuc;
		ArrayList<AspectualRelation> relations = new ArrayList<AspectualRelation>();
		if (nuc.aspectualRelations != null) { 
			relations = new ArrayList<AspectualRelation> (Arrays.asList(nucleus.aspectualRelations));
			relations.add(aspectualRelation);
			nucleus.aspectualRelations = relations.toArray(nucleus.aspectualRelations) ; 
		}
		else {
			nucleus.aspectualRelations = new AspectualRelation[1];
			nucleus.aspectualRelations[0] = aspectualRelation;
		}
		return nucleus;
	}
	
	/**
	 * Add a temporalRelation to a nucleus
	 * @param nucleus
	 * @param temporalRelation
	 */
	public static Nucleus addTemporalRelation (Nucleus nuc, TemporalRelation temporalRelation) {
		Nucleus nucleus = nuc;
		ArrayList<TemporalRelation> relations = new ArrayList<TemporalRelation>();
		if (nuc.temporalRelations != null) { 
			relations = new ArrayList<TemporalRelation>	(Arrays.asList(nucleus.temporalRelations));
			relations.add(temporalRelation);
			nucleus.temporalRelations = relations.toArray(nucleus.temporalRelations) ; 
		}
		else {
			nucleus.temporalRelations = new TemporalRelation[1];
			nucleus.temporalRelations[0] = temporalRelation;		
		}
		return nucleus;
	}
	
	/**
	 * Add an event to a nucleus
	 * @param nucleus
	 * @param event
	 */
	public static Nucleus addEvent (Nucleus nuc, Event event) {
		Nucleus nucleus = nuc;
		ArrayList<Event> relations = new ArrayList<Event>();
		if (nuc.events != null) { 
			relations = new ArrayList<Event> (Arrays.asList(nucleus.events));
			relations.add(event);
			nucleus.events = relations.toArray(nucleus.events) ; 
		}
		else {
			nucleus.events = new Event[1];
			nucleus.events[0] = event;					
		}
		return nucleus;
	}
	
	/**
	 * Add a state to a nucleus
	 * @param nucleus
	 * @param state
	 */
	public static Nucleus addState (Nucleus nuc, State state) {
		Nucleus nucleus = nuc;
		ArrayList<State> relations = new ArrayList<State> ();
		if (nuc.states != null) {
			relations = new ArrayList <State> (Arrays.asList(nucleus.states));
			relations.add(state);
			nucleus.states = relations.toArray(nucleus.states) ; 
		}
		else {
			nucleus.states = new State[1];
			nucleus.states[0] = state;					
		}
		return nucleus;
	}
	
	
	/**
	 * Add a nucleus to the spatioTemporalRepresentation
	 * @param spatioTemporalRepresentation
	 * @param nucleus
	 */
	public static SpatioTemporalRepresentation addNucleus 
	(SpatioTemporalRepresentation spatioTemporalRepresentation, Nucleus nucleus) {
		SpatioTemporalRepresentation str = spatioTemporalRepresentation;
		ArrayList<Nucleus> relations = new ArrayList<Nucleus>();
		if (str.nuclei != null) {
			relations = new ArrayList<Nucleus>(Arrays.asList(spatioTemporalRepresentation.nuclei));
			relations.add(nucleus);
			spatioTemporalRepresentation.nuclei = relations.toArray(spatioTemporalRepresentation.nuclei) ; 
		}
		else {
			spatioTemporalRepresentation.nuclei = new Nucleus[1];
			spatioTemporalRepresentation.nuclei[0] = nucleus;								
		}
		return str;
	}
	
	/**
	 * Adds an interNucleusRelation to the spatioTemporalRepresentation
	 * @param spatioTemporalRepresentation
	 * @param interNucleusRelation
	 */
	public static SpatioTemporalRepresentation addInterNucleusRelation 
	(SpatioTemporalRepresentation spatioTemporalRepresentation,
			InterNucleusRelation interNucleusRelation) {
		SpatioTemporalRepresentation str = spatioTemporalRepresentation;
		ArrayList<InterNucleusRelation> relations = new ArrayList<InterNucleusRelation>();
		if (str.interNucleusRelations != null) {
			relations = new ArrayList<InterNucleusRelation>(Arrays.asList(spatioTemporalRepresentation.interNucleusRelations));
			relations.add(interNucleusRelation);
			spatioTemporalRepresentation.interNucleusRelations = 
				relations.toArray(spatioTemporalRepresentation.interNucleusRelations) ; 
		}
		else {
			spatioTemporalRepresentation.interNucleusRelations = new InterNucleusRelation[1];
			spatioTemporalRepresentation.interNucleusRelations[0] = interNucleusRelation;								
		}
		return str;
	}
	

	/**
	 * Returns an iterator on the list of participants in the event
	 * @param event
	 * @return
	 */
	public static Iterator<EventDiscRefRelation> getParticipants(Event event) {
		return (new ArrayList<EventDiscRefRelation>(Arrays.asList(event.participants))).iterator() ;
	}

	/** 
		Returns the relation with the given mode, or <tt>null</tt> if none is present. 
	*/

	public static EventDiscRefRelation getParticipant(Event event, String mode) { 
		EventDiscRefRelation result = null;
		Iterator partIter = (new ArrayList<EventDiscRefRelation>(Arrays.asList(event.participants))).iterator();
		boolean foundResult = false;
		while (partIter.hasNext() && !foundResult) { 
			EventDiscRefRelation rel = (EventDiscRefRelation) partIter.next();
			if (rel.mode.equals(mode)) { 
				result = rel;
				foundResult = true;
			} // end if check for mode
		} // end while over participants
		return result;
	} // end getParticipant

	/**
	 * Returns an iterator on the list of participants in the state
	 * @param state
	 * @return
	 */
	public static Iterator<StateDiscRefRelation> getParticipants(State state) {
		return (new ArrayList<StateDiscRefRelation>(Arrays.asList(state.participants))).iterator() ;
	}
	
	/**
	 * Returns an iterator on the aspectual relations in the nucleus
	 * @param nucleus
	 * @return
	 */
	public static Iterator<AspectualRelation> getAspectualRelations(Nucleus nucleus) {
		return (new ArrayList<AspectualRelation>(Arrays.asList(nucleus.aspectualRelations))).iterator() ;
	}
	
	/**
	 * Returns an iterator on the temporal relations in the nucleus
	 */
	public static Iterator<TemporalRelation> getTemporalRelations(Nucleus nucleus) {
		return (new ArrayList<TemporalRelation>(Arrays.asList(nucleus.temporalRelations))).iterator() ;
	}
	
	

	/**
	 * Returns an iterator on the events in the nucleus
	 */
	public static Iterator<Event> getEvents(Nucleus nucleus) {
		return (new ArrayList<Event>(Arrays.asList(nucleus.events))).iterator() ;
	}

	public static Event getFirstEvent (Nucleus nucleus) { 
		return (new ArrayList<Event>(Arrays.asList(nucleus.events))).get(0) ;
	} 

	/**
	 * Returns an iterator on the states in the nucleus
	 */
	public static Iterator<State> getStates(Nucleus nucleus) {
		return (new ArrayList<State>(Arrays.asList(nucleus.states))).iterator() ;
	}

	public static State getFirstState (Nucleus nucleus) { 
		return (new ArrayList<State>(Arrays.asList(nucleus.states))).get(0) ;
	} 

	/**
	 * Returns an iterator on the nuclei in the representation
	 * @param spatioTemporalRepresentation
	 * @return
	 */
	public static Iterator<Nucleus> getNuclei(SpatioTemporalRepresentation spatioTemporalRepresentation) {
		return (new ArrayList<Nucleus>(Arrays.asList(spatioTemporalRepresentation.nuclei))).iterator();
	}

	
	/**
	 * Returns an iterator on the internucleusrelations in the representation
	 * @param spatioTemporalRepresentation
	 * @return
	 */
	public static Iterator<InterNucleusRelation> getInterNucleusRelations(SpatioTemporalRepresentation spatioTemporalRepresentation) {
		return (new ArrayList<InterNucleusRelation>(Arrays.asList(spatioTemporalRepresentation.interNucleusRelations))).iterator();
	}
	


	public static String createDOTSpecsIncludingFormulaAndDM
	(Nucleus nucleus, SDRSFormula formula, SDRSFormula dm) {
		String text = "digraph G {\n";
		String specsFormula = SDRSUtils.createDOTSpecsLastFormulaAndLastDM(formula, dm);
		text += specsFormula.substring(12, specsFormula.length()-2);
		String specsNucleus = createDOTSpecs(nucleus);
		text += specsNucleus.substring(12, specsNucleus.length()-6);
		text += nucleus.states[0].stateId.replace('-', '_') + " -> " + formula.type.plf.packedLF.pNodes[0].packedNoms[0].nomVar  + "[weight=0.01, style=invis];\n";
		text += "\n}\n}\n}";
		text += "\n}";
		return text;
	}
		
	public static String createDOTSpecsIncludingFormula(Nucleus nucleus, SDRSFormula formula) {
		String text = "digraph G {\n";
		String specsFormula = SDRSUtils.createDOTSpecsLastFormula(formula);
		text += specsFormula.substring(12, specsFormula.length()-2);
		String specsNucleus = createDOTSpecs(nucleus);
		text += specsNucleus.substring(12, specsNucleus.length()-6);
		text += nucleus.states[0].stateId.replace('-', '_') + " -> " + formula.type.plf.packedLF.pNodes[0].packedNoms[0].nomVar  + "[weight=0.01, style=invis];\n";
		text += "\n}\n}\n}";
		text += "\n}";
		return text;
	}
	
	private static int getIndexNucleus (SpatioTemporalRepresentation str, String nucleusId) {
		for (int i=0; i < str.nuclei.length ; i++) {
			Nucleus nucleus2 = str.nuclei[i];
			if (nucleus2.nucleusId.equals(nucleusId)) {
				return i;
			}
		}
		return -1;
	}
	
	public static String createDOTSpecsFullDiscAndSpatioTemporalRepresentation(SDRS sdrs, SpatioTemporalRepresentation str) {
		String text = "";
		String specsFullDisc = SDRSUtils.createDOTSpecsFullDisc(sdrs);
		text += specsFullDisc.substring(0, specsFullDisc.length()-2);
		for (int i=0; i < str.nuclei.length ; i++) {
			Nucleus nucleus = str.nuclei[i];
			
			SDRSFormula formula = SDRSUtils.getFormulaFromPLF(sdrs, nucleus.plfId);
			if (formula == null) {
				System.out.println("ERROR: SDRS formula associated to nucleus not found");
			}
			
			String specsNucleus = createDOTSpecs(nucleus);
			specsNucleus = specsNucleus.replace("clusterNucleus", "clusterNucleus"+i);
			specsNucleus = specsNucleus.replace("invisN ", "invisN"+i + " ");
			specsNucleus = specsNucleus.replace("c ", "c" + SDRSUtils.getIndexInSDRS(sdrs, formula) + " ");
			text += specsNucleus.substring(12, specsNucleus.length()-6);
			text += nucleus.states[0].stateId.replace('-', '_') + " -> " + 
			formula.type.plf.packedLF.pNodes[0].packedNoms[0].nomVar + 
			"[weight=0.01, style=invis];\n";			
			text += "}\n}\n";
		}
      	text += "compound=true;\n";
      			
		for (int i=0; i < str.interNucleusRelations.length ; i++) {
			InterNucleusRelation inr = str.interNucleusRelations[i];
			text += "invisN" + getIndexNucleus(str, inr.sourceId) +
			" -> " + "invisN" + getIndexNucleus(str, inr.targetId) + 
			" [label=\"" + inr.mode + "\", lhead=clusterNucleus"+ 
			getIndexNucleus(str, inr.targetId) + ", ltail=clusterNucleus"+ 
			getIndexNucleus(str, inr.sourceId) + "];\n";
		}
		text += "\n}";
		return text;
	}
	
	public static String createDOTSpecs(Nucleus nucleus) {
		String text = "digraph G {\n";
	 	text += "subgraph clusterNucleus {\n";
	 	text += "invisN [style=invis];\n";
	 	text += "label = \"Nucleus\";\n";
	 	text += "color=red;\n";
	 	text += "{rank=same;\n";
		for (int i=0; i < nucleus.events.length ; i++) {
			text += nucleus.events[i].eventId.replace('-', '_') +
			"[shape=Mrecord, fontsize=10, label=\"{" + nucleus.events[i].type + "|"+ "Event" +"}\"];\n"; 
			for (int j=0; j < nucleus.events[i].participants.length ; j++) {
				text += nucleus.events[i].eventId.replace('-', '_') + " -> " + 
				nucleus.events[i].participants[j].discRefId + "c"  + " [label=\"" + 
				nucleus.events[i].participants[j].mode + "\"];\n";
			}
		}
		for (int i=0; i < nucleus.states.length ; i++) {
			text += nucleus.states[i].stateId.replace('-', '_')  +"" +
			"[shape=Mrecord, fontsize=10, label=\"{" + nucleus.states[i].type + "|"+ "State" + "}\"];\n"; ;
			for (int j=0; j < nucleus.states[i].participants.length ; j++) {
				text += nucleus.states[i].stateId.replace('-', '_') + " -> " + 
				nucleus.states[i].participants[j].discRefId + "c" + " [label=\"" + 
				nucleus.states[i].participants[j].mode + "\"];\n";
			}		}
		
		for (int i=0; i < nucleus.aspectualRelations.length ; i++) {
			text += nucleus.aspectualRelations[i].eventSourceId.replace('-', '_') + " -> " + 
			nucleus.aspectualRelations[i].stateTargetId.replace('-', '_') + " [label=\"" + 
			nucleus.aspectualRelations[i].aspect + "\"];\n";
		}
		
		for (int i=0; i < nucleus.temporalRelations.length ; i++) {
			text += nucleus.temporalRelations[i].eventSourceId.replace('-', '_') + " -> " + 
			nucleus.temporalRelations[i].stateTargetId.replace('-', '_') + " [label=\"" + 
			nucleus.temporalRelations[i].tempRelation + "\"];\n";
		}
		
		text += "\n}\n}\n}";
			
		return text;
	}


	public static void nucleusToGraph(Nucleus nucleus, String graphName, boolean generatePNG) {

		String DOTFile = graphName + ".dot" ;
		String PNGFile = graphName + ".png" ;

		String DOTText = createDOTSpecs(nucleus) ;

		if (!DOTText.equals("")) {
			LFUtils.writeDOTFile(DOTText,DOTFile);

			if (generatePNG) {
			try	{
				Runtime.getRuntime().exec("dot -Tpng "+DOTFile+" -o "+PNGFile);
			}
			catch (Exception e) {
				e.printStackTrace();
			}
			}
		}

		// showPNGGraph(PNGFile);
	}
	
	public static void fullDiscAndSpatioTemporalRepresentationToGraph
	(SDRS sdrs, SpatioTemporalRepresentation str, String graphName, boolean generatePNG) {

		String DOTFile = graphName + ".dot" ;
		String PNGFile = graphName + ".png" ;

		String DOTText = createDOTSpecsFullDiscAndSpatioTemporalRepresentation(sdrs, str) ;

		if (!DOTText.equals("")) {
			LFUtils.writeDOTFile(DOTText,DOTFile);

			if (generatePNG) {
			try	{
				Runtime.getRuntime().exec("dot -Tpng "+DOTFile+" -o "+PNGFile);
			}
			catch (Exception e) {
				e.printStackTrace();
			}
			}
		}

		// showPNGGraph(PNGFile);
	}
	

	public static void nucleusAndFormulaToGraph(Nucleus nucleus, 
			SDRSFormula formula, String graphName, boolean generatePNG) {

		String DOTFile = graphName + ".dot" ;
		String PNGFile = graphName + ".png" ;

		String DOTText = createDOTSpecsIncludingFormula(nucleus, formula) ;

		if (!DOTText.equals("")) {
			LFUtils.writeDOTFile(DOTText,DOTFile);

			if (generatePNG) {
			try	{
				Runtime.getRuntime().exec("dot -Tpng "+DOTFile+" -o "+PNGFile);
			}
			catch (Exception e) {
				e.printStackTrace();
			}
			}
		}

		// showPNGGraph(PNGFile);
	}
	


	public static void nucleusAndFormulaAndDMToGraph(Nucleus nucleus, 
			SDRSFormula formula, SDRSFormula dm, String graphName, boolean generatePNG) {

		String DOTFile = graphName + ".dot" ;
		String PNGFile = graphName + ".png" ;

		String DOTText = createDOTSpecsIncludingFormulaAndDM(nucleus, formula, dm) ;

		if (!DOTText.equals("")) {
			LFUtils.writeDOTFile(DOTText,DOTFile);

			if (generatePNG) {
			try	{
				Runtime.getRuntime().exec("dot -Tpng "+DOTFile+" -o "+PNGFile);
			}
			catch (Exception e) {
				e.printStackTrace();
			}
			}
		}

		// showPNGGraph(PNGFile);
	}
	
}
