package de.dfki.lt.tr.cast.dialogue;

import java.util.Arrays;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;

import cast.AlreadyExistsOnWMException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryPointer;
import de.dfki.lt.tr.beliefs.slice.distribs.BasicProbDistribution;
import de.dfki.lt.tr.beliefs.slice.distribs.CondIndependentDistribs;
import de.dfki.lt.tr.beliefs.slice.distribs.FormulaProbPair;
import de.dfki.lt.tr.beliefs.slice.distribs.FormulaValues;
import de.dfki.lt.tr.beliefs.slice.distribs.ProbDistribution;
import de.dfki.lt.tr.beliefs.slice.epstatus.EpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.epstatus.SharedEpistemicStatus;
import de.dfki.lt.tr.beliefs.slice.framing.SpatioTemporalFrame;
import de.dfki.lt.tr.beliefs.slice.history.CASTBeliefHistory;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.ElementaryFormula;
import de.dfki.lt.tr.beliefs.slice.logicalcontent.FloatFormula;
import de.dfki.lt.tr.dialmanagement.utils.EpistemicObjectUtils;
import eu.cogx.beliefs.slice.SharedBelief;

public class FakeObjectCreator extends ManagedComponent{ 
	
	
	@Override
	public void runComponent() {
		
		SpatioTemporalFrame stf = EpistemicObjectUtils.curFrame;
		EpistemicStatus status = new SharedEpistemicStatus(Arrays.asList("self", "human"));
		
		BasicProbDistribution colorDistrib = new BasicProbDistribution("color", new FormulaValues(new LinkedList<FormulaProbPair>()));
		((FormulaValues)colorDistrib.values).values.add(new FormulaProbPair(new ElementaryFormula(0, "red"), 0.7f));
		((FormulaValues)colorDistrib.values).values.add(new FormulaProbPair(new ElementaryFormula(0, "blue"), 0.3f));
		
		BasicProbDistribution shapeDistrib = new BasicProbDistribution("shape", new FormulaValues(new LinkedList<FormulaProbPair>()));
		((FormulaValues)shapeDistrib.values).values.add(new FormulaProbPair(new ElementaryFormula(0, "compact"), 0.7f));
		((FormulaValues)shapeDistrib.values).values.add(new FormulaProbPair(new ElementaryFormula(0, "elongated"), 0.3f));
		
		BasicProbDistribution salienceDistrib = new BasicProbDistribution("salience", new FormulaValues(new LinkedList<FormulaProbPair>()));
		((FormulaValues)salienceDistrib.values).values.add(new FormulaProbPair(new FloatFormula(0, 1.0f), 1.0f));
		
		BasicProbDistribution typeDistrib = new BasicProbDistribution("objecttype", new FormulaValues(new LinkedList<FormulaProbPair>()));
		((FormulaValues)typeDistrib.values).values.add(new FormulaProbPair(new ElementaryFormula(0, "box"), 0.8f));
		
		CondIndependentDistribs content = new CondIndependentDistribs(new HashMap<String,ProbDistribution>());
		content.distribs.put("objecttype", typeDistrib);
		content.distribs.put("color", colorDistrib);
		content.distribs.put("shape", shapeDistrib);
		content.distribs.put("salience", salienceDistrib);

		CASTBeliefHistory history = new CASTBeliefHistory(new LinkedList<WorkingMemoryPointer>(), new LinkedList<WorkingMemoryPointer>());
		
		SharedBelief b = new SharedBelief (stf, status, newDataID(), "visualobject", content, history); 
		
		try {
			addToWorkingMemory(b.id, "binder", b);
		} catch (AlreadyExistsOnWMException e) {
			e.printStackTrace();
		} catch (UnknownSubarchitectureException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

}
