package binder.components;

import java.io.BufferedInputStream;
import java.io.BufferedReader;
import java.io.DataInputStream;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.HashMap;
import java.util.Vector;

import com.sun.tools.internal.jxc.SchemaGenerator.Runner;

import beliefmodels.arch.BeliefException;
import beliefmodels.autogen.beliefs.PerceptBelief;
import beliefmodels.autogen.beliefs.PerceptUnionBelief;
import beliefmodels.builders.PerceptUnionBuilder;
import binder.abstr.BeliefWriter;
import binder.abstr.MarkovLogicComponent;
import binder.arch.BindingWorkingMemory;
import binder.utils.FileUtils;
import binder.utils.MLNGenerator;
import cast.AlreadyExistsOnWMException;
import cast.DoesNotExistOnWMException;
import cast.SubarchitectureComponentException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTData;

public class PerceptualGrouping_MLN extends MarkovLogicComponent {

	String MLNFile = markovlogicDir + "grouping.mln";
	String resultsFile = markovlogicDir + "unions.results";

	@Override
	public void start() {
		addChangeFilter(
				ChangeFilterFactory.createLocalTypeFilter(PerceptBelief.class,
						WorkingMemoryOperation.ADD), new WorkingMemoryChangeReceiver() {
					public void workingMemoryChanged(WorkingMemoryChange _wmc) {
						
						try {
							CASTData<PerceptBelief> beliefData = getMemoryEntryWithData(_wmc.address, PerceptBelief.class);					
							performPerceptualGrouping (beliefData.getData());
						}	
			
						 catch (DoesNotExistOnWMException e) {
								e.printStackTrace();
							}
						 catch (UnknownSubarchitectureException e) {	
							e.printStackTrace();
						} 
						 
					}
				}
		);
	}


	public void performPerceptualGrouping(PerceptBelief b) {
	
		Vector<PerceptUnionBelief> existingUnions = extractExistingUnions();
		
		Vector<String> newUnions = new Vector<String>();
		HashMap<String,String> linkToExistingUnions = new HashMap<String,String>();
		for (PerceptUnionBelief u : existingUnions) {
			String newUnionId = newDataID();
			newUnions.add(newUnionId);
			linkToExistingUnions.put(u.id, newUnionId);
			log(newUnionId);
		}
			
		MLNGenerator.writeMLNFile(b, existingUnions, newUnions, MLNFile);
		
		Vector[] inferenceResults = runAlchemyInference(MLNFile, resultsFile);
	
		
		 try {
		PerceptUnionBelief union = PerceptUnionBuilder.createNewSingleUnionBelief(b, newDataID());
		insertBeliefInWM(union);
		 }
		 catch (BeliefException e) {
				e.printStackTrace();
			} catch (AlreadyExistsOnWMException e) {
			e.printStackTrace();
		}

	}

	

	private Vector<PerceptUnionBelief> createNewUnions(	Vector<PerceptUnionBelief> existingUnions, PerceptBelief b) {
		
		return new Vector<PerceptUnionBelief>();
	}
	
	private Vector<PerceptUnionBelief> extractExistingUnions() {

		Vector<PerceptUnionBelief> existingunions = new Vector<PerceptUnionBelief>();

		try {
			CASTData<PerceptUnionBelief>[] unions;

			unions = getWorkingMemoryEntries(BindingWorkingMemory.BINDER_SA, PerceptUnionBelief.class);

			for (int i = (unions.length - 1) ; i >= 0 ; i--) {
				existingunions.add(unions[i].getData());
			}
		}
		catch (UnknownSubarchitectureException e) {
			log("Problem with architecture name!");
		}
		catch (SubarchitectureComponentException e) {
			e.printStackTrace();
		}
		return existingunions;
	}
	
	
	
	
}
