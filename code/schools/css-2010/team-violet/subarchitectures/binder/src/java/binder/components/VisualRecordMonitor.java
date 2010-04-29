package binder.components;

import beliefmodels.arch.BeliefException;
import beliefmodels.autogen.beliefs.StableBelief;
import beliefmodels.autogen.distribs.BasicProbDistribution;
import beliefmodels.autogen.distribs.CondIndependentDistribs;
import beliefmodels.autogen.distribs.FeatureValueProbPair;
import beliefmodels.autogen.distribs.ProbDistribution;
import beliefmodels.autogen.history.CASTBeliefHistory;
import beliefmodels.builders.BeliefContentBuilder;
import beliefmodels.builders.FeatureValueBuilder;
import beliefmodels.builders.PerceptBuilder;
import cast.AlreadyExistsOnWMException;
import cast.DoesNotExistOnWMException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import java.util.LinkedList;
import java.util.List;
import java.util.logging.Level;
import java.util.logging.Logger;


/**
 * Monitor the attributes of a VisualObject StableBelief
 * Delete the object without all attributes, when 3 have all attributes.
 * 
 */

public class VisualRecordMonitor extends ManagedComponent {
   private static final String[] DEFAULT_LABELS = {
      "chakakhan", "heartbreakers", "james", "jesusjones" };

   public void start() {

      System.out.println("Entering start");
      createObjectPlaceholders();

      //percept belief
      addChangeFilter(
            ChangeFilterFactory.createLocalTypeFilter(StableBelief.class,
               WorkingMemoryOperation.OVERWRITE), new WorkingMemoryChangeReceiver() {
               public void workingMemoryChanged(WorkingMemoryChange _wmc) {
                  // TODO: check that it's a belief about a VisualObject and pass
                  // it to onObjectBeliefChanged
                  onObjectBeliefChanged(_wmc);
               }
            }
            );
   }  

   private void createPlaceholder(String recordName) {
       try {
      CondIndependentDistribs features = BeliefContentBuilder.createNewCondIndependentDistribs();
      List<FeatureValueProbPair> labelPairs = new LinkedList<FeatureValueProbPair>();
      labelPairs.add(new FeatureValueProbPair(
               FeatureValueBuilder.createNewStringValue(recordName),
               1.0f));
      BasicProbDistribution labelDistrib =
         BeliefContentBuilder.createNewFeatureDistribution("label", labelPairs);
      BeliefContentBuilder.putNewCondIndependentDistrib(features, labelDistrib);
      ProbDistribution beliefcontent =
         BeliefContentBuilder.createNewDistributionWithExistDep(1.0f, features);
      CASTBeliefHistory hist =
         PerceptBuilder.createNewPerceptHistory(new WorkingMemoryAddress (newDataID(), "vision"));

      beliefmodels.autogen.framing.SpatioTemporalFrame frame=null;
      beliefmodels.autogen.epstatus.EpistemicStatus estatus=null;
      String id;
      id = recordName; // new WorkingMemoryAddress(child.id, BindingWorkingMemory.BINDER_SA);

      // TODO create wm id
      StableBelief belief = new StableBelief(frame, estatus, id, "VisualObject", beliefcontent, hist);
      WorkingMemoryAddress addr = new WorkingMemoryAddress (newDataID(), "binder");
            try {
                addToWorkingMemory(addr, belief);
            } catch (DoesNotExistOnWMException ex) {
                //Logger.getLogger(VisualRecordMonitor.class.getName()).log(Level.SEVERE, null, ex);
            } catch (UnknownSubarchitectureException ex) {
                //Logger.getLogger(VisualRecordMonitor.class.getName()).log(Level.SEVERE, null, ex);
            }
       }
       catch (BeliefException e) {
       }
       catch (AlreadyExistsOnWMException e) {
       }
       
   }

   void createObjectPlaceholders() {
      for (String id: DEFAULT_LABELS) {
         createPlaceholder(id);
      }
   }

   void onObjectBeliefChanged(WorkingMemoryChange _wmc) {
      // TODO: verify VisualObject attributes
   }

}
