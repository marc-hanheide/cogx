package binder.components;

import beliefmodels.autogen.beliefs.StableBelief;
import beliefmodels.autogen.beliefs.CondIndependentDistribs;
import beliefmodels.autogen.beliefs.BasicProbDistribution;
import beliefmodels.autogen.distribs.BasicProbDistribution;
import beliefmodels.autogen.distribs.CondIndependentDistribs;
import beliefmodels.autogen.distribs.FeatureValueProbPair;
import beliefmodels.autogen.distribs.ProbDistribution;
import beliefmodels.autogen.history.CASTBeliefHistory;
import beliefmodels.builders.BeliefContentBuilder;
import beliefmodels.builders.FeatureValueBuilder;
import beliefmodels.builders.PerceptBuilder;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import java.util.LinkedList;
import java.util.List;


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

      beliefmodels.autogen.framing.SpatioTemporalFrame frame;
      beliefmodels.autogen.epstatus.EpistemicStatus estatus;
      String id;
      id = recordName; // new WorkingMemoryAddress(child.id, BindingWorkingMemory.BINDER_SA);

      // TODO create wm id
      StableBelief belief = new StableBelief(frame, estatus, id, "VisualObject", beliefcontent, hist);
      WorkingMemoryAddress addr = new WorkingMemoryAddress (newDataID(), "binder");
      addToWorkingMemory(addr, belief);
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
