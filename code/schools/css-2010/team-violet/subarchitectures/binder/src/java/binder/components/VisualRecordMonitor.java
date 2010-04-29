// vim:sw=3:ts=8:et:
package binder.components;

import beliefmodels.arch.BeliefException;
import beliefmodels.autogen.beliefs.StableBelief;
import beliefmodels.autogen.distribs.BasicProbDistribution;
import beliefmodels.autogen.distribs.CondIndependentDistribs;
import beliefmodels.autogen.distribs.FeatureValueProbPair;
import beliefmodels.autogen.distribs.ProbDistribution;
import beliefmodels.autogen.epstatus.PrivateEpistemicStatus;
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
import cast.core.CASTData;
import violetsound.AePlayWave;


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

      // Testing
      addChangeFilter(
            ChangeFilterFactory.createLocalTypeFilter(StableBelief.class,
               WorkingMemoryOperation.ADD), new WorkingMemoryChangeReceiver() {
               public void workingMemoryChanged(WorkingMemoryChange _wmc) {
                  try {
                     CASTData<StableBelief> beliefData =
                        getMemoryEntryWithData(_wmc.address, StableBelief.class);

                     StableBelief sb = beliefData.getData();
                     System.out.println("StableBelief added: id=" + sb.id 
                        + "@" + _wmc.address.subarchitecture
                        + " type=" + sb.type);
                  } catch (DoesNotExistOnWMException e) {
                     // TODO Auto-generated catch block
                     e.printStackTrace();
                  } catch (UnknownSubarchitectureException e) {
                     // TODO Auto-generated catch block
                     e.printStackTrace();
                  }
               }
            }
            );

      // Testing
      addChangeFilter(
            ChangeFilterFactory.createLocalTypeFilter(StableBelief.class,
               WorkingMemoryOperation.DELETE), new WorkingMemoryChangeReceiver() {
               public void workingMemoryChanged(WorkingMemoryChange _wmc) {
                  //try {
                     System.out.println("StableBelief deleted: id=" + _wmc.address.id);
                  //} catch (DoesNotExistOnWMException e) {
                  //   // TODO Auto-generated catch block
                  //   e.printStackTrace();
                  //} catch (UnknownSubarchitectureException e) {
                  //   // TODO Auto-generated catch block
                  //   e.printStackTrace();
                  //}
               }
            }
            );
   }  

   private void createPlaceholder(String recordName) {
      try {
         System.out.println("createPlaceholder " + recordName);
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
         beliefmodels.autogen.epstatus.EpistemicStatus estatus=
            new PrivateEpistemicStatus();
         String id;
         // id = recordName; // new WorkingMemoryAddress(child.id, BindingWorkingMemory.BINDER_SA);
         id = newDataID(); // new WorkingMemoryAddress(child.id, BindingWorkingMemory.BINDER_SA);

         // TODO create wm id
         // StableBelief belief = new StableBelief(frame, estatus, id, "VisualObject", beliefcontent, hist);
         StableBelief belief = new StableBelief(frame, estatus, id, "VisualObject", features, hist);
         WorkingMemoryAddress addr = new WorkingMemoryAddress (id, "binder");
         System.out.println("WorkingMemoryAddress " + recordName);
         try {
            addToWorkingMemory(addr, belief);
         } catch (DoesNotExistOnWMException ex) {
            //Logger.getLogger(VisualRecordMonitor.class.getName()).log(Level.SEVERE, null, ex);
            System.out.println("Error DoesNotExistOnWMException");
            System.exit(1);
         } catch (UnknownSubarchitectureException ex) {
            //Logger.getLogger(VisualRecordMonitor.class.getName()).log(Level.SEVERE, null, ex);
            System.out.println("Error UnknownSubarchitectureException");
            System.exit(1);
         }
         catch (AlreadyExistsOnWMException e) {
            //Logger.getLogger(VisualRecordMonitor.class.getName()).log(Level.SEVERE, null, ex);
            System.out.println("Error AlreadyExistsOnWMException");
            System.exit(1);
         }
         System.out.println("Presumably done " + recordName);
      }
      catch (BeliefException e) {
         System.out.println("Error BeliefException");
         System.exit(1);
      }

      // To use: new AePlayWave("z:\\Ding.wav").start();
      new AePlayWave("/home/mmarko/prj/cogx/css10/team-violet/wavs/Ding.wav").start();
   }

   void createObjectPlaceholders() {
      System.out.println("Here we go");
      for (String id: DEFAULT_LABELS) {
         System.out.println("in loop " + id);
         createPlaceholder(id);
      }
      System.out.println("Loop over");
   }

   void onObjectBeliefChanged(WorkingMemoryChange _wmc) {
      // TODO: verify VisualObject attributes
   }

   protected void runComponent() {
      createObjectPlaceholders();
   }
}
