// vim:sw=3:ts=8:et:
package binder.components;

import beliefmodels.arch.BeliefException;
import beliefmodels.autogen.beliefs.StableBelief;
import beliefmodels.autogen.distribs.BasicProbDistribution;
import beliefmodels.autogen.distribs.CondIndependentDistribs;
import beliefmodels.autogen.distribs.FeatureValueProbPair;
import beliefmodels.autogen.distribs.FeatureValues;
import beliefmodels.autogen.distribs.ProbDistribution;
import beliefmodels.autogen.epstatus.PrivateEpistemicStatus;
import beliefmodels.autogen.history.CASTBeliefHistory;
import beliefmodels.builders.BeliefContentBuilder;
import beliefmodels.builders.FeatureValueBuilder;
import beliefmodels.builders.PerceptBuilder;
import beliefmodels.autogen.distribs.FeatureValueProbPair;
import beliefmodels.autogen.featurecontent.FeatureValue;
import beliefmodels.autogen.featurecontent.BooleanValue;
import beliefmodels.autogen.featurecontent.FeatureValue;
import beliefmodels.autogen.featurecontent.FloatValue;
import beliefmodels.autogen.featurecontent.IntegerValue;
import beliefmodels.autogen.featurecontent.PointerValue;
import beliefmodels.autogen.featurecontent.StringValue;
import cast.AlreadyExistsOnWMException;
import cast.DoesNotExistOnWMException;
import cast.UnknownSubarchitectureException;
import cast.PermissionException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryAddress;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import castutils.facades.BinderFacade;
import java.util.LinkedList;
import java.util.List;
import java.util.HashMap;
import java.util.Vector;
import java.util.Map.Entry;
import java.util.logging.Level;
import java.util.logging.Logger;
import cast.core.CASTData;
import cast.core.CASTUtils;
import violetsound.AePlayWave;
// import Task3ResultsPresenter;


/**
 * Monitor the attributes of a VisualObject StableBelief
 * Delete the object without all attributes, when 3 have all attributes.
 * 
 */

public class VisualRecordMonitor extends ManagedComponent {
   private final BinderFacade m_binderFacade;
   private static final String[] DEFAULT_LABELS = {
      "chakakhan", "heartbreakers", "james", "jesusjones" };

   private static final String[] KNOWN_RECORD_PROPS = {
      "is-in" };

   private static final String[] RECORD_WAVS = {
      "wavs/curiosity-oh_whats_that.wav",
      "wavs/curiosity-what_is_thaat.wav",
      "wavs/curiosity-what_is_that.wav",
      "wavs/curiosity-whats_thaat.wav"
   };
   private int[] ordRecordWav;

   private static final String[] PERSON_WAVS = {
      "wavs/curiosity-who_are_you.wav",
      "wavs/Shodan-are_you_afraid.wav",
      "wavs/turret-are_you_still_there.wav",
      "wavs/turret-hello_friend.wav",
      "wavs/turret-i_see_you.wav",
      "wavs/turret-there_you_are.wav",
      "wavs/turret-who_are_you.wav"
   };
   private int[] ordPersonWav;

   private class Record {
      public WorkingMemoryAddress address;
      public String label;
      public boolean hasPlace;
      public String Place;
      public Record(String label, WorkingMemoryAddress address) {
         this.label = label;
         this.address = address;
         this.hasPlace = false;
      }
   }
   HashMap<String, Record> KnownRecords = new HashMap<String, Record>(4);

   public static String toString(FeatureValue fv) {
      String featStr="*";
      if (fv instanceof IntegerValue)
         featStr=Integer.toString(((IntegerValue) fv).val);
      if (fv instanceof PointerValue)
         featStr=CASTUtils.toString(((PointerValue) fv).beliefId);
      if (fv instanceof StringValue)
         featStr=((StringValue) fv).val;
      if (fv instanceof FloatValue)
         featStr= Double.toString(((FloatValue) fv).val);
      if (fv instanceof BooleanValue)
         featStr=Boolean.toString(((BooleanValue) fv).val);
      return featStr;
   }

   public VisualRecordMonitor() {
      m_binderFacade = new BinderFacade(this);
      // ord
   }

   public void start() {
      System.out.println("Entering start");

      super.start();
      m_binderFacade.start();

      //percept belief
      addChangeFilter(
            ChangeFilterFactory.createLocalTypeFilter(StableBelief.class,
               WorkingMemoryOperation.OVERWRITE), new WorkingMemoryChangeReceiver() {
               public void workingMemoryChanged(WorkingMemoryChange _wmc) {
                  // onObjectBeliefChanged(_wmc);
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

                    //List<FeatureValue> labels = m_binderFacade.getFeatureValue(sb, "label");
                    //StringValue recordname = ((StringValue) labels.get(0));
                    //System.out.println("ADDED> " + recordname.val);

                    if (sb.type.equals("VisualObject")) {
                       processObjectBelief(sb, "add");
                       new AePlayWave("wavs/curiosity-oh_whats_that.wav").start();
                    }
                    if (sb.type.equals("Person")) {
                       processPersonBelief(sb, "add");
                       new AePlayWave("wavs/Shodan-are_you_afraid.wav").start();
                    }
                    //System.out.println("StableBelief added: id=" + sb.id 
                    //  + "@" + _wmc.address.subarchitecture
                    //  + " type=" + sb.type);
                  } catch (DoesNotExistOnWMException e) {
                    e.printStackTrace();
                  } catch (UnknownSubarchitectureException e) {
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
         //ProbDistribution beliefcontent =
         //   BeliefContentBuilder.createNewDistributionWithExistDep(1.0f, features);
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
         if (false) {
            try {
               addToWorkingMemory(addr, belief);
               KnownRecords.put(addr.id, new Record(recordName, addr));
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
         }
      }
      catch (BeliefException e) {
         System.out.println("Error BeliefException");
         System.exit(1);
      }
   }

   void createObjectPlaceholders() {
      for (String id: DEFAULT_LABELS) {
         createPlaceholder(id);
      }
   }

   void processPersonBelief(StableBelief belief, String op) {
      String name, place, room, record;
      name = ""; place = ""; room = ""; record = "";
      if (belief.content instanceof CondIndependentDistribs) {
         CondIndependentDistribs dist = (CondIndependentDistribs) belief.content;
         for (Entry<String, ProbDistribution> pd : dist.distribs.entrySet()) {
            if (pd.getValue() instanceof BasicProbDistribution) {
               BasicProbDistribution fvd = (BasicProbDistribution) pd.getValue();
               String key = pd.getKey();
               String strval = "";
               for (FeatureValueProbPair fv : ((FeatureValues)fvd.values).values) {
                  String featStr = VisualRecordMonitor.toString(fv.val);
                  strval += featStr;
               }
               if (key.equals("name")) {
                  name = strval;
               }
               else if (key.equals("is-in")) {
                  place = strval;
               }
            }
         }
      }
      if (! name.isEmpty()) {
         Task3ResultsPresenter.setPerson(name, place, room, record);
      }
      else {
         System.out.println("    NOT Adding person belief with NO NAME");
         // Task3ResultsPresenter.setPerson("Empty Label", place, room, record);
      }
   }

   void processObjectBelief(StableBelief belief, String op) {
      String name, place, room, owner;
      name = ""; place = ""; room = ""; owner = "";
      System.out.println("Adding object belief");
      if (belief.content instanceof CondIndependentDistribs) {
         CondIndependentDistribs dist = (CondIndependentDistribs) belief.content;
         for (Entry<String, ProbDistribution> pd : dist.distribs.entrySet()) {
            System.out.println(pd.getKey());
            if (pd.getValue() instanceof BasicProbDistribution) {
               System.out.println("The value seems ok (BasicProbDistribution)");
               BasicProbDistribution fvd = (BasicProbDistribution) pd.getValue();
               String key = pd.getKey();
               String strval = "";
               for (FeatureValueProbPair fv : ((FeatureValues)fvd.values).values) {
                  String featStr = VisualRecordMonitor.toString(fv.val);
                  strval += featStr;
               }
               if (key.equals("label")) {
                  name = strval;
               }
               else if (key.equals("is-in")) {
                  place = strval;
               }
            }
         }
      }
      if (! name.isEmpty()) {
         System.out.println("    Adding object belief with name");
         Task3ResultsPresenter.setRecord(name, place, room, owner);
      }
      else {
         System.out.println("    Adding object belief with NO NAME");
         Task3ResultsPresenter.setRecord("Empty Label", place, room, owner);
      }
   }

   void onObjectBeliefChanged(WorkingMemoryChange _wmc) {
      CASTData<StableBelief> beliefData = null;
      try {
         beliefData = getMemoryEntryWithData(_wmc.address, StableBelief.class);

         StableBelief sb = beliefData.getData();
         if (! sb.type.equals("VisualObject"))
            return;
         System.out.println("StableBelief added: id=" + sb.id 
               + "@" + _wmc.address.subarchitecture
               + " type=" + sb.type);
      } catch (DoesNotExistOnWMException e) {
         e.printStackTrace();
         return;
      } catch (UnknownSubarchitectureException e) {
         e.printStackTrace();
         return;
      }

      //List<FeatureValue> labels = m_binderFacade.getFeatureValue(sb, "label");
      //String recordname = (String) labels.get(0);

      if (! KnownRecords.containsKey(_wmc.address.id)) {
        //System.out.println("Something's wrong: Unknown record address " + _wmc.address.id);
        //new AePlayWave("wavs/Chord.wav").start();
        return;
      }

      // TODO: verify VisualObject attributes
      // new AePlayWave("wavs/curiosity-oh_whats_that.wav").start();

      Record rec = KnownRecords.get(_wmc.address.id);
      rec.hasPlace = true; // TODO: this needs to be verified

      int count = 0;
      for (Record r: KnownRecords.values()) {
         if (r.hasPlace) count++;
      }
      if (count >= 3) {
         for (String k: KnownRecords.keySet()) {
            Record r = KnownRecords.get(k);
            if (! r.hasPlace) {
               try {
                  deleteFromWorkingMemory(r.address);
               }
               catch (DoesNotExistOnWMException ex) { }
               catch (PermissionException ex) { }
               catch (UnknownSubarchitectureException ex) { }
               KnownRecords.remove(k);
            }
         }
      }
   }

   protected void runComponent() {
      // createObjectPlaceholders();
      new AePlayWave("wavs/Xerxes-unit_i832x265_online.wav").start();
      Task3ResultsPresenter.getInstance().setVisible(true);
   }
}
