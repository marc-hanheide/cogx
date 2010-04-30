package binder.gui;

import java.util.Map.Entry;
import java.util.Vector;
import cast.cdl.WorkingMemoryAddress;
import java.io.*;

import beliefmodels.autogen.beliefs.StableBelief;
import beliefmodels.autogen.beliefs.Belief;
import beliefmodels.autogen.distribs.BasicProbDistribution;
import beliefmodels.autogen.distribs.CondIndependentDistribs;
import beliefmodels.autogen.distribs.FeatureValueProbPair;
import beliefmodels.autogen.distribs.FeatureValues;
import beliefmodels.autogen.distribs.ProbDistribution;
import beliefmodels.autogen.featurecontent.BooleanValue;
import beliefmodels.autogen.featurecontent.FeatureValue;
import beliefmodels.autogen.featurecontent.FloatValue;
import beliefmodels.autogen.featurecontent.IntegerValue;
import beliefmodels.autogen.featurecontent.PointerValue;
import beliefmodels.autogen.featurecontent.StringValue;
import cast.DoesNotExistOnWMException;
import cast.UnknownSubarchitectureException;
import cast.architecture.ChangeFilterFactory;
import cast.architecture.ManagedComponent;
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTData;
import cast.core.CASTUtils;

/**
 * Class tat looks into the binder and presents the results
 */
public class PresentResult extends ManagedComponent {

    class Place {
        public String address = "";
        public int id = -1;        
        public String roomaddress = "";
        public String roomname = "UNKNOWN";
    };

    class Room {
        public String address = "";
        public String name = "UKNOWN";
    }
    
    class Person {
        public String address = "";
        public String id = "";
        public int placeid = -1;
        public String name = "UNKNOWN";
        public String roomname = "UNKNOWN";
        public String record = "UNKNOWN";
    };

    class VisualObject {
        public String address = "";
        public String id = "";
        public int placeid = -1;
        public String category = "UNKNOWN";
        public String roomname = "UNKNOWN";
    };

    Vector<Place> m_Places;
    Vector<Room> m_Rooms;
    Vector<Person> m_People;
    Vector<VisualObject> m_VisualObjects;

    public PresentResult()
    {
        m_Places = new Vector<Place>();
        m_Rooms = new Vector<Room>();
        m_People = new Vector<Person>();
        m_VisualObjects = new Vector<VisualObject>();
    }

    public void start() {
	
        addChangeFilter(ChangeFilterFactory.
                        createLocalTypeFilter(StableBelief.class,
                                              WorkingMemoryOperation.ADD), 
                        new WorkingMemoryChangeReceiver() {
                            
                            public void workingMemoryChanged(WorkingMemoryChange _wmc) {                                
                                try {
                                    CASTData<StableBelief> beliefData = getMemoryEntryWithData(_wmc.address, StableBelief.class);
                                    updateBeliefs(beliefData.getData(), _wmc);
                                    
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
		
        addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(StableBelief.class,
                                                                  WorkingMemoryOperation.OVERWRITE), new WorkingMemoryChangeReceiver() {
                                                                          public void workingMemoryChanged(WorkingMemoryChange _wmc) {

                                                                              try {
                                                                                  CASTData<StableBelief> beliefData = getMemoryEntryWithData(_wmc.address, StableBelief.class);
                                                                                  updateBeliefs(beliefData.getData(), _wmc);
                                    
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
        
        addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(StableBelief.class,
                                                                  WorkingMemoryOperation.DELETE), new WorkingMemoryChangeReceiver() {
                                                                          public void workingMemoryChanged(WorkingMemoryChange _wmc) {
                                                                              
                                                                          }
                                                                      }
                        );	       		
    }  
    

    public Room getRoom(StableBelief sBelief) {
        Room r = new Room();
        r.address = sBelief.id;
        
        if (sBelief.content instanceof CondIndependentDistribs) {
            CondIndependentDistribs dist = (CondIndependentDistribs) sBelief.content;
            for (Entry<String, ProbDistribution> pd : dist.distribs.entrySet()) {
                if (pd.getValue() instanceof BasicProbDistribution) {
                    BasicProbDistribution fvd = (BasicProbDistribution) pd.getValue();
                    
                    if (pd.getKey().equals("concepts")) {
                        for (FeatureValueProbPair fv : ((FeatureValues)fvd.values).values) {
                            if (fv.val instanceof StringValue) {
                                r.name = ((StringValue) fv.val).val;
                            }
                            break;
                        }
                    }
                }
            }                                
        }        

        //log("Read room with name " + r.name);
        return r;
    }

    public Place getPlace(StableBelief sBelief) {
        Place p = new Place();
        p.address = sBelief.id;

        if (sBelief.content instanceof CondIndependentDistribs) {
            CondIndependentDistribs dist = (CondIndependentDistribs) sBelief.content;
            for (Entry<String, ProbDistribution> pd : dist.distribs.entrySet()) {
                if (pd.getValue() instanceof BasicProbDistribution) {
                    BasicProbDistribution fvd = (BasicProbDistribution) pd.getValue();
                    
                    if (pd.getKey().equals("PlaceId")) {
                        for (FeatureValueProbPair fv : ((FeatureValues)fvd.values).values) {
                            if (fv.val instanceof IntegerValue) {
                                p.id = ((IntegerValue) fv.val).val;
                            }
                            break;
                        }
                    } else if (pd.getKey().equals("in-room")) {
                        for (FeatureValueProbPair fv : ((FeatureValues)fvd.values).values) {
                            p.roomaddress = ((PointerValue) fv.val).beliefId.id;

                            //log("Getting adress of room " + p.roomaddress);
                            
                            try {
                                CASTData<StableBelief> data = 
                                    getMemoryEntryWithData(((PointerValue) fv.val).beliefId, StableBelief.class);
                                
                                Room r = getRoom(data.getData());

                                if (!r.name.equals("")) {
                                    p.roomname = r.name;
                                }

                            } catch (Exception e) {}
                            break;
                        }
                    }
                }                                
            }

        }

        return p;
    }

    public Person getPerson(StableBelief sBelief) {
        Person p = new Person();
        p.address = sBelief.id;
        
        if (sBelief.content instanceof CondIndependentDistribs) {
            CondIndependentDistribs dist = (CondIndependentDistribs) sBelief.content;
            for (Entry<String, ProbDistribution> pd : dist.distribs.entrySet()) {
                if (pd.getValue() instanceof BasicProbDistribution) {
                    BasicProbDistribution fvd = (BasicProbDistribution) pd.getValue();
                    
                    if (pd.getKey().equals("name")) {
                        for (FeatureValueProbPair fv : ((FeatureValues)fvd.values).values) {
                            p.name = ((StringValue) fv.val).val;
                            break;

                        }
                    } else if (pd.getKey().equals("is-in")) {
                        for (FeatureValueProbPair fv : ((FeatureValues)fvd.values).values) {
                            try {
                                CASTData<StableBelief> data = 
                                    getMemoryEntryWithData(((PointerValue) fv.val).beliefId, StableBelief.class);
                                
                                Place place = getPlace(data.getData());
                                p.placeid = place.id;

                                if (!place.roomaddress.equals("")) {

                                   CASTData<StableBelief> data2 = 
                                       getMemoryEntryWithData(place.roomaddress, StableBelief.class); 

                                   Room r = getRoom(data2.getData());

                                   p.roomname = r.name;
                                }

                            } catch (Exception e) {}
                            break;
                        }
                    }
                }                                
            }

        }

        return p;
    }
	
    public VisualObject getVisualObject(StableBelief sBelief) {
        VisualObject o = new VisualObject();
        o.address = sBelief.id;

        if (sBelief.content instanceof CondIndependentDistribs) {
            CondIndependentDistribs dist = (CondIndependentDistribs) sBelief.content;
            for (Entry<String, ProbDistribution> pd : dist.distribs.entrySet()) {
                if (pd.getValue() instanceof BasicProbDistribution) {
                    BasicProbDistribution fvd = (BasicProbDistribution) pd.getValue();
                    
                    if (pd.getKey().equals("object-label")) {
                        //log("Got an object label for VisualObject");
                        for (FeatureValueProbPair fv : ((FeatureValues)fvd.values).values) {
                            o.category = ((StringValue) fv.val).val;
                            break;

                        }
                    } else if (pd.getKey().equals("is-in")) {

                        //log("Got the is-in for VisualObject");
                        for (FeatureValueProbPair fv : ((FeatureValues)fvd.values).values) {
                            try {
                                CASTData<StableBelief> data = 
                                    getMemoryEntryWithData(((PointerValue) fv.val).beliefId, StableBelief.class);
                                
                                Place place = getPlace(data.getData());
                                o.placeid = place.id;

                                if (!place.roomaddress.equals("")) {

                                   CASTData<StableBelief> data2 = 
                                       getMemoryEntryWithData(place.roomaddress, StableBelief.class); 

                                   Room r = getRoom(data2.getData());

                                   o.roomname = r.name;
                                }

                            } catch (Exception e) {}
                            break;
                        }
                    }
                }                                
            }

        }

        return o;
    }
	
    public void updateBeliefs(StableBelief sBelief, WorkingMemoryChange wmc)
    {
        String info = "Got a new belief at address " +
            wmc.address.id + "@" + wmc.address.subarchitecture + 
            " with type \"" + sBelief.type + "\"";
        
        if (sBelief.content instanceof CondIndependentDistribs) {
            CondIndependentDistribs dist = (CondIndependentDistribs) sBelief.content;
            String features="";
            for (Entry<String, ProbDistribution> pd : dist.distribs.entrySet()) {
                if (pd.getValue() instanceof BasicProbDistribution) {
                    BasicProbDistribution fvd = (BasicProbDistribution) pd.getValue();
                    features+=pd.getKey()+"=[";
                    for (FeatureValueProbPair fv : ((FeatureValues)fvd.values).values) {
                        String featStr = toString(fv.val);
                        features+=featStr+" ";
                    }
                    features+="] ";
                }
            }
            info += " Features: " + features;
        }
        
        //log(info);
        
        
        if (sBelief.type.equals("Place")) {
        
            Place p = getPlace(sBelief);
            
            boolean add = true;
            for (int i = 0; i < m_Places.size(); i++) {
                if (m_Places.elementAt(i).id == p.id) {
                    //log("Place already existed");
                    m_Places.elementAt(i).roomaddress = p.roomaddress;
                    m_Places.elementAt(i).roomname = p.roomname;
                    add = false;
                    break;
                }
            }            
            
            for (int i = 0; i < m_Rooms.size(); i++) {
                if (m_Rooms.elementAt(i).address == p.roomaddress) {
                    p.roomname = m_Rooms.elementAt(i).name;
                }
            }
            
            if (add) m_Places.add(p);

        } else if (sBelief.type.equals("ComaRoom")) {

            Room r = getRoom(sBelief);

            boolean add = true;
            for (int i = 0; i < m_Rooms.size(); i++) {
                if (m_Rooms.elementAt(i).address == sBelief.id) {
                    r.name = new String(m_Rooms.elementAt(i).name);
                    add = false;
                }
            }
            
            if (add) m_Rooms.add(r);

            for (int i = 0; i < m_Places.size(); i++) {
                if (m_Places.elementAt(i).roomaddress == r.address) {
                    m_Places.elementAt(i).roomname = new String(r.name);
                }
            }                     
            
        } else if (sBelief.type.equals("Person")) {

            Person p = getPerson(sBelief);

            for (int i = 0; i < m_Places.size(); i++) {
                if (m_Places.elementAt(i).id == p.placeid) {
                    p.roomname = new String(m_Places.elementAt(i).roomname);
                }
            }
            
            boolean add = true;
            for (int i = 0; i < m_People.size(); i++) {
                if (m_People.elementAt(i).address == p.address) {
                    //log("Person already existed");
                    m_People.elementAt(i).name = p.name;
                    m_People.elementAt(i).roomname = p.roomname;
                    m_People.elementAt(i).record = p.record;
                    add = false;
                    break;
                }
            }            

            if (add) m_People.add(p);

            
        } else if (sBelief.type.equals("VisualObject")) {

            VisualObject vo = getVisualObject(sBelief);
            
            for (int i = 0; i < m_Places.size(); i++) {
                if (m_Places.elementAt(i).id == vo.placeid) {
                    vo.roomname = new String(m_Places.elementAt(i).roomname);
                }
            }
            
            boolean add = true;
            for (int i = 0; i < m_VisualObjects.size(); i++) {
                if (m_VisualObjects.elementAt(i).address == vo.address) {
                    //log("VisualObject already existed");
                    m_VisualObjects.elementAt(i).roomname = vo.roomname;
                    add = false;
                    break;
                }
            }            
            
            if (add) m_VisualObjects.add(vo);
                     
        }        

        info = "\nInformation about the world\n============================\n";
        for (int i = 0; i < m_People.size(); i++) {
            info += "Person at address " + m_People.elementAt(i).address + " with name \"" + m_People.elementAt(i).name + "\" at place with id " + m_People.elementAt(i).placeid + " and in room \"" + m_People.elementAt(i).roomname + "\"\n";
        }
        for (int i = 0; i < m_VisualObjects.size(); i++) {
            info += "VisualObject at address " + m_VisualObjects.elementAt(i).address + " of category \"" + m_VisualObjects.elementAt(i).category + "\" at place with id " + m_VisualObjects.elementAt(i).placeid + " and in room \"" + m_VisualObjects.elementAt(i).roomname + "\"\n";
        }
        info += "----------- Rooms and places ------------\n";
        for (int i = 0; i < m_Rooms.size(); i++) {
            info += "Room at address " + m_Rooms.elementAt(i).address + " with name \"" + m_Rooms.elementAt(i).name + "\"\n";
        }
        for (int i = 0; i < m_Places.size(); i++) {
            info += "Place at address " + m_Places.elementAt(i).address + " with id " + m_Places.elementAt(i).id + " associated with room at address " + m_Places.elementAt(i).roomaddress + " and name \"" + 
                m_Places.elementAt(i).roomname + "\"\n";
        }

        String message = "";
        for (int i = 0; i < m_People.size(); i++) {
            if (m_People.elementAt(i).name != "UNKNOWN") {
                message += "There is a person named " + 
                    m_People.elementAt(i).name + " near place " +
                    m_People.elementAt(i).placeid + " in room " + 
                    m_People.elementAt(i).roomname + ". ";
            } else {
                message += "There is an unknown person near place " +
                    m_People.elementAt(i).placeid + " in room " + 
                    m_People.elementAt(i).roomname + ". ";
            }
        }
        for (int i = 0; i < m_VisualObjects.size(); i++) {
            message += "The object " +
                m_VisualObjects.elementAt(i).category + " is near place " +
                m_VisualObjects.elementAt(i).placeid + " in room " + 
                m_VisualObjects.elementAt(i).roomname + ". ";
        }
        message = "echo \"" + message + "\" | festival --tts";
        
        try {
            FileWriter fstream = new FileWriter("information.txt");
            BufferedWriter out = new BufferedWriter(fstream);
            out.write(info);
            out.close();

            FileWriter fstream2 = new FileWriter("delivermessage.sh");
            BufferedWriter out2 = new BufferedWriter(fstream2);
            out2.write(message);
            out2.close();

            String command = "chmod a+x delivermessage.sh";
            Runtime.getRuntime().exec(command);

        } catch (Exception e) {}
    }

        /**
         * Stolen from tools/castutils/src/java/castutils/viewer/ViewerGUI.java
         * 
         * @param fv
         * @return
         */
        public static String toString(FeatureValue fv) {
            String featStr="*";
            if (fv instanceof IntegerValue)
                featStr=Integer.toString(((IntegerValue) fv).val);
            if (fv instanceof PointerValue)
                featStr = ((PointerValue) fv).beliefId.id + "@" + 
                    ((PointerValue) fv).beliefId.subarchitecture;
            //featStr=CASTUtils.toString(((PointerValue) fv).beliefId);
            if (fv instanceof StringValue)
                featStr=((StringValue) fv).val;
            if (fv instanceof FloatValue)
                featStr= Double.toString(((FloatValue) fv).val);
            if (fv instanceof BooleanValue)
                featStr=Boolean.toString(((BooleanValue) fv).val);
            return featStr;
        }

    }
