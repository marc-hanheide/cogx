/**
 * 
 */
package binding.monitors.fakecomsys;

import java.util.*;

//import org.cognitivesystems.comsys.autogen.ComsysEssentials.PhonString;
//import org.cognitivesystems.comsys.ontology.ComsysOntology;

import binding.abstr.AbstractMonitor;
import binding.common.BindingComponentException;
import BindingData.*;
import BindingFeatures.*;
import BindingFeaturesCommon.*;
import cast.architecture.abstr.WorkingMemoryChangeReceiver;
import cast.architecture.subarchitecture.SubarchitectureProcessException;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.data.CASTData;
import cast.architecture.subarchitecture.PrivilegedManagedProcess;
import cast.cdl.WorkingMemoryChangeQueueBehaviour;
import cast.core.data.CASTData;
import balt.jni.NativeProcessLauncher;
import balt.corba.autogen.FrameworkBasics.BALTTime;

import javax.swing.JOptionPane;

/**
 * @author henrikj
 */
public class FakeComSysBindingMonitor extends AbstractMonitor {

    /** Used to keep track of proxies in binding working memory */
    private HashSet<String> m_nouns;
    private HashMap<String, String> m_groupNoun2Stem;
    private HashSet<String> m_colours;
    private HashSet<String> m_shapes;
    private HashSet<String> m_sizes;
    private HashSet<String> m_relations;

    // stores all the discrefs locally together with their BC addresses
    private TreeMap<String, DemoDiscRef> m_discRefs;

    private ArrayList<String> m_lastAddrs;

    public FakeComSysBindingMonitor(String _id) {
        super(_id);
        m_bLogOutput = true;
        
        m_lastAddrs = new ArrayList<String>();

        m_discRefs = new TreeMap<String, DemoDiscRef>();

        m_nouns = new HashSet<String>();
        m_nouns.add("cup");
        m_nouns.add("mug");
        m_nouns.add("ball");
        m_nouns.add("apple");
        m_nouns.add("banana");
        m_nouns.add("cow");
        m_nouns.add("pen");
        m_nouns.add("computer");
        m_nouns.add("desk");
        m_nouns.add("office");
        m_nouns.add("robot");
        m_nouns.add("human");
        m_nouns.add("mouse");
        m_nouns.add("book");
        m_nouns.add("box");
        m_nouns.add("thing");
        m_nouns.add("cups");
        m_nouns.add("mugs");
        m_nouns.add("balls");
        m_nouns.add("apples");
        m_nouns.add("bananas");
        m_nouns.add("cows");
        m_nouns.add("pens");
        m_nouns.add("computers");
        m_nouns.add("desks");
        m_nouns.add("offices");
        m_nouns.add("robots");
        m_nouns.add("humans");
        m_nouns.add("mice");
        m_nouns.add("books");
        m_nouns.add("boxes");
        m_nouns.add("things");

        m_groupNoun2Stem = new HashMap<String, String>();
        m_groupNoun2Stem.put("cups", "cup");
        m_groupNoun2Stem.put("mugs", "mug");
        m_groupNoun2Stem.put("balls", "ball");
        m_groupNoun2Stem.put("apples", "apple");
        m_groupNoun2Stem.put("bananas", "banana");
        m_groupNoun2Stem.put("cows", "cow");
        m_groupNoun2Stem.put("pens", "pen");
        m_groupNoun2Stem.put("computers", "computer");
        m_groupNoun2Stem.put("desks", "desk");
        m_groupNoun2Stem.put("offices", "office");
        m_groupNoun2Stem.put("robots", "robot");
        m_groupNoun2Stem.put("humans", "human");
        m_groupNoun2Stem.put("mice", "mouse");
        m_groupNoun2Stem.put("books", "book");
        m_groupNoun2Stem.put("boxes", "box");
        m_groupNoun2Stem.put("things", "thing");

        m_colours = new HashSet<String>();
        m_colours.add("red");
        m_colours.add("blue");
        m_colours.add("yellow");
        m_colours.add("green");
        m_colours.add("orange");
        m_colours.add("black");
        m_colours.add("white");
        m_colours.add("gray");
        m_colours.add("grey");
        m_shapes = new HashSet<String>();
        m_shapes.add("long");
        m_shapes.add("short");
        m_shapes.add("round");
        m_shapes.add("squared");
        m_shapes.add("boxy");
        m_shapes.add("hollow");
        m_shapes.add("spherical");
        m_shapes.add("elongated");
        m_shapes.add("smashed");
        m_shapes.add("flat");
        m_shapes.add("circular");
        m_shapes.add("quadratical");
        m_shapes.add("triangular");
        m_shapes.add("rectangular");
        m_shapes.add("hexagonal");
        m_sizes = new HashSet<String>();
        m_sizes.add("small");
        m_sizes.add("big");
        m_sizes.add("huge");
        m_sizes.add("tiny");
        m_sizes.add("sizeable");
        m_sizes.add("large");
        m_sizes.add("enormous");
        m_sizes.add("microscopical");
        m_sizes.add("astronomical");
        m_sizes.add("quantumscale");
        m_sizes.add("mansized");
        m_sizes.add("50ft");
        m_relations = new HashSet<String>();
        m_relations.add("left");
        m_relations.add("on");
        m_relations.add("right");
        m_relations.add("above");
        m_relations.add("under");
        m_relations.add("in");
        m_relations.add("then");
        m_relations.add("and");
        m_relations.add("or");
        m_relations.add("near");
    }

    
    @Override
    public void configure(Properties _config) {
        super.configure(_config);
    }

    /*
     * (non-Javadoc)
     * 
     * @see cast.architecture.abstr.WorkingMemoryReaderProcess#start()
     */
    @Override
    public void start() {
        super.start();
	/*     addChangeFilter(ComsysOntology.PHONSTRING_TYPE,
		   WorkingMemoryOperation.ADD, true,
		   new WorkingMemoryChangeReceiver() {
		   
		   public void workingMemoryChanged(WorkingMemoryChange _wmc) {
		   newPhonStringAdded(_wmc);
		   }
		   });*/
	if (m_sourceID == null) {
	    m_sourceID = "Demo_fake_comsys_system";
	    // it should look more like this in your monitor:
	    //m_sourceID = _wmc.m_address.m_subarchitecture;
	}
	
    }
    
    @Override
	public void runComponent() {
/*	try{
	    for(int i = 0 ; i < 20 ; i++) {
		startNewBasicProxy();
		RelationLabel rel = new RelationLabel();
		rel.m_label = "fakelabel" + i;
		addFeatureToCurrentProxy(rel);
		storeCurrentProxy();
	    }
	    bindNewProxies();
	    sleepProcess(200);
	    for(int i = 10 ; i < 14 ; i++) {
		startNewBasicProxy();
		RelationLabel rel = new RelationLabel();
		rel.m_label = "fakelabel" + i;
		addFeatureToCurrentProxy(rel);
		storeCurrentProxy();
	    }
	    bindNewProxies();

	}
	catch(BindingComponentException _e){
	    log("test gubar");
	}
	*/
	try{
	    while(true) {
		String text = JOptionPane.showInputDialog(null,
							  "-<FAKE COMSYS>- Please type in a simplified sentence",
							  "input", JOptionPane.QUESTION_MESSAGE);
//                deleteOldProxies();
		createAndStoreBindingProxiesFromPhonString(text);
		bindNewProxies();
	    }
	}
	catch (SubarchitectureProcessException e) {
	    e.printStackTrace();
	}
    }
    
    /**
     * (non-Javadoc)
     * 
     * @see cast.architecture.subarchitecture.ManagedProcess#taskAdopted(java.lang.String)
     */
    @Override
    protected void taskAdopted(String _goalID) {

    }

    /**
     * (non-Javadoc)
     * 
     * @see cast.architecture.subarchitecture.ManagedProcess#taskRejected(java.lang.String)
     */
    @Override
    protected void taskRejected(String _goalID) {}

    /*
     * private WorkingMemoryAddress storeColours(TreeSet<String>
     * _colours;) { }
     */

    /**
     * creates the Binding Proxy and stores it. Returns the WM
     * address.
     */
    private String encodeDemoDiscRefAndStoreBindingProxy(DemoDiscRef _dr)//,
//                                                             WorkingMemoryChange _wmc)
            throws SubarchitectureProcessException {
        // initiate current proxy

        if (!_dr.m_noun.equals("")
            && m_groupNoun2Stem.get(_dr.m_noun) != null) { // i.e., if
                                                            // we have a
                                                            // noun and
                                                            // it's a
                                                            // plural
            startNewGroupProxy((short) 0); // < 0 to signify an
                                                // unbounded group
        }
        else {
            startNewBasicProxy();
        }

	// TEST
	//addOtherSourceIDToCurrentProxy(m_sourceID,true); // will prevent binding to anything from the same subarch as this

        // 1st, store the SourceID since it's required. The inherited
        // method from AbstractMonitor can be used.
//        addSourceIDToCurrentProxy();

        // Another simple thing to add is the creation time. Very
        // useful for debugging. Could be used to eliminate "old"
        // proxies too.
  //      addCreationTimeToCurrentProxy();

        // Just to demonstrate, store a reference to the original
        // DemoDiscRef too.
	
        /*
	  SourceData sd = new SourceData();
	  sd.m_type = ComsysOntology.PHONSTRING_TYPE;
	  sd.m_address = _wmc.m_address;
	  sd.m_comparable = false;

	  // after data is set in your feature, then store it onto the
	  // proxy
	  addFeatureToCurrentProxy(sd);
	*/

        // Store the noun
        if (!_dr.m_noun.equals("")) {
            Concept concept = new Concept();
            String noun = null;
            // make sure to store the stem of the noun:
            if (m_groupNoun2Stem.get(_dr.m_noun) == null) {
                noun = _dr.m_noun;
            }
            else {
                noun = m_groupNoun2Stem.get(_dr.m_noun);
                m_currentlyBuiltProxy.m_type = BindingProxyType.GROUP;
            }
            concept.m_concept = noun;
	    
            // in lack of anything better... a noun is perhaps more
            // specific than a concept...
            addFeatureToCurrentProxy(concept, _dr.m_noun_truth_value);
        }

        // Store the colours
        if (!_dr.m_colours.isEmpty()) {
            Iterator<String> colour_iter = _dr.m_colours.iterator();

            while (colour_iter.hasNext()) {

                Colour colour = new Colour();
                colour.m_colour  = colour_iter.next();;

                addFeatureToCurrentProxy(colour, _dr.m_colours_truth_value);
            }
        }
	
	// size
	if(!_dr.m_size.equals("")) { 
	    Size size = new Size();
	    size.m_size = _dr.m_size; 

            addFeatureToCurrentProxy(size, _dr.m_size_truth_value);
	}	    

        // Store the shape
        if (!_dr.m_shape.equals("")) {
            Shape shape = new Shape();
            shape.m_shape = _dr.m_shape;
            addFeatureToCurrentProxy(shape, _dr.m_shape_truth_value);
        }
	
	if(_dr.m_hypothetical) {
	    makeCurrentProxyHypothetical();
	}

	addSalienceToCurrentProxy();
	
	// salience test:
/*	addSalienceToCurrentProxy();
	addSalienceToCurrentProxy(infinitePast(),infiniteFuture());
	addSalienceToCurrentProxy(infinitePast(),endTime(baltTime(3.1415)));
	addSalienceToCurrentProxy(startTime(baltTime(3.1415)),infiniteFuture());
	FeaturePointer ptr = addSalienceToCurrentProxy(startTime(baltTime(3.1415)),endTime(baltTime(20.2)));
	addSalienceToCurrentProxy(infinitePast(),infiniteFuture());
	addSalienceToCurrentProxy(infinitePast(),endTime(baltTime(3.1415)));
	addSalienceToCurrentProxy(startTime(baltTime(3.1415)),infiniteFuture());
	deleteFeatureFromCurrentProxy(ptr);
*/
/*	VisualSize vis = new VisualSize();
	vis.m_visualSize = 1;
	addFeatureToCurrentProxy(
*/
/*	TemporalFrame frame = new TemporalFrame();
	frame.m_temporalFrame = TemporalFrameType.DESIRED;
	addFeatureToCurrentProxy(frame);
	Name name = new Name();
	name.m_name = "Brian";
	addFeatureToCurrentProxy(name);*/
	
        String addr = storeCurrentProxy();
        return addr;
    }

    private ArrayList<DemoDiscRef> PhonString2DiscRefs(String _str) {
        ArrayList<DemoDiscRef> drs = new ArrayList<DemoDiscRef>();
        // ArrayList<String> rels = new ArrayList<String>();

        Boolean dr_finished = false;
        DemoDiscRef dr = new DemoDiscRef();

        StringTokenizer st = new StringTokenizer(_str);
	
	TruthValue truth = TruthValue.POSITIVE;
	
        while (st.hasMoreTokens()) {
            String word = st.nextToken();
            if (word.equals("not")) {
		truth = TruthValue.NEGATIVE;
            }
            if (word.equals("maybe")) {
		dr.m_hypothetical = true;
            }
            if (m_colours.contains(word)) {
                dr.m_colours.add(word);
		dr.m_colours_truth_value = truth;
		truth = TruthValue.POSITIVE;
            }
            else if (m_sizes.contains(word)) {
                dr.m_size = word;
		dr.m_size_truth_value = truth;
		truth = TruthValue.POSITIVE;
            }
            else if (m_shapes.contains(word)) {
                dr.m_shape = word;
		dr.m_shape_truth_value = truth;
		truth = TruthValue.POSITIVE;
            }
            else if (m_nouns.contains(word)) {
                dr.m_noun = word;
		dr.m_noun_truth_value = truth;
		truth = TruthValue.POSITIVE;
            }
            else if (m_relations.contains(word)) {
                // a relation, add existing disc ref and continue with
                // next after storing the relation
                dr.m_relation = word;
                // if possible, replace relation label with label from
                // idl-specified dictionary of relations
                if (dr.m_relation.equalsIgnoreCase("on")) {
                    dr.m_relation = "Rel:TopOf";
                }
                else if (dr.m_relation.equalsIgnoreCase("under")) {
                    dr.m_relation = "Rel:TopOf";
                    dr.m_relationForward = false;
                }
                else if (dr.m_relation.equalsIgnoreCase("right")) {
                    dr.m_relation = "Rel:RightOf";
                    dr.m_relationForward = false; // eh... not
                    // logical, but the
                    // result looks
                    // better!
                }
                else if (dr.m_relation.equalsIgnoreCase("left")) {
                    dr.m_relation = "Rel:RightOf";
                }
                else if (dr.m_relation.equalsIgnoreCase("in")) {
                    dr.m_relation = "Rel:In";
                }
                else if (dr.m_relation.equalsIgnoreCase("above")) {
                    dr.m_relation = "Rel:Above";
                }
                else if (dr.m_relation.equalsIgnoreCase("near")) {
                    dr.m_relation = "Rel:Nearby";
                }
                else if (dr.m_relation.equalsIgnoreCase("then")) {
                    dr.m_relation = "Rel:FutureOf";
                }
                dr_finished = true;
            }
            if (dr_finished || !st.hasMoreTokens()) {
                dr_finished = false;
                drs.add(dr);
                dr = new DemoDiscRef();
            }
        }
        return drs;
    }

    // adds a bunch of proxies from a phon string onto WM
    protected void createAndStoreBindingProxiesFromPhonString(String _str)
	//,
	//						    WorkingMemoryChange _wmc)
	throws SubarchitectureProcessException {
        log("Going to transform this into a proxy: \""
            + _str + "\"");

        ArrayList<DemoDiscRef> drs = PhonString2DiscRefs(_str);
        Iterator<DemoDiscRef> iter = drs.iterator();

        String last_addr = null;
        String last_relation = null;
        while (iter.hasNext()) {

            DemoDiscRef dr = iter.next();

            String addr;
            // store the discref as a proxy
            addr = encodeDemoDiscRefAndStoreBindingProxy(dr);//, _wmc);

            registerProxy(addr);

            // store the relation. Very simple example here... the
            // relations can only be made from one disc ref to the
            // next in sequence
            if (last_addr != null) {
                String relAddr;

                if (dr.m_relationForward) {
                  relAddr = addSimpleRelation(last_addr, addr, last_relation, TemporalFrameType.ASSERTED);
                }
                else {
                  relAddr = addSimpleRelation(addr, last_addr, last_relation, TemporalFrameType.ASSERTED);
                }
                registerProxy(relAddr);
            }
            last_addr = addr;
            last_relation = dr.m_relation;

            // store discRef also locally (could be useful)
            m_discRefs.put(addr, dr);
        }
    }

    // deletes fakecomsys proxy from the previous utterance
    private void deleteOldProxies() throws SubarchitectureProcessException
    {
      for(int i=0; i < m_lastAddrs.size(); i++)
        deleteExistingProxy(m_lastAddrs.get(i));

      m_lastAddrs.clear();
    }

    // saves proxy address to a table
    private void registerProxy(String addr)
    {
      m_lastAddrs.add(addr);
    }



/*    private void newPhonStringAdded(WorkingMemoryChange _wmc) {
        try {
            // 1st, set the ID of the monitored subarchitecture. The
            // assumption is that the first process to suggest a
            // phonString is indeed the correct one... This may change
            // if the monitors are instead put inside the SAs they
            // monitor.
            if (m_sourceID == null) {
                m_sourceID = _wmc.m_address.m_subarchitecture;
            }

            // added or updated relation (only additions are
            // implemented at the moment...)
            log("an added phonstring...");

            // get the id of the working memory entry
            String phonId = _wmc.m_address.m_id;
            String subArchId = _wmc.m_address.m_subarchitecture;

            // get the data from working memory and store it
            // with its id
            CASTData phonstr = getWorkingMemoryEntry(phonId, subArchId);
            PhonString pstr = (PhonString) phonstr.getData();

            // Create the proxies
            createAndStoreBindingProxiesFromPhonString(pstr, _wmc);

            // now trigger the binder
            bindNewProxies();
        }
        catch (SubarchitectureProcessException e) {
            println(e.getLocalizedMessage());
            e.printStackTrace();
        }
    }
*/

}
