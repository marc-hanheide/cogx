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
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.SubarchitectureComponentException;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
import cast.core.CASTData;
import cast.architecture.ManagedComponent;
import cast.cdl.WorkingMemoryChangeQueueBehaviour;
import cast.core.CASTData;
//import balt.jni.NativeProcessLauncher;
//import balt.corba.autogen.FrameworkBasics.BALTTime;

import javax.swing.JOptionPane;

/**
 * @author henrikj
 */
public class FakeComSysBindingMonitor extends AbstractMonitor {

    /** Used to keep track of proxies in binding working memory */
    private HashSet<String> nouns;
    private HashMap<String, String> groupNoun2Stem;
    private HashSet<String> colours;
    private HashSet<String> shapes;
    private HashSet<String> sizes;
    private HashSet<String> relations;

    // stores all the discrefs locally together with their BC addresses
    private TreeMap<String, DemoDiscRef> discRefs;

    private ArrayList<String> lastAddrs;

    public FakeComSysBindingMonitor(String _id) {
        super(_id);
     //   bLogOutput = true;
        
        lastAddrs = new ArrayList<String>();

        discRefs = new TreeMap<String, DemoDiscRef>();

        nouns = new HashSet<String>();
        nouns.add("cup");
        nouns.add("mug");
        nouns.add("ball");
        nouns.add("apple");
        nouns.add("banana");
        nouns.add("cow");
        nouns.add("pen");
        nouns.add("computer");
        nouns.add("desk");
        nouns.add("office");
        nouns.add("robot");
        nouns.add("human");
        nouns.add("mouse");
        nouns.add("book");
        nouns.add("box");
        nouns.add("thing");
        nouns.add("cups");
        nouns.add("mugs");
        nouns.add("balls");
        nouns.add("apples");
        nouns.add("bananas");
        nouns.add("cows");
        nouns.add("pens");
        nouns.add("computers");
        nouns.add("desks");
        nouns.add("offices");
        nouns.add("robots");
        nouns.add("humans");
        nouns.add("mice");
        nouns.add("books");
        nouns.add("boxes");
        nouns.add("things");

        groupNoun2Stem = new HashMap<String, String>();
        groupNoun2Stem.put("cups", "cup");
        groupNoun2Stem.put("mugs", "mug");
        groupNoun2Stem.put("balls", "ball");
        groupNoun2Stem.put("apples", "apple");
        groupNoun2Stem.put("bananas", "banana");
        groupNoun2Stem.put("cows", "cow");
        groupNoun2Stem.put("pens", "pen");
        groupNoun2Stem.put("computers", "computer");
        groupNoun2Stem.put("desks", "desk");
        groupNoun2Stem.put("offices", "office");
        groupNoun2Stem.put("robots", "robot");
        groupNoun2Stem.put("humans", "human");
        groupNoun2Stem.put("mice", "mouse");
        groupNoun2Stem.put("books", "book");
        groupNoun2Stem.put("boxes", "box");
        groupNoun2Stem.put("things", "thing");

        colours = new HashSet<String>();
        colours.add("red");
        colours.add("blue");
        colours.add("yellow");
        colours.add("green");
        colours.add("orange");
        colours.add("black");
        colours.add("white");
        colours.add("gray");
        colours.add("grey");
        shapes = new HashSet<String>();
        shapes.add("long");
        shapes.add("short");
        shapes.add("round");
        shapes.add("squared");
        shapes.add("boxy");
        shapes.add("hollow");
        shapes.add("spherical");
        shapes.add("elongated");
        shapes.add("smashed");
        shapes.add("flat");
        shapes.add("circular");
        shapes.add("quadratical");
        shapes.add("triangular");
        shapes.add("rectangular");
        shapes.add("hexagonal");
        sizes = new HashSet<String>();
        sizes.add("small");
        sizes.add("big");
        sizes.add("huge");
        sizes.add("tiny");
        sizes.add("sizeable");
        sizes.add("large");
        sizes.add("enormous");
        sizes.add("microscopical");
        sizes.add("astronomical");
        sizes.add("quantumscale");
        sizes.add("mansized");
        sizes.add("50ft");
        relations = new HashSet<String>();
        relations.add("left");
        relations.add("on");
        relations.add("right");
        relations.add("above");
        relations.add("under");
        relations.add("in");
        relations.add("then");
        relations.add("and");
        relations.add("or");
        relations.add("near");
    }

    
    @Override
    public void configure(Map<String,String> _config) {
        super.configure(_config);
    }

    /*
     * (non-Javadoc)
     * 
     * @see cast.architecture.abstr.WorkingMemoryReaderComponent#start()
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
	if (sourceID == null) {
	    sourceID = "Demo_fake_comsys_system";
	    // it should look more like this in your monitor:
	    //sourceID = _wmc.address.subarchitecture;
	}
	
    }
    
    @Override
	public void runComponent() {
/*	try{
	    for(int i = 0 ; i < 20 ; i++) {
		startNewBasicProxy();
		RelationLabel rel = new RelationLabel();
		rel.labelstr = "fakelabel" + i;
		addFeatureToCurrentProxy(rel);
		storeCurrentProxy();
	    }
	    bindNewProxies();
	    sleepComponent(200);
	    for(int i = 10 ; i < 14 ; i++) {
		startNewBasicProxy();
		RelationLabel rel = new RelationLabel();
		rel.labelstr = "fakelabel" + i;
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
	catch (SubarchitectureComponentException e) {
	    e.printStackTrace();
	}
    }
    
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
            throws SubarchitectureComponentException {
        // initiate current proxy

        if (!_dr.noun.equals("")
            && groupNoun2Stem.get(_dr.noun) != null) { // i.e., if
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
	//addOtherSourceIDToCurrentProxy(sourceID,true); // will prevent binding to anything from the same subarch as this

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
	  sd.type = ComsysOntology.PHONSTRING_TYPE;
	  sd.address = _wmc.address;
	  sd.comparable = false;

	  // after data is set in your feature, then store it onto the
	  // proxy
	  addFeatureToCurrentProxy(sd);
	*/

        // Store the noun
        if (!_dr.noun.equals("")) {
            Concept concept = new Concept();
            String noun = null;
            // make sure to store the stem of the noun:
            if (groupNoun2Stem.get(_dr.noun) == null) {
                noun = _dr.noun;
            }
            else {
                noun = groupNoun2Stem.get(_dr.noun);
                currentlyBuiltProxy.type = BindingProxyType.GROUP;
            }
            concept.conceptstr = noun;
	    
            // in lack of anything better... a noun is perhaps more
            // specific than a concept...
            addFeatureToCurrentProxy(concept, _dr.noun_truth_value);
        }

        // Store the colours
        if (!_dr.colours.isEmpty()) {
            Iterator<String> colour_iter = _dr.colours.iterator();

            while (colour_iter.hasNext()) {

                Colour colour = new Colour();
                colour.colourstr  = colour_iter.next();;

                addFeatureToCurrentProxy(colour, _dr.colours_truth_value);
            }
        }
	
	// size
	if(!_dr.size.equals("")) { 
	    Size size = new Size();
	    size.sizestr = _dr.size; 

            addFeatureToCurrentProxy(size, _dr.size_truth_value);
	}	    

        // Store the shape
        if (!_dr.shape.equals("")) {
            Shape shape = new Shape();
            shape.shapestr = _dr.shape;
            addFeatureToCurrentProxy(shape, _dr.shape_truth_value);
        }
	
	if(_dr.hypothetical) {
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
	vis.visualSize = 1;
	addFeatureToCurrentProxy(
*/
/*	TemporalFrame frame = new TemporalFrame();
	frame.temporalFrame = TemporalFrameType.DESIRED;
	addFeatureToCurrentProxy(frame);
	Name name = new Name();
	name.name = "Brian";
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
	
	boolean truth = true;
	
        while (st.hasMoreTokens()) {
            String word = st.nextToken();
            if (word.equals("not")) {
		truth = false;
            }
            if (word.equals("maybe")) {
		dr.hypothetical = true;
            }
            if (colours.contains(word)) {
                dr.colours.add(word);
		dr.colours_truth_value = truth;
		truth = true;
            }
            else if (sizes.contains(word)) {
                dr.size = word;
		dr.size_truth_value = truth;
		truth = true;
            }
            else if (shapes.contains(word)) {
                dr.shape = word;
		dr.shape_truth_value = truth;
		truth = true;
            }
            else if (nouns.contains(word)) {
                dr.noun = word;
		dr.noun_truth_value = truth;
		truth = true;
            }
            else if (relations.contains(word)) {
                // a relation, add existing disc ref and continue with
                // next after storing the relation
                dr.relation = word;
                // if possible, replace relation label with label from
                // idl-specified dictionary of relations
                if (dr.relation.equalsIgnoreCase("on")) {
                    dr.relation = "Rel:TopOf";
                }
                else if (dr.relation.equalsIgnoreCase("under")) {
                    dr.relation = "Rel:TopOf";
                    dr.relationForward = false;
                }
                else if (dr.relation.equalsIgnoreCase("right")) {
                    dr.relation = "Rel:RightOf";
                    dr.relationForward = false; // eh... not
                    // logical, but the
                    // result looks
                    // better!
                }
                else if (dr.relation.equalsIgnoreCase("left")) {
                    dr.relation = "Rel:RightOf";
                }
                else if (dr.relation.equalsIgnoreCase("in")) {
                    dr.relation = "Rel:In";
                }
                else if (dr.relation.equalsIgnoreCase("above")) {
                    dr.relation = "Rel:Above";
                }
                else if (dr.relation.equalsIgnoreCase("near")) {
                    dr.relation = "Rel:Nearby";
                }
                else if (dr.relation.equalsIgnoreCase("then")) {
                    dr.relation = "Rel:FutureOf";
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
	throws SubarchitectureComponentException {
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

                if (dr.relationForward) {
                  relAddr = addSimpleRelation(last_addr, addr, last_relation, TemporalFrameType.ASSERTED);
                }
                else {
                  relAddr = addSimpleRelation(addr, last_addr, last_relation, TemporalFrameType.ASSERTED);
                }
                registerProxy(relAddr);
            }
            last_addr = addr;
            last_relation = dr.relation;

            // store discRef also locally (could be useful)
            discRefs.put(addr, dr);
        }
    }

    // deletes fakecomsys proxy from the previous utterance
    private void deleteOldProxies() throws SubarchitectureComponentException
    {
      for(int i=0; i < lastAddrs.size(); i++)
        deleteExistingProxy(lastAddrs.get(i));

      lastAddrs.clear();
    }

    // saves proxy address to a table
    private void registerProxy(String addr)
    {
      lastAddrs.add(addr);
    }



/*    private void newPhonStringAdded(WorkingMemoryChange _wmc) {
        try {
            // 1st, set the ID of the monitored subarchitecture. The
            // assumption is that the first process to suggest a
            // phonString is indeed the correct one... This may change
            // if the monitors are instead put inside the SAs they
            // monitor.
            if (sourceID == null) {
                sourceID = _wmc.address.subarchitecture;
            }

            // added or updated relation (only additions are
            // implemented at the moment...)
            log("an added phonstring...");

            // get the id of the working memory entry
            String phonId = _wmc.address.id;
            String subArchId = _wmc.address.subarchitecture;

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
