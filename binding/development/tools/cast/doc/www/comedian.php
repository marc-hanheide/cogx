<!DOCTYPE html PUBLIC "-//W3C//DTD XHTML 1.0 Strict//EN"
    "http://www.w3.org/TR/xhtml1/DTD/xhtml1-strict.dtd">

<html xmlns="http://www.w3.org/1999/xhtml">
<head>
  <? require_once("include/functions.php") ?>
  <meta name="generator" content=
  "HTML Tidy for Linux/x86 (vers 1st March 2005), see www.w3.org" />
  <meta http-equiv="content-type" content=
  "text/html; charset=us-ascii" />
  <link rel="icon" href="images/icon.png" type="image/png" />

  <title>CAST &gt; Examples</title>
<style type="text/css" media="all">
   /*<![CDATA[*/
   @import "css/cosy.css";
/*]]>*/
</style>
</head>

<body>
  <?php
      $string = get_include_contents('include/header.php');
      echo $string;
      ?>

  <div class="content">
    <h2>The Comedian Example</h2>

    <p>The example code presents a very simple architecture design.
    The example is based on the idea of 2 comedians: a straight man
    who sets up the joke, and a funny man who delivers the
    punchline. Once a joke has been told, a director asks the
    audience for its reaction. The joke telling happens on the
    stage, which is (oddly) separated from the audience so that the
    joke must be transmitted to the audience before they can
    deliver their reaction. In (vaguely) architectural terms we
    have the following....</p>

    <h3>The Stage Subarchitecture</h3>

    <p>The stage subarchitecture contains one managed process and
    one unmanaged process. The straight man is the unmanaged
    process (in terms of our architecture design). This process
    just waits for a little while before setting up a joke with a
    setup line. This is realised in the code by the process writing
    a joke struct into the stage working memory with a string for
    the setup, but (crucially!) an empty punchline. When this is
    added to working memory this triggers a change message to be
    broadcast to the other components in the subarchitecture. In
    this case only the funny man is listening and using this change
    information the funny man component gets the joke struct from
    working memory and provides a (much-needed) punchline.</p>

    <h3>The Audience Subarchitecture</h3>

    <p>The audience subarchitecture contains a single managed
    process that waits for jokes to appear in its subarchitecture
    working memory. When this occurs it writes its reaction to the
    joke back into working memory.</p>

    <h3>The Director Subarchitecture</h3>

    <p>In addition to these processing components the example
    contains a directing subarchitecture that manages the
    interaction between the stage and the audience. This
    subarchitecture contains an assistant director that suggests
    things for a director to do, and the director itself. The job
    of these two components is to send a completed joke to the
    audience from the stage and to send the resulting reaction
    back.</p>
  </div>

  <div class="content">
    <h2>Running The Example</h2>

    <p>Assuming you have CAST <a href="install.php">installed</a>
    correctly, you should be able to follow the instructions below
    to run the comedian example from a terminal.</p>

    <ol>
      <li>Set your library path to include the directory where the
      CAST library files were installed (e.g. $OUTPUT_DIR/lib).
      This can be accomplished using the LD_LIBRARY_PATH variable
      on Linux or DYLD_LIBRARY_PATH on a Mac.</li>

      <li>Start the Java implementation of the CORBA
      nameserver:<br />
      <code>tnameserv -ORBInitialPort 1050 &amp;</code></li>

      <li>Next, start the demo. The basic command is as follows
      (all 1 line):<br />
      <code>java -classpath $CLASSPATH:$OUTPUT_DIR/classes/
      cast.server.CASTProcessServer -h localhost -f
      $CAST_HOME/src/comedian-architecture/config/comedian-architecture.cast</code><br />

      Under Mac OS X you may need to tell java about the library
      path, so add something like
      -Djava.library.path=$OUTPUT_DIR/lib to the command line.</li>

      <li>This should produce some output like the following:<br />

        <pre>
["straight.man": ahem...]
[LOG "funny.man": *cough*]
["ass.director": is that a joke I hear?!]
["director": someone told a joke! ]
["director": let's see what the audience thinks of...]
["director": Q: What's brown and sticky?]
["director": A: A stick!]
["audience.member": (takes deep breath)]
["ass.director": is that a reaction I hear?!]
["director": and the audience says...]
["director": YAY!]
</pre>
      </li>
    </ol>

    <p>The code used in the example is explained in more detail
    below.</p>
  </div>

  <div class="content">
    <h2>Component Classes</h2>

    <p>Every processing component in the example is derived from a
    component from CAST. Essentially this means that each component
    runs in its own thread which is lauched by the server,
    communicates with other components via working memory, and can
    be written in either Java or C++. The inheritance hierarchy of
    the CAST components does its best to hide a lot of the
    underlying detail of communication from the end user whilst
    giving you the functionality specified by the schema. This can
    be summed up by looking at the inheritance hierarchy for the
    FunnyMan class in the demo). In brief (with slight variations
    across languages) this looks like...</p>

    <p><strong>FrameworkProcess</strong> [<a href=
    "doxygen/html/classFrameworkProcess.html">C++</a>][<a href=
    "doxygen/html/classbalt_1_1core_1_1processes_1_1FrameworkProcess.html">Java</a>]:
    Basic threading and underlying ability to be connect to other
    processes.</p>

    <p><strong>InspectableComponent</strong> [<a href=
    "doxygen/html/classcast_1_1InspectableComponent.html">C++</a>][<a href="doxygen/html/classcast_1_1core_1_1components_1_1InspectableComponent.html">Java</a>]:
    Supports specific connections to a GUI for general 2D, 3D and
    text rendering.</p>

    <p><strong>CASTComponent</strong> [<a href=
    "doxygen/html/classcast_1_1CASTComponent.html">C++</a>][<a href="doxygen/html/classcast_1_1core_1_1components_1_1CASTComponent.html">Java</a>]:
    CAST-specific printing and run mechanisms.</p>

    <p><strong>CASTUIComponent</strong> [<a href=
    "doxygen/html/classcast_1_1CASTUIComponent.html">C++</a>][<a href="doxygen/html/classcast_1_1core_1_1components_1_1CASTUIComponent.html">Java</a>]:
    Overrides output methods for GUI connections if required.</p>

    <p><strong>CASTProcessingComponent</strong> [<a href=
    "doxygen/html/classcast_1_1CASTProcessingComponent.html">C++</a>][<a href="doxygen/html/classcast_1_1core_1_1components_1_1CASTProcessingComponent.html">Java</a>]:
    Ability to understand CAST-data using an ontology specified by
    a derived class.</p>

    <p><strong>WorkingMemoryReaderProcess</strong> [<a href=
    "doxygen/html/classcast_1_1WorkingMemoryReaderProcess.html">C++</a>],
    <strong>WorkingMemoryWriterProcess</strong> [<a href=
    "doxygen/html/classcast_1_1WorkingMemoryWriterProcess.html">C++</a>],
    <strong>WorkingMemoryReaderWriterProcess</strong> [<a href=
    "doxygen/html/classcast_1_1architecture_1_1abstr_1_1WorkingMemoryReaderWriterProcess.html">Java</a>]:
    Methods to read and write to working memory, and to receive
    information about changes to working memory.</p>

    <p><strong>ManagedProcess</strong> [<a href=
    "doxygen/html/classcast_1_1ManagedProcess.html">C++</a>][<a href="doxygen/html/classcast_1_1architecture_1_1subarchitecture_1_1ManagedProcess.html">Java</a>]:
    Combines all of the above with methods to communicate with a
    subarchitecture task manager (i.e. propose processing tasks and
    receive responses to these).</p>

    <p><strong>FunnyMan</strong> [<a href=
    "doxygen/html/classFunnyMan.html">C++</a>][<a href=
    "doxygen/html/classcomedyarch_1_1subarchitectures_1_1stage_1_1FunnyMan.html">Java</a>]:
    Instantiation-specific class that uses all of the previously
    described functionality.</p>
  </div>

  <div class="content">
    <h2>Data and Ontologies</h2>

    <p>All data items communicated between processing components in
    the example (and in CAST in general) have 2 crucial parts.</p>

    <p>The first part is the data itself. It must either be a
    primitive type, and array or sequence of a primitive type, a
    struct defined in IDL, or an array of such structs. The IDL
    part is required to allow the underlying framework to translate
    the data across languages and transmit across network
    connections using CORBA. IDL defined types must be explicitly
    added to the framework at runtime so that translation code can
    be dynamically built (which is done before processes actually
    execute). This done automatically by the
    <strong>CASTOntology</strong> [<a href=
    "doxygen/html/classcast_1_1CASTOntology.html">C++</a>][<a href=
    "doxygen/html/classcast_1_1core_1_1ontologies_1_1CASTOntology.html">Java</a>]
    class when you establish a mapping between a datatype and an
    ontological type. And example of this is in the ComedyOntology
    class in Java or in C++.</p>

    <p>The second part of data is the ontological type. This a
    string that specifies what a particular data item represents.
    In order to translate data between processes each
    subarchitecture/architecture must maintain a mapping between
    underlying datatype (such as strings, ints or IDL-defined
    structs) and ontological types (such as jokes, responses,
    recognition results etc.). An ontological type can only have
    one datatype, but that datatype can be used for any number of
    ontological types. You can see this in action in the demo by
    looking at ComedyOntology in both <a href=
    "doxygen/html/classcomedyarch_1_1global_1_1ComedyOntology.html">
    Java</a> and <a href=
    "doxygen/html/classComedyOntology.html">C++</a>.</p>
  </div>

  <div class="content">
    <h2>A ManagedProcess in Detail</h2>

    <p>We can now illuminate some of this with a more detailed look
    at the FunnyMan processing component from the example. The Java
    and C++ code is nearly identical, so we will use the C++
    version. The full source is available for both <a href=
    "doxygen/html/FunnyMan_8java-source.html">Java</a> and <a href=
    "doxygen/html/FunnyMan_8cpp-source.html">C++</a>.</p>

    <p>Any component that can read from working memory receives
    updates about when the working memory in its subarchitecture
    has changed in some way. This information is fed to the derived
    class via a collection of WorkingMemoryChangeReceiver [<a href=
    "doxygen/html/classcast_1_1WorkingMemoryChangeReceiver.html">C++</a>][<a href="doxygen/html/interfacecast_1_1architecture_1_1abstr_1_1WorkingMemoryChangeReceiver.html">Java</a>]
    objects that filter working memory changes for processing based
    on the the ontological type of the changes, operation type,
    source component and other information. The FunnyMan adds a
    change filter for jokes with the following code:</p>
    <pre>
00068   //create a WorkingMemoryChangeReceiver that automatically calls a
00069   //member function of this class
00070   MemberFunctionChangeReceiver&lt;FunnyMan&gt; * pReceiver = 
00071     new MemberFunctionChangeReceiver&lt;FunnyMan&gt;(this,
00072                                                &amp;FunnyMan::newJokeAdded);
00073     
00074   //add this receiver to listen for changes
00075   addChangeFilter(//listen for joke types
00076                   ComedyOntology::JOKE_TYPE, 
00077                   //that are added
00078                   cdl::ADD, 
00079                   //that are local to this subarchitecture
00080                   true,
00081                   //the receiver object
00082                   pReceiver);  
</pre>

    <p>Once a relevant change is received, FunnyMan pulls the
    information from working memory:</p>
    <pre>
00118   // get the id of the changed entry
00119   string id(_wmc.m_address.m_id);
00120 
00121   // get the data from working memory
00122   shared_ptr&lt; const CASTData&lt;ComedyEssentials::Joke&gt; &gt; pJokeData 
00123     = getWorkingMemoryEntry&lt;ComedyEssentials::Joke&gt;(id);
</pre>

    <p>And then gets at the underlying object via the wrapper:</p>
    <pre>
00125   //now get at a reference to the actual joke
00126   shared_ptr&lt;const ComedyEssentials::Joke&gt; jk = pJokeData-&gt;getData();
</pre>

    <p>The FunnyMan class then checks to see if this is a joke
    needing a punchline, in which case it must propose a task to
    supply one (in line with the abstract architecture design).</p>
    <pre>
00128   string punchline(jk-&gt;m_punchline);
00129   
00130   // now see if we can provide a punchline
00131   if (punchline == "") {
00132     
00133     // in this case we need to propose a task to do
00134     // some processsing
00135     
00136     // get a new id for the task
00137     string taskID = newTaskID();
00138     //println(jk-&gt;m_setup);
00139     
00140     // store the data we want to process for
00141     // later... we could just store the id in
00142     // working memory if we wanted to, I guess it
00143     // depends on storage/transport tradeoffs
00144     (*m_pProposedProcessing)[taskID] = pJokeData;
00145     
00146     // then ask for permission
00147     proposeInformationProcessingTask(taskID,
00148                                      ComedyGoals::ADD_PUNCHLINE_TASK);
00149     
00150     //then immediately retract task
00151     //retractInformationProcessingTask(taskID);
00152   }
00153   else {
00154     // nothing to do then :(
00155   }
</pre>

    <p>The final call pushes a task proposal to the subarchitecture
    task manager. After this the component must wait for a
    response. The response comes via either the taskAdopted or
    taskRejected methods. In this case we will look at the
    former.</p>
    <pre>
00163 void FunnyMan::taskAdopted(const string &amp;_taskID) {
00164     
00165   TypedDataMap::iterator i = m_pProposedProcessing-&gt;find(_taskID);
00166 
00167   // if we have stored this goal earlier
00168   if (i != m_pProposedProcessing-&gt;end()) {
00169     // we could call quip(data) directly, but let's queue the
00170     // data instead... it's always good to build some suspense
00171     m_pSetups-&gt;push_back(i-&gt;second);
00172     //and take it out of the prosed list
00173     m_pProposedProcessing-&gt;erase(i);
00174   }
00175   else {
00176     println("oh, this is my goal, but I have no data: "
00177             + _taskID);
00178   }
00179   
00180   // and now we're finished, tell the goal manager that the task is
00181   // over successfully (assuming it is... naughty!)
00182   taskComplete(_taskID,cdl::PROCESSING_COMPLETE_SUCCESS);
00183 
00184 }
</pre>

    <p>The above code first checks that its has some stored data
    for the goal (components only receive adopts and rejects for
    goals they've proposed though), then adds the data to a queue
    for later processing. After this it signals to the task manager
    that its task is complete. This is actually incorrect because
    the data is still queued but it demonstrates that flow of
    control in the toolkit is fairly flexible. The queue is
    processed in the components runComponent method, which every
    component has. This method represents the main processing
    thread for each component. In this case the component just
    checks the queue for data to processes.</p>
    <pre>
00198 void FunnyMan::runComponent() {
00199   
00200   while(m_status == STATUS_RUN) {
00201 
00202     
00203     // must check that we're still running after sleep
00204     if(m_status == STATUS_RUN) {
00205 
00206       //prevent external access
00207       lockProcess();
00208 
00209       // check (synchronised) joke queue
00210       TypedDataVector::iterator i = m_pSetups-&gt;begin();
00211 
00212       // see what's in there
00213       while (i != m_pSetups-&gt;end()) {
00214         
00215         // quick, tell the punchline
00216         quip(*i);
00217 
00218         // erase and move to the next point in the list
00219         i = m_pSetups-&gt;erase(i);
00220 
00221       }
00222 
00223       unlockProcess();
00224     }
00225 
00226   }
00227 }
</pre>

    <p>Although nothing really happens in the above (the quip
    method does a lot of the work), it highlights the fact that it
    is desirable (i.e. necessary) to lock the execution thread when
    doing processing in this method. For all components the mutex
    is used to stop asynchronous input to the component via
    superclasses. This means that no new data is received whilst
    the mutex is locked. Obviously a derived client can choose not
    to lock the mutex in the runComponent method, but if they then
    interact with data that is also being operated on by the
    asynchronously called methods (task adoption, working memory
    changes) they could be in parallel processing-style
    trouble!</p>

    <p>The quip method is the final part of FunnyMan's
    functionality, and the part where it actually does some
    processing. It starts off by calling another method to actually
    create the the punchline for the joke.</p>
    <pre>
00233 void FunnyMan::quip(shared_ptr&lt; const CASTData&lt;ComedyEssentials::Joke&gt; &gt; _pData) {
00234   
00235   //create a new copy of the data so we can change it
00236   ComedyEssentials::Joke *pJK = new ComedyEssentials::Joke(*_pData-&gt;getData());
00237 
00238 
00239   string setup(pJK-&gt;m_setup);
00240   // work out a smarty-pants punchline
00241   string punchline = generatePunchline(setup);
</pre>

    <p>It then writes itsdata into the struct it received (a lot)
    earlier as input. (please tolerate the tiny bit of CORBA, it's
    only necessary in C++ when detailing with strings!).</p>
    <pre>
00243   // time it right
00244   println("*cough*");
00245   pJK-&gt;m_punchline = CORBA::string_dup(punchline.c_str());
</pre>

    <p>And finally call a method from a superclass to write the
    data back into working memory with the same id as before.</p>
    <pre>
00247   // now write this back into working memory, this will manage the
00248   // memory for us
00249   overwriteWorkingMemory(_pData-&gt;getID(),
00250                          ComedyOntology::JOKE_TYPE, pJK);
</pre>

    <p>In terms of C++ memory management, all data transfers
    (addToWorkingMemory and overwriteWorkingMemory) require
    pointers to data on the heap. The calling process loses
    ownership of the pointer and therefore must not access or
    delete the memory subsequently.</p>
  </div>

  <div class="content">
    <h2>Configuration</h2>

    <p>The configuration for the example comedian architecture is
    defined in the <code>.cast</code> file passed as an argument to
    the CASTProcessServer at run time. The format for this file is
    described in detail on the <a href="config.php">configuration
    page</a>.</p>
  </div>


<?php
            $string = get_include_contents('include/footer.php');
          echo $string;

          $string = get_include_contents('include/menu.php');
          echo $string;
          ?>
</body>
</html>
