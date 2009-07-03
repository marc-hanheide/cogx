namespace cast {

/**
 * \page hello_world_start Hello World Tutorial
 *
 * \section hello_world_overview Overview
 *
 * This tutorial will provide a brief overview of the CoSy
 * Architecture Schema Toolkit (CAST) version 2, and give you enough
 * experience to start writing your own components in either C++ or
 * Java (just do the tutorial in one language or the other unless you
 * want to be really thorough). Even if you've used CAST v1 before you
 * should work through this tutorial to see all of the interface
 * changes that have taken place. The main changes are due to
 * switching from omniORB and Java CORBA to ZeroC's Ice for the
 * underlying communication middleware, but there have also been
 * changes to the build system and a few interface things
 * (particualarly in C++).
 *
 *
 * \section hello_world_task Tutorial Task
 * 
 * In this tutorial we will write a very simple "Hello World"
 * subarchitecture. This subarchitecture will contain two components:
 * one (called HelloWriter) which will write an object containing a
 * string to working memory, and another (called HelloReader) which
 * will read the object from working memory and print out the
 * string. In this tutorial we will write a system with a single
 * subarchitecture, but the lessons are applicable to systems with
 * mulitple subarchitectures (such as the other example, the comedian
 * subarchitecure).
 * 
 * 
 * \section hello_world_prerequisites     Prerequisites 
 *  
 * This tutorial assumes that you have \ref man_installation
 * "installed CAST" on your system. If this is the case, then we need
 * to create somewhere to put our tutorial code. It is current
 * practice to create subarchitectures in separate directories under a
 * directory called subarchitectures. So, to get started (assuming
 * you're in a directory called $TUTORIAL_ROOT) create a directory to hold
 * future subarchitectures, and a directory within that to contain
 * our tutorial example:
 * 
 \verbatim
 cd $TUTORIAL_ROOT
 mkdir -p subarchitectures/hello-world
 \endverbatim
 * 
 * Within a subarchitecture directory, we place source code under a
 * src directory split into different languages. Let's create these
 * directories next:
 * 
\verbatim
cd $TUTORIAL_ROOT
mkdir -p subarchitectures/hello-world/src/slice
mkdir -p subarchitectures/hello-world/src/c++
mkdir -p subarchitectures/hello-world/src/java
\endverbatim


\section hello_world_slice Writing A Slice File

All data structures that are to be written to working memory in CAST
must first be defined in the Ice interface definition languagae <a
href="http://www.zeroc.com/doc/Ice-3.3.1/reference/">Slice</a>. This
allows components to exchange information regardless of what language
they are written and where they are on a network. For more information
on Ice and Slice start <a
href="http://www.zeroc.com/ice.html">here</a>. It is also sensible to
keep a copy of the <a
href="http://www.zeroc.com/download/Ice/3.3/Ice-3.3.1.pdf">Ice
manual</a> handy as it is an excellent reference. Any WM object in
CAST must be a Slice <code>class</code>. For this example we will
create a class called <code>Announcement</code> to contain the message
we will pass between our components. This can be done as
follows. First create the file
\verbatim
subarchitectures/hello-world/src/slice/HelloWorldData.ice 
\endverbatim

then edit it to add the following:

\verbatim
#ifndef HWD_ICE
#define HWD_ICE

module helloworld {
    class Announcement {
        string message;
    };
};

#endif
\endverbatim


The rest of the tutorial is language specific so pick either 
\ref hello_world_cpp "Hello World in C++" or \ref hello_world_java "Hello World in Java".

\page hello_world_cpp Hello World C++

Now we've defined the data we're going to be sharing, let's start with
our first C++ component, <code>HelloWriter</code>, which will write an
<code>Announcement</code> object to working memory. The first step is
to create the header and source files for the component in our c++
source directory. Create the files <code>HelloWriter.hpp</code> and <code>HelloWriter.cpp</code>
under <code>src/c++</code>. Next, edit <code>HelloWriter.hpp</code> to contain:


\code 
#ifndef HELLO_WRITER_HPP_
#define HELLO_WRITER_HPP_

#include <cast/architecture.hpp>

class HelloWriter :
  public cast::ManagedComponent {

};

#endif
\endcode

This creates a component as a subclass of cast::ManagedComponent (the most
commonly used CAST component that can read and write to WM) but does
nothing else. To complement this, edit <code>HelloWriter.cpp</code> to
contain:

\code 
#include "HelloWriter.hpp"

extern "C" {
  cast::CASTComponentPtr
  newComponent() {
    return new HelloWriter();
  }
}
\endcode


This file first includes the header, then defines a function which
will create a new instance of our component (returning the result via
an \link cast::CASTComponentPtr Ice smart pointer\endlink). This
function is very important as it's the only way that the CAST
component framework knows how to create an instance of this component.

Before we go any further, let's compile this component to check that
everything's working so far. To do this we will use <a
href="http://www.cmake.org">CMake</a> (this is overkill for a single
component, but will be useful as we add additional c++ components). To
do this means adding the component into the cmake build process. The
easiest way to do this is to copy the CAST template files as
follows. First we need to make our subarchitecture a new cmake
project. 

To do this, first copy the "top level" cmake file
TopLevelTemplateCMakeLists.txt from CAST's template directory (the
install prefix + share/cast/templates,
e.g. <code>/usr/local/share/cast/templates</code>)into $TUTORIAL_ROOT
and rename to CMakeLists.txt. E.g.

\code 
cp /usr/local/share/cast/templates/TopLevelTemplateCMakeLists.txt $TUTORIAL_ROOT/CMakeLists.txt
\endcode

Next, edit this new file, replacing <code>MYPROJECT</code> with the
name of your project (e.g. <code>CASTTutorial</code>). Then include
our new subarchitecture in this project. To do this, edit
<code>CMakeLists.txt</code> again to add the following line at the
end:

\code 
add_subdirectory (subarchitectures/hello-world)
\endcode


Next we must tell CMake that some content exists in the
<code>hello-world</code> directory. To do this copy
<code>ProjectTemplateCMakeLists.txt</code> from CAST's template directory into
hello-world and rename it to <code>CMakeLists.txt</code>. E.g.

\code 
cp /usr/local/share/cast/templates/ProjectTemplateCMakeLists.txt $TUTORIAL_ROOT/subarchitectures/hello-world/CMakeLists.txt
\endcode

Then edit the new CMakeLists.txt file and replace <code>MYPROJECTNAME</code> with
<code>HelloWorld</code>. Next we need to include our component in this
new project. To do this first edit our new CMakeLists.txt to include
the line <code>add_subdirectory(src/c++)</code>. Next copy
<code>ComponentTemplateCMakeLists.txt</code> from from CAST's template
directory into our <code>c++</code> directory and rename to
<code>CMakeLists.txt</code>. E.g.

\code 
cp /usr/local/share/cast/templates/ComponentTemplateCMakeLists.txt $TUTORIAL_ROOT/subarchitectures/hello-world/src/c++/CMakeLists.txt
\endcode

Then edit this new file and replace <code>MYCOMPONENTNAME</code> with
<code>HelloWriter</code>. This latter file tells cmake to create a new
shared library called <code>libHelloWriter.so</code> which CAST will
use to create an instance of our new component when we run the system.

Finally we have to run CMake to create Makefiles, then actually
compile the project. For our own sanity we create our Makefiles and
build in a separate directory called <code>BUILD</code>. Create this
and change into it:

\code
mkdir $TUTORIAL_ROOT/BUILD
cd $TUTORIAL_ROOT/BUILD
\endcode

Next run the <code>ccmake</code> executable on the directory containing the toplevel CMakeLists.txt file. E.g.

\code
cd $TUTORIAL_ROOT/BUILD
ccmake ..
\endcode


This command will produce a GUI window with a number of options. The
default values should be fine. To create the build files press 'c' (to
configure the build files) until you are able to press 'g' (to
generate the build files). If you encounter errors, follow the
instructions in the error messages to fix them.

All that remains now is to build the project. Do this by making the
<code>install</code> target in the <code>BUILD</code> directory. E.g.

\code
cd $TUTORIAL_ROOT
make -C BUILD  install
\endcode

If this has been successful you should see a new file called
<code>$TUTORIAL_ROOT/output/lib/libHelloWriter.so</code>.

To complete our sanity check we should run our new component and see
what happens. As the component doesn't do anything nothing should
happen, but a successful nothing is better than an unsuccessful
something! CAST systems are run using \ref man_running "clients and servers", 
and are \ref man_configuration "configured using a file called a CAST file" which describes which components to run on which
machines. Let's create a basic CAST file called
<code>subarchitectures/hello-world/config/helloworld.cast</code> and enter the following lines:

\code 
HOST localhost

SUBARCHITECTURE hello-world
CPP WM SubarchitectureWorkingMemory
CPP TM AlwaysPositiveTaskManager
CPP MG writer HelloWriter
\endcode

The first line defines the default host one which the components
should be run. For this you can try the name of your machine or
"localhost". After that we define a subarchitecture. The first two
lines in this construct define the component classes used for the
subarchitecture working memory (WM) and task manager (TM). The example
above uses built-in components for these roles. The final line in the
subarchitecture definition introduces our new component. The first
part of the line tells CAST what language the component is written
in. The second part tells CAST that it's a managed component (i.e. the
class we inherited for our component). The third part defines a unique
ID we can use to refer to our component. And the final part is the
library that CAST will load to instantiate our component
(i.e. libHelloWriter.so from which it uses the newComponent
function). For more information see \ref man_configuration "the configuration instructions". Now we've defined our system we must can run it. Open two
terminals and change directory to $TUTORIAL_ROOT in each of them. 

Before you run the server, you must make sure that your library path
contains the directory where <code>libHelloWriter.so</code> is located
(so your component can be instantiated at run-time). The simplest way to do this is to set the environment variable
<code>LD_LIBRARY_PATH</code> (or <code>DYLD_LIBRARY_PATH</code> on OS
X) to include the directory which contains your library. E.g. (on Linux in bash)

\verbatim
export LD_LIBRARY_PATH=output/lib:$LD_LIBRARY_PATH
\endverbatim


Now we can start the server. In the
first terminal (where you've set your library path variable), run the <code>cast-server</code>
executable. This should give you some output like the following:

\code 
vonnegut:tutorial nah$ cast-server
Java server: 1530
CPP server: 1531
\endcode

The numbers tell you the process numbers of the underlying CAST
servers. Finally run the <code>cast-client</code> executable, passing
our new CAST file as the first argument:

\code 
vonnegut:tutorial nah$ cast-client subarchitectures/hello-world/config/helloworld.cast
CoSy Architecture Schema Toolkit. Release: 0.2.0
\endcode

As our component doesn't do anything, you should see much else in terms of output. To stop the system running Ctrl-C the client script. This will send appropriate shutdown signals to all running components.

Having got this far we can be pretty confident that CAST is running fine and that our new component has been accepted into its bosom. Next we can extend our component to actually do something. For our tutorial we want HelloWriter to add an <code>Announcement</code> object to working memory for some other component to read. To do this, we must get the component to execute some code when it runs. Every component in CAST inherits a \link cast::CASTComponent::runComponent() runComponent()\endlink member function which is called when the component is run. To use this, extend your HelloWriter class header to include the following lines:

\code 
protected:
  virtual void runComponent();
\endcode

Then add the following to your HelloWriter.cpp file:

\code 
void HelloWriter::runComponent() {
  println("Look out world, here I come...");
}
\endcode

This code just prints out (using CAST's built in \link cast::CASTComponent::println print method\endlink) a
nice message so we know things are running. Recompile and run your
component to verify that this works.

Next we need to create an instance of <code>Announcement</code> to write
to working memory. We have already defined this object in Slice, but
we now need to use this in C++. To do this we must use Ice's slice2cpp program to generate C++ source code from our Slice definition file. Although you could do this by hand, CAST has a built-in cmake macro to do this for you (borrowed from the lovely folk at <a href="http://orca-robotics.sourceforge.net/"> Orca</a>). Open <code>SliceLibraryTemplateCMakeLists.txt</code> from CAST's template directory, and paste the contents at the top of your component's CMakeLists.txt file, replacing <code>MYSLICEFILE</code> with <code>HelloWorldData</code> as you go. E.g. we should change the file to:


\code 
include_directories(.)
include(Slice2Cpp)

cast_slice2cpp(GEN_CPP GEN_HPP HelloWorldData.ice)
add_library(HelloWorldData SHARED ${GEN_CPP})
install(TARGETS HelloWorldData LIBRARY DESTINATION lib ARCHIVE DESTINATION lib)

add_cast_component(HelloWriter HelloWriter.cpp)
\endcode 

This will create <code>HelloWorldData.hpp</code> and <code>.cpp</code> containing the C++ version of our slice file, and from this create a library called <code>libHelloWorldData.so</code> when these files are compiled. As we will use this compiled code in our component, we should also tell cmake to link our component with this library. To do this, add the following line at the end of the file:

\code 
link_cast_component(${CAST_COMPONENT_NAME} HelloWorldData)
\endcode

Where <code>${CAST_COMPONENT_NAME}</code> is a variable set by <code>add_cast_component</code> that contains the name of the last component added. Using this variable makes the cmake files a little easier to maintain as things get more complex later on. Once this is complete, reconfigure (by running <code>ccmake</code> as before) and rebuild your component (by running <code>make</code> as before). You should now see some output about "Generating source" and some extra compilations. 

With this step complete we can now finally create an instance of {{Announcement}} to write to working memory. First off, include the newly generated <code>HelloWorldData.hpp</code> in <code>HelloWriter.hpp</code>:

\code 
#include <HelloWorldData.hpp>
\endcode

Then, in <code>runComponent()</code> create an instance of <code>Announcement</code> with the message you want to send:

\code 
void HelloWriter::runComponent() {
  println("Look out world, here I come...");
  helloworld::AnnouncementPtr ann = new helloworld::Announcement("Hello World!");
}
\endcode

The most important things to note about the addition line is that the object is allocated on the heap (i.e. using <code>new</code>) rather than on the stack, and the result of the allocation is stored using a smart (reference-counted) pointer (the name of the class with the postfix <code>Ptr</code>). This is purely an Ice constraint, but it must be obeyed for all Slice <code>class</code> types, and therefore for all instances of classes that are to be written to working memory in CAST. For more information on why this is the case read <a href="http://www.zeroc.com/faq/stackVsHeapAllocation.html">this</a>.

The only remaining task for this component is to write the newly created object to working memory. All working memory entries are created with an ID string which should ideally be generated using \link cast::WorkingMemoryWriterComponent::newDataID() newDataID()\endlink. The ID and the object are then sent to working memory via a call to \link cast::WorkingMemoryWriterComponent::addToWorkingMemory addToWorkingMemory\endlink as demonstrated in the final line of runComponent:

\code 
void HelloWriter::runComponent() {
  println("Look out world, here I come...");
  helloworld::AnnouncementPtr ann = new helloworld::Announcement("Hello World!");
  addToWorkingMemory(newDataID(), ann);
}
\endcode


Now we have our writer component we need <code>HelloReader</code> to read the announcement that has been written to WM. Create the component by copying <code>HelloWriter</code>, but remove the runComponent() method (as we don't need it for this component). Next, edit the
<code>CMakeLists.txt</code> file to add a new component called
<code>HelloReader</code>, i.e. add

\code 
add_cast_component(HelloReader HelloReader.cpp)
link_cast_component(${CAST_COMPONENT_NAME} HelloWorldData)
\endcode

Compile the code to check that this new component is correct. 

When data on working memory is changed (e.g. when a new <code>Annoucement</code> object is written to it) the working memory generates a new \link cast::cdl::WorkingMemoryChange Working Memory Change\endlink event which is sent to all components that listening for such events. So, the first thing our <code>HelloReader</code> must do is register with the working memory to listen for the events it cares about. When a component is started up, before \link cast::CASTComponent::runComponent() runComponent()\endlink is called (but after the constructor and \link cast::CASTComponent::configure(const std::map<std::string,std::string>&) configure\endlink), each component's \link cast::CASTComponent::start() start\endlink member function is called. It is here we will tell the WM what change events we're interested in. We do it here because it guarantees that we'll see any changes that are created by other components' runComponent methods, because start is always called before runComponent (for more information on methods are called when, see \ref man_system_architecture). 

To do this, we first add the start method to <code>HelloReader.hpp</code> and <code>.cpp</code>:

\code 
protected:
virtual void start();
\endcode


\code 
void HelloReader::start() {

}
\endcode

Next we need to tell working memory what we're interested in. We do this by passing CAST a \link cast::cdl::WorkingMemoryChangeFilter WorkingMemoryChangeFilter\endlink which describes what WM changes the component is interested in, and a \link cast::WorkingMemoryChangeReceiver WorkingMemoryChangeReceiver\endlink which is a callback object that is called when events matching the filter occur. Although you can create change filters by hand, the file \link ChangeFilterFactory.hpp ChangeFilterFactory\endlink (included via the previously inclusion of architecture.hpp) provides helper functions to create most commonly required filter types. For our <code>HelloReader</code> component add the following line to the start function:

\code 
addChangeFilter(cast::createLocalTypeFilter<helloworld::Announcement>(cast::cdl::ADD),
                                                                      new cast::MemberFunctionChangeReceiver<HelloReader>(this,
		                                                                                                          &HelloReader::makeAnnouncement));
\endcode


This does the following things. <code>cast::createLocalTypeFilter<helloworld::Announcement>(cast::cdl::ADD) </code> creates a working memory change filter that listens for additions to working memory (<code>cast::cdl::ADD</code>) of the <code>Announcement</code> class (the template argument) in the working memory of this component's subarchitecture (<code>cast::createLocalTypeFilter</code>). If you used cast::createGlobalTypeFilter you could listen for matching events on any working memory in the whole system. The second argument to \link cast::WorkingMemoryReaderComponent::addChangeFilter addChangeFilter\endlink creates a new change receiver object that automatically calls a member function in a class. In this case we've told it to call the member function <code>makeAnnouncement</code> which we've yet to define. So, next we'll define this. In order to be called when a change occurs, the function must have a particular signature. It must return <code>void</code> and must accept and a single const <code>WorkingMemoryChange</code> reference, i.e. (leaving the header as an for the reader):

\code 
void HelloReader::makeAnnouncement(const cast::cdl::WorkingMemoryChange & _wmc) {

}
\endcode

The input argument, \link cast::cdl::WorkingMemoryChange WorkingMemoryChange\endlink contains a number of fields which describe the event that has occurred (these are matched to the filter to determine whether the change event should be received or not). We can use one of these, the <code>address</code> field, to access the changed data on working memory. The address is of type \link cast::cdl::WorkingMemoryAddress WorkingMemoryAddress\endlink which contains an \link cast::cdl::WorkingMemoryAddress::id id\endlink field and a \link cast::cdl::WorkingMemoryAddress::subarchitecture subarchitecture\endlink field. In this case <code>id</code> is the string generated by <code>HelloWriter</code> using \link cast::WorkingMemoryWriterComponent::newDataID() newDataID()\endlink. The <code>subarchitecture</code> field will contain the name of the current component's subarchitecture. This is "hello-world" as described above in the CAST file. With the address from the change event we can read the data from working memory. There are many ways to do this, but the easiest is to use \link cast::WorkingMemoryReaderComponent::getMemoryEntry( const cdl::WorkingMemoryAddress & ) getMemoryEntry\endlink. This accepts an address and returns an Ice smart pointer to the requested object. To use this, add the following to <code>makeAnnouncement</code>:

\code 
  helloworld::AnnouncementPtr ann = getMemoryEntry<helloworld::Announcement>(_wmc.address);
\endcode

And finally we can print out our announcement by adding:

		     
\verbatim
  println("I'd like to make an announcement: %s", ann->message.c_str());    
\endverbatim


At this point we have completed our code, but need to add our new
component into our system to test it. To do this, add the following
extra line to the end of the <code>helloworld.cast</code> CAST file:

\verbatim
CPP MG reader HelloReader
\endverbatim

With this in place you should be able to re-run the system and see (in the %server terminal):

\verbatim
CoSy Architecture Schema Toolkit. Release: 0.2.0
["writer": Look out world, here I come...]
["reader": I'd like to make an announcement: Hello World!]
\endverbatim

If you see this, congratulations, you've written your first CAST system :)

\page hello_world_java Hello World Java

Now we've defined the data we're going to be sharing, let's start with
our first Java component, <code>HelloWriter</code>, which will write an
<code>Announcement</code> object to working memory. The first step is
to create the source file for the component in our java
source directory. Create the file <code>HelloWriter.java</code> 
under <code>subarchitectures/hello-world/src/java/helloworld</code>. Next, edit <code>HelloWriter.java</code> to contain:


\code
package helloworld;

import cast.architecture.ManagedComponent;

public class HelloWriter extends ManagedComponent {

}
\endcode

This creates a component as a subclass of
cast.architecture.ManagedComponent (the most commonly used CAST
component that can read from and write to WM) but does nothing else.

Before we go any further, let's compile this component to check that everything's working so far. To do this we will use <a
href="http://ant.apache.org">Apache Ant</a> compile everything (this
is overkill for a single file, but useful as we add more
components). 


To do this, first copy the "top level" ant build file
build-toplevel.xml from CAST's template directory (the
install prefix + share/cast/templates,
e.g. <code>/usr/local/share/cast/templates</code>)into $TUTORIAL_ROOT
and rename to build.xml. E.g.

\code 
cp /usr/local/share/cast/templates/build-toplevel.xml $TUTORIAL_ROOT/build.xml
\endcode

Next, edit this file to include our new project. To do this open the newly created <code>$TUTORIAL_ROOT/build.xml</code> and add the following line to the end of "template" target (around line 24):

\verbatim
  <ant target="${target}" dir="subarchitectures/hello-world" />
\endverbatim


Finally we must add a subarchitecture-specific build file for our tutorual subarchitecture.
The easiest way to do this is to copy the CAST ant
subarchitecture template file from the CAST template directory as follows:

\verbatim
cp /usr/local/share/cast/templates/build-template.xml $TUTORIAL_ROOT/subarchitectures/hello-world/build.xml
\endverbatim


All that remains now is to build the project. Do this by running <code>ant</code>:

\verbatim
cd $TUTORIAL_ROOT
ant all
\endverbatim

If this has been successful you should see a new file called <code>output/classes/helloworld/HelloWriter.class</code>. If you see an error similar to "taskdef class Slice2JavaTask cannot be found" make sure your CLASSPATH is setup as described in \ref man_installation "the installation guide" (both the prerequisites section and step 7), i.e. make sure that the Ice and cast jar files are included in your CLASSPATH.

To complete our sanity check we should run our new component and see
what happens. As the component doesn't do anything nothing should
happen, but a successful nothing is better than an unsuccessful
something! CAST systems are run using \ref man_running "clients and servers", 
and are \ref man_configuration "configured using a file called a CAST file" which describes which components to run on which
machines. Let's create a basic CAST file called
<code>subarchitectures/hello-world/config/helloworld.cast</code> and enter the following lines:

\code 
HOST localhost

SUBARCHITECTURE hello-world
JAVA WM cast.architecture.SubarchitectureWorkingMemory
JAVA TM cast.architecture.AlwaysPositiveTaskManager
JAVA MG writer helloworld.HelloWriter
\endcode

The first line defines the default host one which the components
should be run. For this you can try the name of your machine or
"localhost". After that we define a subarchitecture. The first two
lines in this construct define the component classes used for the
subarchitecture working memory (WM) and task manager (TM). The example
above uses built-in components for these roles. The final line in the
subarchitecture definition introduces our new component. The first
part of the line tells CAST what language the component is written
in. The second part tells CAST that it's a managed component (i.e. the class we inherited for our
component). The third part defines a unique ID we can use to refer to
our component. And the final part is the class that CAST will load to
instantiate our component. For more information see \ref man_configuration "the configuration instructions". 

Now we've defined our system we must can run it. At this point you should also make sure that <code>output/classes</code> is included in your CLASSPATH, so that the CAST server can find the results of our compilation. Open two terminals and change directory to $TUTORIAL_ROOT in each of them. In the first terminal, run the <code>cast-server</code> executable. This should give you some output like the following:


\code 
vonnegut:tutorial nah$ cast-server
Java server: 1530
CPP server: 1531
\endcode

The numbers tell you the process numbers of the underlying CAST
servers. Finally run the <code>cast-client</code> executable, passing
our new CAST file as the first argument:

\code 
vonnegut:tutorial nah$ cast-client subarchitectures/hello-world/config/helloworld.cast
CoSy Architecture Schema Toolkit. Release: 0.2.0
\endcode

As our component doesn't do anything, you should see much else in terms of output. To stop the system running Ctrl-C the client script. This will send appropriate shutdown signals to all running components.

Having got this far we can be pretty confident that CAST is running fine and that our new component has been accepted into its bosom. Next we can extend our component to actually do something. For our tutorial we want <code>HelloWriter</code> to add an <code>Announcement</code> object to working memory for some other component to read. To do this, we must get the component to execute some code when it runs. Every component in CAST inherits a \link cast::core::CASTComponent::runComponent() runComponent()\endlink member function which is called when the component is run. To use this, extend your <code>HelloWriter</code> class to include the following lines:

\code
@Override
protected void runComponent() {
  println("Look out world, here I come...");
}
\endcode

This code just prints out (using CAST's built in \link cast::core::CASTComponent::println print method\endlink) a
nice message so we know things are running. Recompile and run your
component to verify that this works.

Next we need to create an instance of <code>Announcement</code> to write to working memory. We have already defined this object in Slice, but we now need to use this in Java. To do this we must use Ice's <code>slice2java</code> program to generate Java source code from our Slice definition file. Although you could do this by hand, Ice provides an ant task to do this for you . Open <code>slice2java-template.xml</code> from CAST's cmake directory, and paste the contents into your subarchitecture's build.xml file's slice task, replacing <code>MYSLICEFILE</code> with <code>HelloWorldData</code> as you go. E.g. we should change <code>$TUTORIAL_ROOT/subarchitectures/hello-world/build.xml</code> to contain:

\verbatim
<target name="slice" depends="prepare" description="generates source from slice">
  <slice2java tie="true" outputdir="${src.dir}">
    <fileset dir="${slice.dir}" includes="HelloWorldData.ice"/>
      <includepath>
        <pathelement path="${slice.dir}"/>
      </includepath>
  </slice2java>
</target>
\endverbatim

This will create Java versions of the classes defined in our slice file. When you rebuild the project you should now see some output about "slice2java" and some extra compilations. 

With this step complete we can now finally create an instance of <code>Announcement</code> to write to working memory. First off, include the newly generated class file:


\code
import helloworld.Announcement;
\endcode

Then, in <code>runComponent()</code> create an instance of <code>Announcement</code> with the message you want to send:

\code
@Override
protected void runComponent() {
  Announcement ann = new Announcement("Hello World!");
}
\endcode

The only remaining task for this component is to write the newly created object to working memory. All working memory entries are created with an ID string which should ideally be generated using \link cast::architecture::WorkingMemoryWriterComponent::newDataID() newDataID()\endlink. The ID and the object are then sent to working memory via a call to \link cast::architecture::WorkingMemoryWriterComponent::addToWorkingMemory addToWorkingMemory\endlink as demonstrated in the final line of runComponent (you'll also need an additional import):

\code
import cast.AlreadyExistsOnWMException;
\endcode

\code
protected void runComponent() {
  Announcement ann = new Announcement("Hello World!");
  try {
    addToWorkingMemory(newDataID(), ann);
  } catch (AlreadyExistsOnWMException e) {
    e.printStackTrace();
  }
}
\endcode

Now we have our writer component we need <code>HelloReader</code> to read the announcement that has been written to WM. Create the component by copying <code>HelloWriter</code>, but remove the runComponent() method (as we don't need it for this component). Compile the code to check that this new component is correct.

When data on working memory is changed (e.g. when a new <code>Annoucement</code> object is written to it) the working memory generates a new \link cast::cdl::WorkingMemoryChange Working Memory Change\endlink event which is sent to all components that listening for such events. So, the first thing our <code>HelloReader</code> must do is register with the working memory to listen for the events it cares about. When a component is started up, before \link cast::core::CASTComponent::runComponent() runComponent()\endlink is called (but after the constructor and \link cast::core::CASTComponent::configure(const std::map<std::string,std::string>&) configure\endlink), each component's \link cast::core::CASTComponent::start() start\endlink member function is called. It is here we will tell the WM what change events we're interested in. We do it here because it guarantees that we'll see any changes that are created by other components' runComponent methods, because start is always called before runComponent (for more information on methods are called when, see \ref man_system_architecture). 

To do this, we first add the start method to <code>HelloReader.java</code>:

\code 
@Override
protected void start() {
}
\endcode

Next we need to tell working memory what we're interested in. We do this by passing CAST a \link cast::cdl::WorkingMemoryChangeFilter WorkingMemoryChangeFilter\endlink which describes what WM changes the component is interested in, and a \link cast::architecture::WorkingMemoryChangeReceiver WorkingMemoryChangeReceiver\endlink which is a callback object that is called when events matching the filter occur. Although you can create change filters by hand, the class \link cast::architecture::ChangeFilterFactory ChangeFilterFactory\endlink provides helper functions to create most commonly required filter types. For our <code>HelloReader</code> component, start by importing  the classes:

\code
//Factory functions for change filters
import cast.architecture.ChangeFilterFactory;
//Classes for receiving and managing changes
import cast.architecture.WorkingMemoryChangeReceiver;
import cast.cdl.WorkingMemoryChange;
import cast.cdl.WorkingMemoryOperation;
\endcode


... then add the following line to the start function:

\code
  addChangeFilter(ChangeFilterFactory.createLocalTypeFilter(Announcement.class, WorkingMemoryOperation.ADD), 
                                                            new WorkingMemoryChangeReceiver() {
                                                              public void workingMemoryChanged(WorkingMemoryChange _wmc) {
                                                                makeAnnouncement(_wmc);
                                                              }
                                                            });
\endcode


This does the following things. <code>ChangeFilterFactory.createLocalTypeFilter(Announcement.class, WorkingMemoryOperation.ADD)</code> creates a working memory change filter that listens for additions to working memory (<code>WorkingMemoryOperation.ADD</code>) of the <code>Announcement</code> class (the first argument) in the working memory of this component's subarchitecture (<code>ChangeFilterFactory.createLocalTypeFilter</code>). If you used ChangeFilterFactory.createGlobalTypeFilter you could listen for matching events on any working memory in the whole system. The second argument to \link cast::architecture::WorkingMemoryReaderComponent::addChangeFilter addChangeFilter\endlink creates a new change receiver object that calls a member function in a class. In this case we've told it to call the member function <code>makeAnnouncement</code> which we've yet to define. So, next we'll define this. In order to be called when a change occurs, the function must have a particular signature. It must return <code>void</code> and must accept and a single <code>WorkingMemoryChange</code> object, i.e.:

\code 
private void makeAnnouncement(WorkingMemoryChange _wmc) {
}
\endcode

The input argument, \link cast::cdl::WorkingMemoryChange
WorkingMemoryChange\endlink contains a number of fields which describe
the event that has occurred (these are matched to the filter to
determine whether the change event should be received or not). We can
use one of these, the <code>address</code> field, to access the
changed data on working memory. The address is of type \link
cast::cdl::WorkingMemoryAddress WorkingMemoryAddress\endlink which
contains an \link cast::cdl::WorkingMemoryAddress::id id\endlink field
and a \link cast::cdl::WorkingMemoryAddress::subarchitecture
subarchitecture\endlink field. In this case <code>id</code> is the
string generated by <code>HelloWriter</code> using \link
cast::architecture::WorkingMemoryWriterComponent::newDataID()
newDataID()\endlink. The <code>subarchitecture</code> field will
contain the name of the current component's subarchitecture. This is "hello-world" as described above in the CAST file. With the address from the change event we can read the data from working memory. There are many ways to do this, but the easiest is to use \link cast::architecture::WorkingMemoryReaderComponent::getMemoryEntry( WorkingMemoryAddress, Class< T > ) getMemoryEntry\endlink. This accepts an address and returns an Ice smart pointer to the requested object. To use this, first add the following imports:

\code
//For exceptions thrown by getMemoryEntry
import cast.UnknownSubarchitectureException;
import cast.DoesNotExistOnWMException;
\endcode

Then add the following to <code>makeAnnouncement</code>:

\code
  try {
    Announcement ann = getMemoryEntry(_wmc.address, Announcement.class);
  } catch (DoesNotExistOnWMException e) {
    e.printStackTrace();
  } catch (UnknownSubarchitectureException e) {
    e.printStackTrace();
  }
\endcode

And finally we can print out our announcement in <code>makeAnnouncement</code> by adding:

\code
  println("I'd like to make an announcement: " + ann.message);
\endcode

At this point we have completed our code, but need to add our new
component into our system to test it. To do this, add the following
extra line to the end of the <code>helloworld.cast</code> CAST file:

\verbatim
JAVA MG reader helloworld.HelloReader
\endverbatim

With this in place you should be able to re-run the system and see (in the %server terminal):

\verbatim
CoSy Architecture Schema Toolkit. Release: 0.2.0
["writer": Look out world, here I come...]
["reader": I'd like to make an announcement: Hello World!]
\endverbatim

If you see this, congratulations, you've written your first CAST system :)


* 
*/
}
