s/**
   \page man_configuration Configuring A CAST System

\section config_intro Introduction

To run a CAST system you must provide the client with a description of
the components you wish to run and the way that they are divided
across subarchitectures. This configuration file is written in plain
text and called a <em>CAST file</em>. For the sake of consistency we
always give the config files a .cast suffix and store them in a
/config directory. In a CAST config file you can describe anything
from a single component, to a single subarchitecture (if you want to
test it on its own) or to a full architecture with any number of
subarchitectures. For each component you can specify the host it runs
on and a list of command-line arguments which as passed into the
component's configure method [\link
cast::CASTComponent::configure(const std::map<std::string,std::string>&) c++\endlink|\link cast::core::CASTComponent::configure(Map<String,String>) java\endlink].

\section config_format Formatting

\subsection config_comments Comments

Comment lines start with #.


\subsection config_host The HOST Line

If you want to specify a host machine to be used as the default for
all subsequent components, you can specify it using the keyword
<code>HOST</code>. E.g. The entry...

\verbatim
HOST dewey 
\endverbatim

... will cause the configuration code to assign all subsequent
components the hostname of dewey unless overridden.

   


\subsection config_subarch The SUBARCHITECTURE Section

The majority of a config file will probably be made up of
subarchitecture descriptions. A subarchitecture description is started
with the following line

\verbatim
SUBARCHITECTURE <subarchName> <opt:subarchHost>
\endverbatim

This states that the component descriptions following this line make
up the subarchitecture, and run on the optionally specified host. E.g.

\verbatim
SUBARCHITECTURE vision.sa louie
\endverbatim

Describes a subarchitecture <code>vision.sa</code> with its components
to run by default on <code>louie</code>. The lines following this
<em>must</em> describe a working memory component, and task manager
component and zero or more processing components.



\section config_wm The Working Memory Line

You describe a subarchitecture working memory with the following line...

\verbatim
<opt:host> <lang> WM <class> <opt:command line>
\endverbatim

Where <code><opt:host></code> is an optional name of the host to run
the component on, <code><lang></code> specifies the language the
working memory was written in and is either <code>CPP</code> or
<code>JAVA</code>, <code>WM</code> states that this component is a
working memory, <code><class></code> is the name of the component
library (for C++) or is a fully qualified class name (for Java), and
<code><opt:command line></code> is a list command parameters which are
passed to the component's configure method. These are described in
more detail \ref config_args "here". E.g.

\verbatim
JAVA WM cast.architecture.SubarchitectureWorkingMemory --debug -v 2 -N
\endverbatim

... specifies a java working memory component with some arguments, and 

\verbatim
hewey C++ WM SubarchitectureWorkingMemory --log
\endverbatim


... does the same for a C++ working memory (from
libSubarchtectureWorkingMemory.so), but also specifies that it runs on
a specific host.



\subsection config_tm The Task Manager Line

This is the same format as the working memory line, but with <code>TM</code>
instead of <code>WM</code>. E.g.

\verbatim
hewey JAVA TM cast.architecture.AlwaysPositiveTaskManager 
\endverbatim


\subsection config_um Unmanaged Component lines

Unmanaged components [\link cast::UnmanagedComponent c++\endlink|\link
cast::architecture::UnmanagedComponent java\endlink] are added to the
subarchitecture using a line of the following format...

\verbatim
<opt:host> <lang> UM <id> <class> <opt:command line>
\endverbatim

This is almost identical to preceding lines, with the addition of the
<code><id></code> argument which specifies the unique identifier of
the processing component (used to identify it at rutime). E.g.

\verbatim
CPP UM ernie.wise StraightMan --setup "what's brown and sticky"
\endverbatim


\subsection config_mg Managed Component lines

Managed components [\link cast::ManagedComponent c++\endlink|\link
cast::architecture::ManagedComponent java\endlink] are added in the same way as unmanaged components,
but with <code>MG</code> instead of <code>UM</code>. E.g.

\verbatim
JAVA MG eric.morecambe comedyarch.FunnyMan --punchline "a stick" --log #--debug
\endverbatim


\subsection CAST Components

You can also add components a CAST system which are not part of a
subarchitecture [\link cast::CASTComponent c++\endlink|\link
cast::core::CASTComponent java\endlink].

This is done in a similar way to other components, but with a
<code>COMPONENT</code> prefix...

\verbatim
COMPONENT <opt:host> <lang> <name> <class> <opt:command line>
\endverbatim

e.g.

\verbatim
COMPONENT CPP eg ExampleComponent --log  #--debug
\endverbatim

Would create an extra C++ component in the configuration.



\section config_args "Command Line" Configuration

The command line-style arguments given to any process are parsed into
string key-value pairs and are placed into the map given as input to
the component's configure method [\link cast::CASTComponent::configure(const zstd::map<std::string,std::string>&) c++\endlink|\link
cast::core::CASTComponent::configure(Map<String,String>)
java\endlink]. Any command line entry starting with <code>-</code> is
treated as a key and then the subsequent string is treated as its
associated value. Keys without attached values are given the value
"true". Explicitly marked strings are not broken up. E.g.

\verbatim
--debug -v 2 -N -say "Hello World"
\endverbatim

would result in the key-value pairs:

\verbatim
--debug = true
-v = 2 
-N  = true
-say = Hello World
\endverbatim

\section config_example Example Configuration

\verbatim
HOST localhost

SUBARCHITECTURE stage.subarch 
CPP WM SubarchitectureWorkingMemory  --log --debug
CPP TM AlwaysPositiveTaskManager #--log --debug
JAVA UM straight.man comedyarch.StraightMan -s "what's brown and sticky" --log --debug
CPP MG funny.man FunnyMan --punchline "a stick" --log --debug

SUBARCHITECTURE audience.subarch 
CPP WM SubarchitectureWorkingMemory  --log --debug
CPP TM AlwaysPositiveTaskManager #--log --debug
JAVA MG audience.member comedyarch.AudienceMember --reaction "YAY!" --log --debug


SUBARCHITECTURE director.subarch
#CPP WM SubarchitectureWorkingMemory  --log --debug
JAVA WM cast.architecture.SubarchitectureWorkingMemory  --log --debug
CPP TM AlwaysPositiveTaskManager #--log --debug
JAVA MG ass.director comedyarch.AssistantDirector --log #--debug
JAVA MG director comedyarch.Director --audience audience.subarch --log  #--debug
\endverbatim


*/
