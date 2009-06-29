/**
   \page man_running Running CAST

This is just a quick overview of the CAST commands.  

\section server CAST Servers

A CAST system can run across multiple machines. To run CAST on one or
more machines, run one CAST server on each machine you wish to
use. This is done using the <code>cast-server</code> command. Each
machine should produce some output like the text below. The
<code>cast-server</code> executable actually runs severs for both the
languages supported by CAST. The numbers displayed by on the terminal
are the process ids of the Java and C++ servers.

\verbatim
vonnegut:~ nah$ cast-server
Java server: 1301
CPP server: 1302
\endverbatim


To enable the server to find your compiled components you must do the
following. For Java components your <code>CLASSPATH</code> environment
variable must include the directories and/or jars which contain the
classes. For C++ components your component libraries
(e.g. <code>libHelloWriter.so</code>) must be in the library search
path. The easiest way to do this is to set the environment variable
<code>LD_LIBRARY_PATH</code> (or <code>DYLD_LIBRARY_PATH</code> on OS
X) to include the directories which contain your libraries.

\section client CAST Client

Once all the servers are running, you can run a CAST system by running
a single client on one machine. This machine can be either one of the
machines running a server or another machine entirely. The client is
run using the <code>cast-client</code> with a single argument, \ref
man_configuration "a cast file". E.g.

\verbatim
vonnegut:~ nah$ cast-client /usr/local/share/cast/config/comedian-architecture.cast 
CoSy Architecture Schema Toolkit. Release: 0.2.0
[starting [1/11]: stage.subarch_wm]
[starting [2/11]: stage.subarch_tm]
[starting [3/11]: funny.man]
...
\endverbatim

The cast file contains a description of the system you wish to
run. For more details on this file see \ref man_configuration
"configuration instructions". When a CAST system runs it passes
through a series of synchronised steps. These are described in \ref
man_system_architecture "system architecture overview".
 
To shut down a running CAST system, CTRL-C the client. This will send
the appropriate signals to the servers, halting execution and
destroying the components. Once a system has been shut down, you can
run the client again without restarting the servers. 


*/
