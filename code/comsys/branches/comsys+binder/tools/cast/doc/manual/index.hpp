namespace cast {
/**
 * \mainpage CAST Manual
 *

 * \section index_overview Overview 
 * 
 * In essence CAST is a software framework for developing integrated
 * systems from a collection of heterogenous components. The feature
 * that distinguishes CAST from most other component-based frameworks
 * is that it is designed to support component communication via
 * shared <em>working memories</em> (WMs), rather than via
 * direct/client server communication (or any other software
 * architecture style). For simiplicity you can treat a working memory
 * as a hashtable that maps string ids to objects of various
 * types. Each component in a CAST system is connected directly to one
 * working memory, then all working memories are interconnected,
 * allowing each component to access any object in the system via it's
 * own WM. A collection of components connected to the same working
 * memory is called a <em>subarchitecture</em> (SA), and a full system
 * (or architecture) is made up of one or more subarchitectures. 

 * \section index_docs Documentation
 * 
 * This is the documentation for the CoSy Architecture Schema Toolkit. 
 *
 *  - \ref man_installation 
 *  - \ref man_running 
 *  - \ref man_configuration 
*  - \ref man_system_architecture

*  For code documentation (incompleteness guaranteed!), see the API.
 * \section index_sw_docs API
 *  - <a href="../../java/html/namespacecast.html">Java</a>
 *  - <a href="../../c++/html/namespacecast.html">C++</a>


 * \section index_tut Tutorial
 * 
 * - \ref hello_world_start 
 * 
*/
}
