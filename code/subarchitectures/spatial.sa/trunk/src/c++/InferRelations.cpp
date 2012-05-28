#include <fstream>
#include <map>
#include <dai/alldai.h>  // Include main libDAI header file
#include <dai/jtree.h>  // Include main libDAI header file
#include <dai/bp.h>  // Include main libDAI header file
#include <dai/decmap.h>  // Include main libDAI header file
#include "RelationEvaluation.hpp"

using namespace dai;
using namespace std;

namespace spatial {
bool inferRelationsThreeObjects(vector<double> &ret, double BOnA, double AOnB,
    double BOnT, double AOnT, double BInA, double AInB, double BInT,
    double AInT) {
  ofstream out("tmp.fg");
  // Write the variable part
  out << "32\n";
  out << "\n";
  out << "# Perceived B on A\n";
  out << "1\n";
  out << "11\n";
  out << "2\n";
  out << "2\n";
  out << "0 " << 1 - BOnA << "\n";
  out << "1 " << BOnA << "\n";
  out << "\n";
  out << "# Perceived A on B\n";
  out << "1\n";
  out << "8\n";
  out << "2\n";
  out << "2\n";
  out << "0 " << 1 - AOnB << "\n";
  out << "1 " << AOnB << "\n";
  out << "\n";
  out << "# Perceived B on T\n";
  out << "1\n";
  out << "5\n";
  out << "2\n";
  out << "2\n";
  out << "0 " << 1 - BOnT << "\n";
  out << "1 " << BOnT << "\n";
  out << "\n";
  out << "# Perceived A on T\n";
  out << "1\n";
  out << "2\n";
  out << "2\n";
  out << "2\n";
  out << "0 " << 1 - AOnT << "\n";
  out << "1 " << AOnT << "\n";
  out << "\n";
  out << "# Perceived B in A\n";
  out << "1\n";
  out << "9\n";
  out << "2\n";
  out << "2\n";
  out << "0 " << 1 - BInA << "\n";
  out << "1 " << BInA << "\n";
  out << "\n";
  out << "# Perceived A in B\n";
  out << "1\n";
  out << "6\n";
  out << "2\n";
  out << "2\n";
  out << "0 " << 1 - AInB << "\n";
  out << "1 " << AInB << "\n";
  out << "\n";
  out << "# Perceived B in T\n";
  out << "1\n";
  out << "3\n";
  out << "2\n";
  out << "2\n";
  out << "0 " << 1 - BInT << "\n";
  out << "1 " << BInT << "\n";
  out << "\n";
  out << "# Perceived A in T\n";
  out << "1\n";
  out << "0\n";
  out << "2\n";
  out << "2\n";
  out << "0 " << 1 - AInT << "\n";
  out << "1 " << AInT << "\n";
  out << "\n";

  // Write the constant part
  ifstream in("partial-fg.txt");
  if (!in.good()) {
    return false;
  }
  char buf[512];
  in.getline(buf, 511);
  while (!in.eof()) {
    out << buf << "\n";
    in.getline(buf, 511);
  }
  in.close();
  out.close();

  // Read FactorGraph from the file specified by the first command line argument
  FactorGraph fg;
  fg.ReadFromFile("tmp.fg");

  // Set some constants
  size_t maxiter = 10000;
  Real tol = 1e-9;
  size_t verb = 1;

  // Store the constants in a PropertySet object
  PropertySet opts;
  opts.set("maxiter", maxiter); // Maximum number of iterations
  opts.set("tol", tol); // Tolerance for convergence
  opts.set("verbose", verb); // Verbosity (amount of output generated)

  // Construct a JTree (junction tree) object from the FactorGraph fg
  // using the parameters specified by opts and an additional property
  // that specifies the type of updates the JTree algorithm should perform
  JTree jt(fg, opts("updates", string("HUGIN")));
  // Initialize junction tree algorithm
  jt.init();
  // Run junction tree algorithm
  jt.run();

  // Construct another JTree (junction tree) object that is used to calculate
  // the joint configuration of variables that has maximum probability (MAP state)
  JTree jtmap(fg, opts("updates", string("HUGIN"))("inference", string(
      "MAXPROD")));
  // Initialize junction tree algorithm
  jtmap.init();
  // Run junction tree algorithm
  jtmap.run();
  // Calculate joint state of all variables that has maximum probability
  vector<size_t> jtmapstate = jtmap.findMaximum();

  // Construct a BP (belief propagation) object from the FactorGraph fg
  // using the parameters specified by opts and two additional properties,
  // specifying the type of updates the BP algorithm should perform and
  // whether they should be done in the real or in the logdomain
  BP bp(fg, opts("updates", string("SEQRND"))("logdomain", false));
  // Initialize belief propagation algorithm
  bp.init();
  // Run belief propagation algorithm
  bp.run();

  // Construct a BP (belief propagation) object from the FactorGraph fg
  // using the parameters specified by opts and two additional properties,
  // specifying the type of updates the BP algorithm should perform and
  // whether they should be done in the real or in the logdomain
  //
  // Note that inference is set to MAXPROD, which means that the object
  // will perform the max-product algorithm instead of the sum-product algorithm
  BP mp(fg, opts("updates", string("SEQRND"))("logdomain", false)("inference",
      string("MAXPROD"))("damping", string("0.1")));
  // Initialize max-product algorithm
  mp.init();
  // Run max-product algorithm
  mp.run();
  // Calculate joint state of all variables that has maximum probability
  // based on the max-product result
  vector<size_t> mpstate = mp.findMaximum();

  //  // Construct a decimation algorithm object from the FactorGraph fg
  //  // using the parameters specified by opts and three additional properties,
  //  // specifying that the decimation algorithm should use the max-product
  //  // algorithm and should completely reinitalize its state at every step
  //  DecMAP decmap(fg, opts("reinit",true)("ianame",string("BP"))("iaopts",string("[damping=0.1,inference=MAXPROD,logdomain=0,maxiter=1000,tol=1e-9,updates=SEQRND,verbose=1]")) );
  //  decmap.init();
  //  decmap.run();
  //  vector<size_t> decmapstate = decmap.findMaximum();

  // Report variable marginals for fg, calculated by the junction tree algorithm
  cout << "Exact variable marginals:" << endl;
  for (size_t i = 0; i < fg.nrVars(); i++) // iterate over all variables in fg
    cout << jt.belief(fg.var(i)) << endl; // display the "belief" of jt for that variable

  // Report variable marginals for fg, calculated by the belief propagation algorithm
  cout << "Approximate (loopy belief propagation) variable marginals:" << endl;
  for (size_t i = 0; i < fg.nrVars(); i++) {
    // iterate over all variables in fg
    cout << bp.belief(fg.var(i)) << endl; // display the belief of bp for that variable
    ret.push_back(bp.belief(fg.var(i))[1]);
  }

  // Report factor marginals for fg, calculated by the junction tree algorithm
  cout << "Exact factor marginals:" << endl;
  for (size_t I = 0; I < fg.nrFactors(); I++) // iterate over all factors in fg
    cout << jt.belief(fg.factor(I).vars()) << endl; // display the "belief" of jt for the variables in that factor

  // Report factor marginals for fg, calculated by the belief propagation algorithm
  cout << "Approximate (loopy belief propagation) factor marginals:" << endl;
  for (size_t I = 0; I < fg.nrFactors(); I++) // iterate over all factors in fg
    cout << bp.belief(fg.factor(I).vars()) << endl; // display the belief of bp for the variables in that factor

  // Report log partition sum (normalizing constant) of fg, calculated by the junction tree algorithm
  cout << "Exact log partition sum: " << jt.logZ() << endl;

  // Report log partition sum of fg, approximated by the belief propagation algorithm
  cout << "Approximate (loopy belief propagation) log partition sum: "
      << bp.logZ() << endl;

  // Report exact MAP variable marginals
  cout << "Exact MAP variable marginals:" << endl;
  for (size_t i = 0; i < fg.nrVars(); i++)
    cout << jtmap.belief(fg.var(i)) << endl;

  // Report max-product variable marginals
  cout << "Approximate (max-product) MAP variable marginals:" << endl;
  for (size_t i = 0; i < fg.nrVars(); i++)
    cout << mp.belief(fg.var(i)) << endl;

  // Report exact MAP factor marginals
  cout << "Exact MAP factor marginals:" << endl;
  for (size_t I = 0; I < fg.nrFactors(); I++)
    cout << jtmap.belief(fg.factor(I).vars()) << " == " << jtmap.beliefF(I)
        << endl;

  // Report max-product factor marginals
  cout << "Approximate (max-product) MAP factor marginals:" << endl;
  for (size_t I = 0; I < fg.nrFactors(); I++)
    cout << mp.belief(fg.factor(I).vars()) << " == " << mp.beliefF(I) << endl;

  // Report exact MAP joint state
  cout << "Exact MAP state (log score = " << fg.logScore(jtmapstate) << "):"
      << endl;
  for (size_t i = 0; i < jtmapstate.size(); i++)
    cout << fg.var(i) << ": " << jtmapstate[i] << endl;

  // Report max-product MAP joint state
  cout << "Approximate (max-product) MAP state (log score = " << fg.logScore(
      mpstate) << "):" << endl;
  for (size_t i = 0; i < mpstate.size(); i++)
    cout << fg.var(i) << ": " << mpstate[i] << endl;

  //  // Report DecMAP joint state
  //  cout << "Approximate DecMAP state (log score = " << fg.logScore( decmapstate ) << "):" << endl;
  //  for( size_t i = 0; i < decmapstate.size(); i++ )
  //    cout << fg.var(i) << ": " << decmapstate[i] << endl;
  return true;
}
}
;
