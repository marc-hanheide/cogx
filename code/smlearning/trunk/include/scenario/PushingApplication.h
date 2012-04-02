/** @file PushingApplication.h
 * 
 *
 * @author	Sergio Roa (DFKI)
 *
 * @version 1.0
 *
 * Copyright 2009      Sergio Roa
 *
 * @author	Marek Kopicki (see copyright.txt),
 * 			<A HREF="http://www.cs.bham.ac.uk/~msk">The University Of Birmingham</A>
 * @author      Jan Hanzelka - DFKI
 * @author	Manuel Noll (DFKI)

   This is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This package is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License.
   If not, see <http://www.gnu.org/licenses/>.
 
 */

#ifndef PUSHINGAPPLICATION_H
#define PUSHINGAPPLICATION_H


#include <Golem/Phys/Application.h>
#include <boost/program_options.hpp>

#include <scenario/Scenario.h>
#include <scenario/PredictingScenario.h>
#include <scenario/Polyflap.h>

namespace po = boost::program_options;

namespace smlearning {

bool XMLData(Scenario::Desc&, XMLContext*, Context*);
bool XMLData(PredictingScenario::Desc&, XMLContext*, Context*);

/** Application */
template<typename S, typename D>
class PushingApplication : public golem::Application 
{
public:
	/** std cto */
	virtual void define_program_options_desc();
	/** Main function */			
	virtual int main(int argc, char *argv[]);
	/** reads the ovbject description and the options necessary for the program run	*/
	virtual int read_program_options(int argc, char *argv[]);

	/** options map */
	boost::program_options::variables_map vm;

	
protected:

	/** Runs Application */
	virtual void run(int argc, char *argv[]);
	int start_experiment(char *argv[]);

	/** options description */
	boost::program_options::options_description prgOptDesc;
	/** Scenario pointer (should be used with derived classes) */
	S *pScenario;
};

///////// Public //////////
template<typename S, typename D>
void PushingApplication<S,D>::define_program_options_desc() 
{
	try {

		if (S::getName() == "Scenario" || S::getName() == "PredictingScenario") {
			prgOptDesc.add_options()
				("help,h", "produce help message")
				("numSequences,S", po::value<string>(), "number of sequences")
				("startingPosition,P", po::value<string>(), "only starting position to use")
				// ("storeLabels,L", "store labels")
				("configFile,C", po::value<string>(), "name of the xml config file");
		}
		if (S::getName() == "PredictingScenario" ) {
			prgOptDesc.add_options ()
				("ssmFile,s", po::value<string>(), "name of SSM file")
				("seqFile,D", po::value<string>(), "name of file containing data sequences");
		}
		
		
		
	} catch(std::exception& e) {
		cerr << "error: " << e.what() << "\n";
	} catch(...) {
		cerr << "Exception of unknown type!\n";
		
	}
	
}

///////// Public //////////
/** \brief reads the object description and options necessary for the program run
*/
template<typename S, typename D>
int PushingApplication<S,D>::read_program_options(int argc, char *argv[]) {

	std::cout << "reading programm options..."<<std::endl;
	try {
		po::store(po::parse_command_line(argc, argv, prgOptDesc), vm);
		po::notify(vm);

		if (vm.count("help")) {
			
			cout << prgOptDesc << endl;
			return 1;
		}

		if (S::getName() == "PredictingScenario" ) {
			if (!vm.count("ssmFile")) {
				cout << "You should provide a .ssm file name" << endl;
				return 1;
			}
		}


	}catch(std::exception& e) {
		cerr << "error: " << e.what() << "\n";
	}catch(...) {
		cerr << "Exception of unknown type!\n";

	}

	std::cout << "reading programm options done"<<std::endl;
	return 0;
}

///////// Protected //////////
template<typename S, typename D>
int PushingApplication<S,D>::start_experiment(char *argv[]) {
	int problemOccured;
	if (vm.count("configFile")) {
		char* arr[2];
		//char* arr [] = (char *){argv[0], vm["configFile"].as<string>().c_str()};
		arr[0] = argv[0];
		arr[1] =  (char*)vm["configFile"].as<string>().c_str();
		problemOccured = Application::main(2, arr);

	} else {
		char* arr [] = {argv[0]};
		problemOccured = Application::main(1, arr);
	}
	if (problemOccured) {
		cout << endl << "Please use the option -C or --configFile for the configuration file path argument." << endl;
		cout << endl << prgOptDesc << endl;
		return 1;
	}
		
}

///////// Public //////////
/** \brief Main function 
*/
template<typename S, typename D>
int PushingApplication<S,D>::main(int argc, char *argv[]) {


	try {
		std::cout << "starting in main"<<std::endl;
		define_program_options_desc();

		if (read_program_options(argc, argv)) {
			std::cout << "return with 1, nothing done"<<std::endl;
			return 1;
		}

		std::cout << "start experiment"<<std::endl;
		start_experiment(argv);




	} catch(std::exception& e) {
		cerr << "error: " << e.what() << "\n";
	} catch(...) {
		cerr << "Exception of unknown type!\n";
	}

	return 1;
	
}

///////// Protected //////////
/** \brief Runs Application 
*/
template<typename S, typename D>
void PushingApplication<S,D>::run(int argc, char *argv[]) {


	D desc;
	// Description desc;
std::cout << "step 1"<<std::endl;
	XMLData(desc, xmlcontext(), context());

	pScenario = dynamic_cast<S*>(scene()->createObject(desc)); // throws
	cout << "Scenario type: " << S::getName() << endl;
	if (pScenario == NULL) {
		context()->getMessageStream()->write(Message::LEVEL_CRIT, "PushingApplication::run(): unable to cast to Scenario");
		return;
	}

	// Random number generator seed
	context()->getMessageStream()->write(Message::LEVEL_INFO, "Random number generator seed %d", context()->getRandSeed()._U32[0]);
	
	try {
		Polyflap* actor=new Polyflap;
		pScenario->setActorObject(actor);
		pScenario->init(vm);
		pScenario->run(argc, argv);
	}
	catch (const Scenario::Interrupted&) {
	}
	std::cout << "step 3"<<std::endl;
	scene()->releaseObject(*pScenario);
}



}; /* namespace smlearning */

#endif
