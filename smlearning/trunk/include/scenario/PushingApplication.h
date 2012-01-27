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
#include <scenario/Polyflap.h>

namespace po = boost::program_options;

namespace smlearning {

bool XMLData(Scenario::Desc&, XMLContext*, Context*);

/** Application */
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
	virtual int start_experiment(char *argv[]);

	/** options description */
	boost::program_options::options_description prgOptDesc;
};


}; /* namespace smlearning */

#endif
