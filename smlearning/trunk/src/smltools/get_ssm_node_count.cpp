/** @file get_ssm_node_count.cpp
 *
 * Get CrySSMEx SSM state count from a set of SSMs in a directory containing regions
 *
 * @author      Sergio Roa - DFKI
 * @version 2.0 beta
 *
 */

#include <boost/program_options.hpp>
#include <metalearning/GNGSMRegion.h>

namespace po = boost::program_options;
namespace fs = boost::filesystem;

using namespace smlearning;

int main (int argc, char* argv[])
{
	string prefix;
	unsigned int total_state_count = 0;
	
	po::options_description desc("Allowed parameters:");
	desc.add_options()
		("help,h", "produce help message")
		("prefix,p", po::value(&prefix)->default_value("./"), "Prefix for directory where SSMs are located.");

	// Declare an options description instance which will include
	// all the options
	po::options_description all("Basic usage:");
	all.add(desc);
  
	po::variables_map vm;
	po::store(po::command_line_parser(argc, argv).
		  options(all).run(), vm);
	po::notify(vm);

	if (vm.count("help")) {
		cout << desc << "\n";
		return 0;
	}

	boost::regex regfile_re ("(.*_final)\\.reg");
	boost::cmatch matches;
	// cout << matches.size() << endl;
	fs::path p(prefix);
	if(!exists(p)) {
		cerr<<p.leaf()<<" does not exist." << endl;
		return 1;
	}

	fs::directory_iterator dir_iter (p), dir_end;
	for (;dir_iter != dir_end; ++dir_iter)
	{
		string dirstring (dir_iter->leaf().c_str());
		char *dirchar = (char *)dirstring.c_str();
		if (boost::regex_match ((const char*)dirchar, matches, regfile_re))
		{
			GNGSMRegion currentRegion;
			string regionFileName (matches[1].first, matches[1].second);
			cout << dir_iter->leaf() << endl;
			cout << regionFileName << endl;
			if (currentRegion.readData (regionFileName)) {
				cout << "region data correctly read..." << endl << "======" << endl;
			}
			else {
				cerr << "Error by readying region data..." << endl;
				return 1;
			}
			unsigned int state_count = currentRegion.cryssmex.getSSM ()->state_count ();
			cout << "Nr. of nodes: " << state_count << endl;
			total_state_count += state_count;
		}
	}
	cout << endl << "Total nr. of nodes: " << total_state_count << endl;
		
	return 0;

}
