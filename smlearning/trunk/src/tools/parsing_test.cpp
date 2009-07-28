#include <tools/data_handling.h>

using namespace smlearning;

int main(int argc, char * argv[]) {
	//init the xerces lib
	try 
	{
		XMLPlatformUtils::Initialize();
	}
	catch (const XMLException& toCatch) 
	{
		char* message = XMLString::transcode(toCatch.getMessage());
		cerr << "Error during xerces initialization! :" << endl << message << endl;
		XMLString::release(&message);
	}

	OfflineRNN myRNN;
	XercesDOMParser parser;
	if (myRNN.parse_netfile (parser, "/usr/local/bin/SMLearning/defaultnet.xml", false)) {
		myRNN.load_net (1, "train");
	}
	else {
		cerr << "XML net file parsing not successful..." << endl;
		return 1;
	}

	myRNN.print_net_data ();

	return 0;
}
