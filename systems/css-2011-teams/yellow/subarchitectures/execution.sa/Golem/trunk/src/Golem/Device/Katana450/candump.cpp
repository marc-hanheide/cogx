// CanBus Dumper for Logbüchse
// Neuronics AG 
// 4. Februar 2008

#include <canbus_interface.h>
#include <iostream>
#include <fstream>

CanBusInterface * mCAN;

int main(int argc, char *argv[])
{
	mCAN = new CanBusInterface();
	
	mCAN->open();
	std::ofstream myfile;
	
	if ( argc > 1 )
		myfile.open (argv[1]);	
		
	mCAN->flush();
	
	while(1)
	{	
		CanBusMsg msg = mCAN->read();
		//int pendingReads = mCAN->getExtStatus().pendingReads;
		CanErrorFlags status = mCAN->status();
		if ( argc > 1 )
			myfile << mCAN->CanBusMsg2log(msg) <<"\t" << status  << std::endl;
		else
			std::cout << mCAN->CanBusMsg2log(msg) <<"\t" << status  << std::endl;
	}	
}
