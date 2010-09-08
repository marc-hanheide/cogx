#ifndef DEFAULTDATA_ICE
#define DEFAULTDATA_ICE

#include <cast/slice/CDL.ice>

/**
 * Data structures representing the default knowledge maintained by default.sa
 * and interfaces used to access that knowledge.
 * @author Andrzej Pronobis
 */

module DefaultData 
{

    sequence<string> BindingRow;
    sequence<BindingRow> BindingTable;
    dictionary<string, int> StringToIntMap;

    struct QueryResults 
    {
    	string query;
    	StringToIntMap varPosMap;
    	BindingTable bt;
    };

    interface HFCInterface 
    {
	QueryResults querySelect(string q);
	string ping();
    };

   interface QdlQueryHandlerInterface
   {
	QueryResults querySelect(string q);
   }

};
#endif // DEFAULTDATA_ICE