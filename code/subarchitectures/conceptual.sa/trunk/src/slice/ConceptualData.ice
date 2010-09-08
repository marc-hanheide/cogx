#ifndef CONCEPTUALDATA_ICE
#define CONCEPTUALDATA_ICE

#include <cast/slice/CDL.ice>

/**
 * Data structures representing the knowledge stored in the conceptual layer
 * and interfaces used to access that knowledge.
 * @author Andrzej Pronobis
 */

module ConceptualData 
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
    };


   interface QdlQueryHandlerInterface
   {
	QueryResults querySelect(string q);
   }

};
#endif // CONCEPTUALDATA_ICE