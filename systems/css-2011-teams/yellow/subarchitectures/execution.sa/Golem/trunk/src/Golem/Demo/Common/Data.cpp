/** @file Data.cpp
 * 
 * @author	Marek Kopicki (The University Of Birmingham)
 *
 * @version 1.0
 *
 */

#include <Golem/Demo/Common/Data.h>
#include <Golem/Tools/Data.h>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

void golem::XMLData(Retina::Desc &val, XMLContext* context, bool create) {
	ASSERT(context)

	XMLData("volume", val.volume, context, create);
	XMLData("clustering", val.clustering, context, create);
	XMLData("cluster_diam", val.clusterDiam, context, create);
	XMLData("diam_fac", val.receptiveFieldDiamFac, context, create);
	
	XMLData(val.retinaPose, context->getContextFirst("pose"), create);
	XMLData("n1", val.receptiveFieldNum.n1, context->getContextFirst("receptive_field size"), create);
	XMLData("n2", val.receptiveFieldNum.n2, context->getContextFirst("receptive_field size"), create);
	XMLData("n3", val.receptiveFieldNum.n3, context->getContextFirst("receptive_field size"), create);
	XMLData(val.receptiveFieldGrid, context->getContextFirst("receptive_field grid"), create);
}

//------------------------------------------------------------------------------
