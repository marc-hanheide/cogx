
#include "FormBestResults.hh"
#include "BestResult.hh"

namespace Z
{

static int CmpBestResults(const void *a, const void *b)
{
  if( BestResults(*(unsigned*)a)->sig > BestResults(*(unsigned*)b)->sig )
    return -1;  // a is first
  else
    return 1 ;  // b is first
}

void FormBestResults::Rank()
{
  RankGestalts(Gestalt::BEST_RESULT, CmpBestResults);
}

/**
**	Mask()
*/
void FormBestResults::Mask()
{
}

bool FormBestResults::NeedsOperate()
{ 
  return needsOperate;	
}

FormBestResults::FormBestResults(Config *cfg) : GestaltPrinciple(cfg)
{
  needsOperate = false;
}

/**
**	InformNewGestalt	
*/
void FormBestResults::InformNewGestalt(Gestalt::Type type, unsigned idx)
{
	Mask();
	Rank();	

  StopRunTime();
}

void FormBestResults::Operate(bool incremental)
{
	/// make new Best Results!!!
}

void FormBestResults::OperateNonIncremental()
{
	/// make new Best Results
// 	printf("make new best results\n");
	NewGestalt(new BestResult(0));	
}


}
