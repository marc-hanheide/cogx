#include "AVSPolicyManager.hh"
#include "Utils/CureDebug.hh"

//g++ -L /home/cure/lib/cure/ -L ./ -lcuretoolbox -lpthread -lAVSPolicyManager Test.cc  -I /home/cure/toolbox/src/ 

int main(int argc, char * argv[])
{
  cure_debug_level = 90;

  const char *optstring = "hc:";
  std::string args = "[-h help] [-c config.cfg] ";
  std::string configfile = "-";
  char o = getopt(argc, argv, optstring);
  while (o != -1) {
    switch (o) {
      
      break;
    case 'c':
      configfile = optarg;
      break;
    case 'h':
    case '?':
      std::cerr << "Usage: " << argv[0] << " " << args << std::endl;
    std::cerr << "Example " << argv[0] << " "
	      <<"-c config.cfg"<<std::endl;
    return -1;
    }
    o = getopt(argc, argv, optstring);
  }
  AVSPolicyManager pm;
  Cure::LongArray pols;

  double q;
  double p=.9;
  double t=.01;
  pm.m_MaxSteps=10;
  std::cerr<<"Call preCompute/n";
  pm.preCompute("book",.05);
  //pm.preCompute("Needle",.05);
  std::list<std::string> policy; 
  std::string currentCfg="Unknown";
  std::cerr<<"\nTry nextGreedyRatioPolicy:\n";
  pols.grow(0);
  while( pm.nextGreedyRatioPolicy(policy,
				  currentCfg,
				  t,p)){
    pols.add(0,pm.m_CurrentPolicy);
    std::cerr<<"\ngot ";
    for (std::list<std::string>::iterator pi = policy.begin();
	 pi != policy.end(); pi++) {
      std::cerr<<(*pi)<<"->";
      currentCfg=(*pi);
  
   }
  }

  pm.resetToPrior();
  pm.m_CurrentRoom=0;
  double ec=pm.expectedCost(q,pols,p,t);
  std::cerr<<"\nExpectedCost="<<ec<<" "<<q<<"\n";
  pols.grow(0);
  getchar();

  pm.m_ExpectedCost=10;
  currentCfg="Unknown";
  std::cerr<<"\nTry nextGreedyCostPolicy:\n";
  while( pm.nextGreedyCostPolicy(policy,
				  currentCfg,

				  t,p)){
 
    std::cerr<<"\ngot ";
    pols.add(0,pm.m_CurrentPolicy);
    for (std::list<std::string>::iterator pi = policy.begin();
	 pi != policy.end(); pi++) {
      std::cerr
<<(*pi)<<"->";
      currentCfg=(*pi);
    }
  }
  pm.resetToPrior();
  pm.m_CurrentRoom=0;
  ec=pm.expectedCost(q,pols,p,t);
  std::cerr<<"\nExpectedCost="<<ec<<" "<<q<<"\n";
  pols.grow(0);
  getchar();
  pm.m_ExpectedCost=10;
  currentCfg="Unknown";
  std::cerr<<"\nTry next1StepPolicy:\n";
  while( pm.next1StepPolicy(policy,
			    currentCfg,
			    t,p)){
    std::cerr<<"\ngot ";
    pols.add(0,pm.m_CurrentPolicy);
    for (std::list<std::string>::iterator pi = policy.begin();
	 pi != policy.end(); pi++) {
      std::cerr<<(*pi)<<"->";
      currentCfg=(*pi);
    }
  }
  pm.resetToPrior();
  pm.m_CurrentRoom=0;
  ec=pm.expectedCost(q,pols,p,t);
  std::cerr<<"\nExpectedCost="<<ec<<" "<<q<<"\n";
  pols.grow(0);
  getchar();
  pm.m_ExpectedCost=10;
  currentCfg="Unknown";
  std::cerr<<"\nTry next2StepPolicy:\n";
  while( pm.next2StepPolicy(policy,
			    currentCfg,
			    t,p)){
    std::cerr<<"\ngot ";
    pols.add(0,pm.m_CurrentPolicy);
    for (std::list<std::string>::iterator pi = policy.begin();
	 pi != policy.end(); pi++) {
      std::cerr<<(*pi)<<"->";
      currentCfg=(*pi);
    }
  }
  pm.resetToPrior();
  pm.m_CurrentRoom=0;
  ec=pm.expectedCost(q,pols,p,t);
  std::cerr<<"\nExpectedCost="<<ec<<" "<<q<<"\n";
  pols.grow(0);
  getchar();
  pm.m_ExpectedCost=10;
  currentCfg="Unknown";
  std::cerr<<"\nTry next3StepPolicy:\n";
  while( pm.next3StepPolicy(policy,
			    currentCfg,
			    t,p)){
    std::cerr<<"\ngot ";
    pols.add(0,pm.m_CurrentPolicy);
    for (std::list<std::string>::iterator pi = policy.begin();
	 pi != policy.end(); pi++) {
      std::cerr<<(*pi)<<"->";
      currentCfg=(*pi);
    }
  }
  pm.resetToPrior();
  pm.m_CurrentRoom=0;
  ec=pm.expectedCost(q,pols,p,t);
  std::cerr<<"\nExpectedCost="<<ec<<" "<<q<<"\n";
  pols.grow(0);
  getchar();
  pm.m_ExpectedCost=10;
  currentCfg="Unknown";
  std::cerr<<"\nTry next4StepPolicy:\n";
  while( pm.next4StepPolicy(policy,
			    currentCfg,
			    t,p)){
    std::cerr<<"\ngot ";
    pols.add(0,pm.m_CurrentPolicy);
    for (std::list<std::string>::iterator pi = policy.begin();
	 pi != policy.end(); pi++) {
      std::cerr<<(*pi)<<"->";
      currentCfg=(*pi);
    }
  }


  pm.resetToPrior();
  pm.m_CurrentRoom=0;
  ec=pm.expectedCost(q,pols,p,t);
  std::cerr<<"\nExpectedCost="<<ec<<" "<<q<<"\n";
  pols.grow(0);
  getchar();
  short type=1;
  pm.m_Type=type;

  pm.m_ExpectedCost=10;

  currentCfg="Unknown";

  std::cerr<<"\nTry NextNearlyBestPolicy Type "<<type<<":\n";
  while( pm.nextNearlyBestPolicy(policy,
				  currentCfg,
				  t,p)){
    std::cerr<<"\ngot ";
    pols.add(0,pm.m_CurrentPolicy);
    for (std::list<std::string>::iterator pi = policy.begin();
	 pi != policy.end(); pi++) {
      std::cerr<<(*pi)<<"->";
      currentCfg=(*pi);
    }
  }
  pm.resetToPrior();
  pm.m_CurrentRoom=0;
  ec=pm.expectedCost(q,pols,p,t);
  std::cerr<<"\nExpectedCost="<<ec<<" "<<q<<"\n";
  pols.grow(0);
  getchar();
  type=2;
  pm.m_Type=type;
  currentCfg="Unknown";
  std::cerr<<"\nTry NextNearlyBestPolicy Type "<<type<<":\n";
  while( pm.nextNearlyBestPolicy(policy,
				  currentCfg,
				  t,p)){
    std::cerr<<"\ngot ";
    pols.add(0,pm.m_CurrentPolicy);
    for (std::list<std::string>::iterator pi = policy.begin();
	 pi != policy.end(); pi++) {
      std::cerr<<(*pi)<<"->";
      currentCfg=(*pi);
    }
  }

  type=4;
  pm.resetToPrior();
  pm.m_CurrentRoom=0;
  ec=pm.expectedCost(q,pols,p,t);
 std::cerr<<"\nExpectedCost="<<ec<<" "<<q<<"\n";
  pols.grow(0);
  getchar();

  pm.m_Type=type;
  currentCfg="Unknown";
  std::cerr<<"\nTry NextNearlyBestPolicy Type "<<type<<":\n";
  while( pm.nextNearlyBestPolicy(policy,
				  currentCfg,
				  t,p)){
    std::cerr<<"\ngot ";
    pols.add(0,pm.m_CurrentPolicy);
    for (std::list<std::string>::iterator pi = policy.begin();
	 pi != policy.end(); pi++) {
      std::cerr<<(*pi)<<"->";
      currentCfg=(*pi);
    }
  }
  type=8;
  pm.resetToPrior();
  pm.m_CurrentRoom=0;
  ec=pm.expectedCost(q,pols,p,t);
  std::cerr<<"\nExpectedCost="<<ec<<" "<<q<<"\n";
  pols.grow(0);
  getchar();
  return 0;
}
