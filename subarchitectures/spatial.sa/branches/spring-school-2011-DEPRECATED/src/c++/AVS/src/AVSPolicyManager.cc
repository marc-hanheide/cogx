#include "AVSPolicyManager.hh"

#include "AddressBank/ConfigFileReader.hh"
#include "Utils/CureDebug.hh"
#include "Math/ShortMatrix.hh"
using namespace std;
void policyPrint(std::list<std::string> &lst){
  for (std::list<std::string>::iterator pi = lst.begin();
	 pi != lst.end(); pi++) {
       std::cerr<<(*pi)<<" -> ";
    }

    std::cerr<<"\n";
}
void  AVSPolicyManager::Configuration::print(){
  std::cerr<<m_Configuration<<"\n";
  std::cerr<<m_Final<<" "<<m_Room<<" "<<m_Index<<" "<<m_Probability
	   <<" "<<m_NumberOfSubsets<<" "<<m_NumberOfSupersets<<"\n";
  for (int i=0; i<m_NumberOfSubsets;i++)
    std::cerr<<m_Subsets[i]->m_Configuration<<"\n";
  std::cerr<<m_Configuration<<"Super\n";
  for (int i=0; i<m_NumberOfSupersets;i++)
    std::cerr<<m_Supersets[i]->m_Configuration<<"\n";
  std::cerr<<"\nP="<<m_Probability<<" "<<m_APrioriProbability<<"\n";
}
short AVSPolicyManager::Configuration::size(){
  std::istringstream sst(m_Configuration);
  std::string str;
  std::string pre="none";
  short k=0;
  if (sst>>str){
    k++;
    while (sst>>str){
      sst>>str;
     k++;
    }
  }
 return k;
}
void AVSPolicyManager::Configuration::operator =(Configuration &cfg){
  m_Configuration=cfg.m_Configuration;
  m_Room=cfg.m_Room;
  m_Final=cfg.m_Final;
  m_Index=cfg.m_Index;
  m_Probability=cfg.m_Probability;
  m_EliminatedProbability=cfg.m_EliminatedProbability;
  m_APrioriProbability=cfg.m_APrioriProbability;
}

AVSPolicyManager::Configuration *  
AVSPolicyManager::Configuration::getSubset(long m){
  for (long k=0;k<m_NumberOfSubsets;k++){
    Configuration *c=m_Subsets[k]->subset(m);
    if (c)return c;
  }
  return 0;
}    
AVSPolicyManager::Configuration *  
AVSPolicyManager::Configuration::subset(long &m){
  if (m==0)return this;
  m--;
  for (long k=0;k<m_NumberOfSubsets;k++){
    Configuration *c=m_Subsets[k]->subset(m);
    if (c)return c;
  }
  return 0;
}    
void  AVSPolicyManager::Configuration::init(){
  m_Index=-1;
  m_Room=-1;
  m_Final=false;
  m_Subsets=0;
  m_NumberOfSubsets=0;
  m_Supersets=0;
  m_NumberOfSupersets=0;
  m_Probability=-1;
  m_EliminatedProbability=0;
  m_APrioriProbability=0;
}
void  AVSPolicyManager::Configuration::clear(){
  if (m_Subsets) delete [] m_Subsets;
  if (m_Supersets) delete [] m_Supersets;
  m_Subsets=0;
  m_NumberOfSubsets=0;
  m_Supersets=0;
  m_NumberOfSupersets=0;
}

bool  AVSPolicyManager::Configuration::isSubset(Configuration &cfg){
  if (cfg==(*this))return true;
  if (m_NumberOfSubsets==0)return false;
  for (long k=0;k<m_NumberOfSubsets;k++){
    if (m_Subsets[k]->isSubset(cfg))return true;
  }
  return false;
}

void AVSPolicyManager::Configuration::setFinal(std::string &object){
  std::istringstream sst(m_Configuration);
  std::string str;
  m_Final=false;
  if (sst>>str)
    if (str==object)m_Final=true;
}
bool AVSPolicyManager::Configuration::moreOrEqualGeneral(std::string &a, 
							 std::string &b){
  std::istringstream sst(a);
  std::istringstream sst2(b);
  std::string str,str2;
  std::string pre="none";
  std::string pre2="none";
  bool ok;
  while (sst>>str){
    ok=false;
    while (sst2>>str2){
      if (str2==str){
	if ((pre!=pre2)&&(pre!="none"))return false;
	ok=true;
	sst2>>pre2;
	break;
      }else{
	if (!(sst2>>pre2))return false;
      }
    }
    if (!ok)return false;
    if (!(sst>>pre))return true;
  }
  //should never come here.
  return true;
}
void AVSPolicyManager::Configuration::pruneSubsets(){
  return;
  Configuration *cfg;
  for (int i=0;i<m_NumberOfSubsets;i++){ 
    cfg=m_Subsets[i];
    for (int j=0;j<m_NumberOfSubsets;j++){ 
      if (m_Subsets[j]->addToSubsets(cfg)){
	removeFromSubsets(i);
	i--;
	break;
      }
    }
  }
}
bool AVSPolicyManager::Configuration::addToSupersets(Configuration *cfg){
  if (cfg==this)return false;
  if (cfg->m_Configuration==m_Configuration)return false;
  if (moreOrEqualGeneral(cfg->m_Configuration,m_Configuration)){
    Configuration **s=m_Supersets;
    m_Supersets=new Configuration*[m_NumberOfSupersets+1];
    if (s){
      memcpy(m_Supersets,s,m_NumberOfSupersets*sizeof(Configuration*));
      delete[]s;
    }
    m_Supersets[m_NumberOfSupersets]=cfg;
    m_NumberOfSupersets++;
    return true;
  }
  return false;
}
void AVSPolicyManager::Configuration::removeFromSubsets(int ind)
{
  m_NumberOfSubsets--;
  for (int i=ind; i<m_NumberOfSubsets;i++){
    m_Subsets[i]=m_Subsets[i+1];
  }
}
bool AVSPolicyManager::Configuration::addToSubsets(Configuration *cfg){
  if (cfg==this)return false;
  if (cfg->m_Configuration==m_Configuration)return false;
  if (moreOrEqualGeneral(m_Configuration,cfg->m_Configuration)){
    // So we do not need all subsets on our list just 
    // be able to reach all subsets by following links.
    for (int i=0;i<m_NumberOfSubsets;i++){
      if (m_Subsets[i]==cfg)return true;
      if (m_Subsets[i]->addToSubsets(cfg))return true;
    }
    Configuration **s=m_Subsets;
    m_Subsets=new Configuration*[m_NumberOfSubsets+1];
    if (s){
      memcpy(m_Subsets,s,m_NumberOfSubsets*sizeof(Configuration*));
      delete[]s;
    }
    m_Subsets[m_NumberOfSubsets]=cfg;
    m_NumberOfSubsets++;
    return true;
  }
  return false;
}
void AVSPolicyManager::Configuration::eliminate(double &p){
  if (m_NumberOfSubsets>0){
    for (long i=0;i<m_NumberOfSubsets;i++){
      m_Subsets[i]->eliminate(p);
    }
  }
  if (m_Probability>0){
    for (long i=0; i<m_NumberOfSupersets;i++){
      m_Supersets[i]->update(m_Probability);
    }
  }
  p+=m_Probability;
  m_EliminatedProbability+=m_Probability;
  m_Probability=0;
}
void AVSPolicyManager::Configuration::update(double p){
  if (m_Probability>0){
    m_Probability-=p;
    m_EliminatedProbability+=p;
    if (m_Probability<0){
      m_EliminatedProbability+=m_Probability;
      m_Probability=0;
    }
  }
}

void AVSPolicyManager::Configuration::normalize(double p,double percentage){
  if (m_Probability<0)return;
  m_EliminatedProbability*=(percentage);
  m_Probability+=m_EliminatedProbability;
  m_Probability*=p;
  m_EliminatedProbability=0;
}

void AVSPolicyManager::PolicyPath::print(){
  std::cerr<<"Costs ";
  for (long i=0;i<m_NumberCosts;i++)
    std::cerr<<m_C[i]<<" ";
  std::cerr<<"\nProbs ";
  for (long i=0;i<m_NumberProbs;i++)
    std::cerr<<m_Element[i]<<" ";
  std::cerr<<"\n";
  for (long i=0;i<m_NumberSteps;i++)
    std::cerr<<m_Index[i]<<" ";
  std::cerr<<"\n";
}

void AVSPolicyManager::Policy::print(){
  std::cerr<<"Policy for room "<<m_Room<<"\n";
    for (std::list<std::string>::iterator pi = m_Policy.begin();
	 pi != m_Policy.end(); pi++) {
       std::cerr<<(*pi)<<" -> ";
    }
    std::cerr<<"\n";
}
void  AVSPolicyManager::PolicyTree::listPolicies(Policy *policies){
  if (m_NumberOfChildren>0){
    for (short i=0;i<m_NumberOfChildren;i++){
      m_Children[i]->listPolicies(policies);
      while (policies->m_Next){
	policies->add(m_Configuration);
	    policies=policies->m_Next;
      }
    }
  }else{	//No children means it is a room
	policies=policies->addNew(m_Configuration,true);
  }
}
void AVSPolicyManager::PolicyTree::completeList(std::list<std::string> 
						&configurations,
						std::list<std::string> 
						&more)
{
  std::istringstream sst(m_Configuration);
  std::string tok, object;
  short cnt=0;
  while (sst>>tok){
    cnt++;
    if (sst>>tok){
      if (!((tok=="IN")//||(tok=="In")||(tok=="in")
	    ||(tok=="ON"))){//||(tok=="On")||(tok=="on"))){
	std::cerr<<"ERROR: Mal-formed Configuration "<<m_Configuration<<"\n";
	return;
      }
    }else break;
  }
  if (cnt>1){
    std::string x[cnt-1];
    std::string p[cnt-1];
    sst.clear();
    sst.str(m_Configuration);
    
    cnt=0;
    sst>>object;  //read past the current object
    while (sst>>tok){
      p[cnt]=tok;
      sst>>tok;
      x[cnt]=tok;
      cnt++;
    }
    Cure::LongArray combin,poss;
    for (int i=0;i<(cnt-1);i++)
      poss.add(0,i);

    poss.combinations(combin);
    //So all possible combinations of tokens are in combin ech on its own
    //row starting with poss (ie all tokens) and ending with an empty row
    // (ie no tokens).  The last row is the case just find the room then do
    // the search for this configuration.   
    for (long i=0;i<combin.rows(); i++){
      std::ostringstream osst;
      long k=combin.columns(i);
      osst<<object;
      for (int j=0;j<k;j++){

	long n=combin(i,j);
	osst<<" "<<p[n]<<" ";
	osst<<x[n];
      }
      osst<<" "<<p[cnt-1]<<" "<<x[cnt-1];
      tok=osst.str();
      bool ok=false;
      for (std::list<std::string>::iterator pi = configurations.begin();
	   pi != configurations.end(); pi++) {
	if (tok==(*pi)){
	  ok=true;
	  break;
	}
      }
      if (!ok){ 
	for (std::list<std::string>::iterator pi = more.begin();
	   pi != more.end(); pi++) {
	  if (tok==(*pi)){
	    ok=true;
	    break;
	  }
	}
	if (!ok){
	  more.push_back(tok);
	}
      }
    }
  }
}
void AVSPolicyManager::PolicyTree::setConfiguration(std::string tmp){
  m_Configuration=tmp;     
  std::istringstream sst(m_Configuration);
  std::string tok, child;
  short cnt=0;

  clearChildren();
  while (sst>>child){
    cnt++;

    if (sst>>tok){
      // Nice but lead to harder comparisons later
      if (!((tok=="IN")//||(tok=="In")||(tok=="in") 
	    ||(tok=="ON"))){//||(tok=="On")||(tok=="on"))){
	std::cerr<<"ERROR: Malformed Configuration "<<tmp<<"\n";
	return;
      }
    }else break;

  }
  m_Size=cnt;
  if (cnt>1){
    std::string x[cnt-1];
    std::string p[cnt-1];
    sst.clear();
    sst.str(m_Configuration);

    cnt=0;
    sst>>tok;  //read past the current object
    while (sst>>tok){
      p[cnt]=tok;
      sst>>tok;
      x[cnt]=tok;
      cnt++;
    }
    Cure::LongArray combin,poss;
    for (int i=0;i<(cnt-1);i++)
      poss.add(0,i);
     poss.combinations(combin);
    if (cnt==1)combin.grow(1);
   //So all possible combinations of tokens are in combin ech on its own
    //row starting with poss (ie all tokens) and ending with an empty row
    // (ie no tokens).  The last row is the case just find the room then do
    // the search for this configuration.   
    m_NumberOfChildren =combin.rows();
    m_Children=new PolicyTree*[m_NumberOfChildren];
    int ii=0;
    for (long i=0;i<m_NumberOfChildren;ii++){
      std::ostringstream osst;
      long k=combin.columns(ii);
      bool good=true;
      int m=cnt-2;
      for (int j=k-1;j>=0;j--){
	long n=combin(ii,j);
	if (n!=m){
	  good=false; 
	  break;
	}
	m=n-1;
      }
      if (good){
	for (int j=0;j<k;j++){
	  long n=combin(ii,j);
	  if (j!=0)osst<<p[n]<<" ";
	  osst<<x[n]<<" ";
	  //So not good is logically a possible policy but it makes no sense
	  // find B in room1 -> find B on C in room1 .
	}
	if (k==0)osst<<x[cnt-1];
	else osst<<p[cnt-1]<<" "<<x[cnt-1];
	tok=osst.str();
	m_Children[i]=new PolicyTree(this);
	m_Children[i]->setConfiguration(tok);
	i++;
      }else m_NumberOfChildren--;
    }
  }
}
AVSPolicyManager::AVSPolicyManager(){
  m_Configs=0;
  m_NumberOfConfigs=0;
  m_CurrentPolicy=-1;
  m_MaxVaribles=2;  
  m_ConfigurationsFilename="configs.txt";
  m_RoomsFilename="rooms.txt";
  m_CurrentRoom=0; 
  m_NumberOfRooms=0;
  m_CostCredit=1.0;
  m_MinimumCost=.5;
   m_DependPolicies=0;
  m_MaxSteps=10;
  m_Type=1;
  m_ExpectedCost=10.0;
  m_ProbabilityItsHere=0;
  m_Threshold=1.0;
  m_Rooms=0;
}
void  AVSPolicyManager::clearConfigs(){
  if (m_Configs)delete[]m_Configs;
  m_NumberOfConfigs=0;
  m_Configs=0;
  m_PolicyConfigs.grow(0);
  if (m_Rooms)delete[]m_Rooms;
  m_Rooms=0;
  m_NumberOfRooms=0;
}
void  AVSPolicyManager::addToConfigs(std::string str){
  Configuration *s=m_Configs;
  m_Configs=new Configuration[m_NumberOfConfigs+1];
  Configuration *cc=&m_Configs[m_NumberOfConfigs];
  cc->m_Configuration=str;
  cc->m_Index=m_NumberOfConfigs;
  for (int i=0;i<(m_NumberOfConfigs);i++){
      m_Configs[i]=s[i];
  }

  if (s)delete[]s;
  m_NumberOfConfigs++;
  m_Credit.reallocate(1,m_NumberOfConfigs,false);
}
long  AVSPolicyManager::getConfig(std::string &str){
  for (int i=0;i<m_NumberOfConfigs;i++){
    if (str==m_Configs[i].m_Configuration){
      return i;
    }
  }
  addToConfigs(str);
  return m_NumberOfConfigs-1;
}
long  AVSPolicyManager::findConfig(std::string &str){
  for (int i=0;i<m_NumberOfConfigs;i++){
    if (str==m_Configs[i].m_Configuration){
      return i;
    }
  }
  return -1;
}
void  AVSPolicyManager::getPolicy(long row,std::list<std::string> &pol){
  pol.clear();
  long c=m_PolicyConfigs.columns(row);
  for (long i=0;i<c;i++)
    pol.push_back(m_Configs[m_PolicyConfigs(row,i)].m_Configuration);
}
void AVSPolicyManager::getExclusiveConfigurations
(std::list<std::string> &configurations,
 Cure::Matrix &probabilities,
 std::string searchTarget){
  configurations.clear();
  probabilities.grow(0,1);
  getFileConfigurations(configurations,probabilities,searchTarget);
}

void  AVSPolicyManager::getConfigurations(std::list<std::string> 
					  &configurations,
					  Cure::Matrix &probabilities,
					  std::string searchTarget){
  configurations.clear();
  probabilities.grow(0,1);
  getFileConfigurations(configurations,probabilities,searchTarget);
  //IMPLEMENTATION NEEDED#############################################################################
  
  //HERE WE shall call some function that generates 
  //configurations and probabilities (ie the Bayes Net).
}
  
void AVSPolicyManager::addProbabilities(Cure::Matrix &probabilities,
					std::list<std::string> &more){
  for (std::list<std::string>::iterator pi = more.begin();
       pi != more.end(); pi++) {
    probabilities.grow(probabilities.Rows+1,1);
    //IMPLEMENTATION NEEDED#############################################################################
    //   probabilities(probabilities.Rows-1,0)=BN.getProbability(*pi);
  }
}

void AVSPolicyManager::listPolicies(std::string searchTarget){
  m_SearchTarget=searchTarget;
  std::list<std::string> configurations;
  Cure::Matrix probabilities;
  getConfigurations(configurations,probabilities,searchTarget);
  configure(configurations,probabilities);
}
void AVSPolicyManager::configure(std::list<std::string> &configurations,
	       Cure::Matrix &probabilities)
{
  Policy policies;
  std::list<std::string> more;
  std::list<std::string> rooms;
  getFileRoomCosts(rooms,m_RoomCosts);
  m_NumberOfRooms=m_RoomCosts.Rows;
  if (m_Rooms)delete[]m_Rooms;
  m_Rooms=0;
  policies.clear();
  clearConfigs();
  Policy *pol =&policies;
  for (std::list<std::string>::iterator pi = configurations.begin();
       pi != configurations.end(); pi++) {
    while (pol->m_Next){
      pol=pol->m_Next;
    }
    PolicyTree pt(*pi);
    pt.listPolicies(pol);
    pt.completeList(configurations,more);
  }
  addProbabilities(probabilities,more);
  for (std::list<std::string>::iterator pi = more.begin();
       pi != more.end(); pi++) {
    while (pol->m_Next)pol=pol->m_Next;
    PolicyTree pt(*pi);
    pt.listPolicies(pol);
  }
  m_PolicyConfigs.grow(0);
  m_PolicyRooms.grow(0);
  m_RoomPolicy.grow(0);
  if(policies.m_Next==0)return;
  Policy *p=&policies;
  std::string str1,str2;
  str1="init";
  str2="notinit and not anything else";
  double prob=probabilities(0,0);

  long k=-1;
  long i=-1;
  long row=0;
  short sizep, size;
  std::string room;
  long roomindex=-1;
  while (p->m_Next){
    if (p->m_Room!=room){
      room=p->m_Room;
      roomindex=0;
      for (std::list<std::string>::iterator pi = rooms.begin();
	   pi != rooms.end(); pi++,roomindex++) {
	if (room==(*pi))break;
      }
    }
    sizep=0;
    for (std::list<std::string>::iterator pi = p->m_Policy.begin();
	 pi != p->m_Policy.end(); pi++) {
      str1=(*pi);
      k=getConfig(str1);
      m_PolicyConfigs.add(row,k);
      size=m_Configs[k].size();
      if ((size-sizep)>m_MaxVaribles){
	m_PolicyConfigs.grow(row);//This erases the row just added
	break;
      }
      sizep=size;
    }
    if (m_PolicyConfigs.rows()>row){//we added a config
      m_PolicyRooms.add(row,roomindex);
      m_RoomPolicy.add(roomindex,row);
      if (str1!=str2){//we are at the next final configuration
	i++;
	
	str2=str1;
	prob=probabilities(i,0);
	m_Configs[k].setAPrioriProbability(prob);
      }
      row++;
    }
    p=p->m_Next;
  }
  for (i=0;i<(m_NumberOfConfigs);i++)
    for (int j=0;j<(m_NumberOfConfigs);j++)
      m_Configs[i].add(&m_Configs[j]);

  for (i=0;i<m_NumberOfConfigs;i++)

    m_Configs[i].pruneSubsets();

  

  for (i=0;i<m_NumberOfConfigs;i++){
    m_Configs[i].setFinal(m_SearchTarget);
    room=m_Configs[i].getRoom();
    roomindex=0;
    for (std::list<std::string>::iterator pi = rooms.begin();
	 pi != rooms.end(); pi++,roomindex++) {
      if (room==(*pi))break;
    }
    m_Configs[i].m_Room=roomindex;
  }
   makeDepend();
}
void AVSPolicyManager::listExclusivePolicies(std::string searchTarget){
  std::list<std::string> configurations;
  Cure::Matrix probabilities;
  m_SearchTarget=searchTarget;
  getExclusiveConfigurations(configurations,probabilities,searchTarget);
  configureExclusive(configurations,probabilities);
}
void AVSPolicyManager::
configureExclusive(std::list<std::string> &configurations,
		   Cure::Matrix &probabilities){
  
  std::list<std::string> more;
  Policy policies;
  std::list<std::string> rooms;
  getFileRoomCosts(rooms,m_RoomCosts);
  m_NumberOfRooms=m_RoomCosts.Rows;
  if (m_Rooms)delete[]m_Rooms;
  m_Rooms=0;
  policies.clear();
  clearConfigs();
  Policy *pol =&policies;
  probabilities.print();
  for (std::list<std::string>::iterator pi = configurations.begin();
       pi != configurations.end(); pi++) {
    
    while (pol->m_Next){
      pol=pol->m_Next;
    }
    PolicyTree pt(*pi);
    pt.listPolicies(pol);
    pt.completeList(configurations,more);
  }
  //Now We have a list configurations that contain mutually exlusive configs
  // and a matrix probabilities that contains their probs. and
  // a list more that contains all other configurations for which we need
  // infer probabilities.  We need to convert the probabities by
  // adding all subsets with probabilities to supersets 

  for (std::list<std::string>::iterator pi = more.begin();
       pi != more.end(); pi++) {
    while (pol->m_Next)pol=pol->m_Next;
    PolicyTree pt(*pi);
    pt.listPolicies(pol);
  }
  m_PolicyConfigs.grow(0);
  m_PolicyRooms.grow(0);
  m_RoomPolicy.grow(0);
  if(policies.m_Next==0)return;
  Policy *p=&policies;
  std::string str1,str2;
  str1="init";
  str2="notinit and not anything else";
  double prob=probabilities(0,0);

  long k=-1;
  long i=-1;
  long row=0;
  short sizep, size;
  std::string room;
  long roomindex=-1;
  Cure::LongArray probtoconfig;
  while (p->m_Next){
    if (p->m_Room!=room){
      room=p->m_Room;
      roomindex=0;
      for (std::list<std::string>::iterator pi = rooms.begin();
	   pi != rooms.end(); pi++,roomindex++) {
	if (room==(*pi))break;
      }
    }
    sizep=0;
    for (std::list<std::string>::iterator pi = p->m_Policy.begin();
	 pi != p->m_Policy.end(); pi++) {
      str1=(*pi);
      k=getConfig(str1);
      m_PolicyConfigs.add(row,k);
      size=m_Configs[k].size();
      if ((size-sizep)>m_MaxVaribles){
	m_PolicyConfigs.grow(row);//This erases the row just added
	break;
      }
      sizep=size;
    }
    if (m_PolicyConfigs.rows()>row){//we added a config
      m_PolicyRooms.add(row,roomindex);
      m_RoomPolicy.add(roomindex,row);
      if (str1!=str2){//we are at the next final configuration
	i++;
	
	str2=str1;
	probtoconfig.add(i,k);
	//prob=probabilities(i,0);
	//m_Configs[k].setAPrioriProbability(prob);
      }
      row++;
    }
    p=p->m_Next;
  }
  for (i=0;i<(m_NumberOfConfigs);i++)
    for (int j=0;j<(m_NumberOfConfigs);j++)
      m_Configs[i].add(&m_Configs[j]);

  for (i=0;i<m_NumberOfConfigs;i++)
    m_Configs[i].pruneSubsets();
  for (i=0;i<m_NumberOfConfigs;i++){
    m_Configs[i].setFinal(m_SearchTarget);
    room=m_Configs[i].getRoom();
    roomindex=0;
    for (std::list<std::string>::iterator pi = rooms.begin();
	 pi != rooms.end(); pi++,roomindex++) {
      if (room==(*pi))break;
    }
    m_Configs[i].m_Room=roomindex;
  }

  long n=probabilities.Rows;
  for (long i=0;i<n;i++){
    long k=probtoconfig(i,0);
    for (long j=0; j<m_Configs[k].m_NumberOfSupersets;j++)
      m_Configs[k].m_Supersets[j]->m_APrioriProbability+=probabilities(i,0);
    m_Configs[k].m_APrioriProbability+=probabilities(i,0);
  }
  for (i=0;i<m_NumberOfConfigs;i++)
    m_Configs[i].resetProbability();
  for (i=0;i<m_NumberOfConfigs;i++)
    m_Configs[i].print();
  makeDepend();
}
  /**
   * Each room i, has a set of configs that are not rooms or
   * final searches and are supersets of two or more final search
   * configurations with Probabilities >0 and which are either
   * disjoint or have unequal probabilities.  The indecies of this set
   * of Configs are listed in row i of this.
   */
void AVSPolicyManager::makeDepend(){
  m_Depend.grow(0);
  m_NumberOfRooms=m_RoomCosts.Rows;
  if (m_Rooms)delete[]m_Rooms;

  m_Rooms=new Configuration*[m_NumberOfRooms];
  m_Depend.grow(m_NumberOfRooms);
  for (long i=0;i<m_NumberOfConfigs;i++)
    if (!m_Configs[i].m_Final)
      if (m_Configs[i].m_NumberOfSupersets!=0){
	bool addok=false;
	long n=0;
	Configuration *c= m_Configs[i].getSubset(n);
	Configuration *c2;
	while (c){
	  if ((c->m_Final)&&(c->m_Probability>0)){
	    long m=n+1; 
	    c2= m_Configs[i].getSubset(m);    
	    while (c2){
	      m++;
	      if (c!=c2)
		if ((c2->m_Final)&&(c2->m_Probability>0)){
		  double diff=(c2->m_Probability-c->m_Probability);
		  if ((diff>1E-3)||(diff<-1E-3)){
		    addok=true;
		  } else if ((!c->isSubset(*c2))&&(!c2->isSubset(*c))){
		    addok=true;
		  }
		}
	      if (addok)break;
	      c2= m_Configs[i].getSubset(m);
	    }

	  }
	  n++;
	  c= m_Configs[i].getSubset(n);
	  if (addok)break;
	}
	if (addok){
	  m_Depend.add(m_Configs[i].m_Room,i);
	}
      }
  /**
   * Each Policy of row i of m_PolicyConfigs may contain a search for the
   * configuration: m_Configs[m_Depend(m_PolicyRooms(i,0),j)].
   *  If it does then j is listed on row i of this.
   */
  m_PolicyDepend.grow(0);
  m_PolicyDepend.grow(m_PolicyConfigs.rows());
  if (m_DependPolicies)delete []m_DependPolicies;
  m_DependPolicies=new Cure::LongArray[m_Depend.rows()];
  for (int i=0;i<m_Depend.rows();i++){
    m_DependPolicies[i].grow(m_Depend.columns(i));
  }
  for (long i=0; i<m_PolicyConfigs.rows();i++){
    int room=m_PolicyRooms(i,0);
    int top=m_Depend.columns(room);
    int n=m_PolicyConfigs.columns(i);
    for (int j=0; j<top;j++){
      long index=m_Depend(room,j);

      for (int k=0;k<n;k++)
	if (m_PolicyConfigs(i,k)==index){
	  m_PolicyDepend.add(i,j);

	  m_DependPolicies[room].add(j,i);
	}
    }
  }
  m_ProbabilityItsHere=0;
  m_RoomProbability.reallocateZero(3,m_NumberOfRooms);

  for (int room=0; room<m_NumberOfRooms;room++){
    if (0<m_RoomPolicy.columns(room)){
	long k=m_PolicyConfigs(m_RoomPolicy(room,0),0);
	double d=m_Configs[k].getRoomAPrioriProbability();
	m_RoomProbability(0,room)=d;
	m_ProbabilityItsHere+=d;
	m_Rooms[room]=m_Configs[k].getRoomConfiguration();
    }else {
      std::cerr<<"ERROR Room has no Policy "<<room<<"\n";
      m_Rooms[room]=0;
      m_RoomProbability(0,room)=0;
    }
  }
  if (m_ProbabilityItsHere>.99){//We can not allow certianty
    double d=.99/m_ProbabilityItsHere;
    for (long i=0;i<m_NumberOfConfigs;i++){
      double p=m_Configs[i].m_Probability;
      p*=d;
      m_Configs[i].setAPrioriProbability(p);
    }
    m_ProbabilityItsHere=.99;
  }
  for (int room=0; room<m_NumberOfRooms;room++)
    m_RoomProbability(2,room)=(1+m_RoomProbability(0,room))/
   (m_NumberOfRooms+m_ProbabilityItsHere);
}
bool AVSPolicyManager::nextNearlyBestPolicy(std::list<std::string> &pol, 
					    std::string &currentCfg,
					    double threshold,
					    double percentage) 
{
  update(currentCfg,percentage);
  m_Threshold = threshold;
  bool ok=false;
  for (int i=0;i<m_NumberOfRooms;i++)
    if (m_Rooms[i]->m_Probability>m_Threshold){
      ok=true;
      break;
    }
  if (!ok)return false;
  long restorepolicy=m_CurrentPolicy;
  long restoreroom=m_CurrentRoom;
  PolicyPath restorePath;
  allocate(restorePath,1);
  save(restorePath);
  PolicyPath bestPaths[m_NumberOfRooms];

  short num[m_NumberOfRooms];
  long maxprev=1;
  for (int i=0; i<m_NumberOfRooms; i++){
    long m=(m_Depend.columns(i)+1); //how many branches we need in room i
    num[i]=0;
    if (m>maxprev)maxprev=m;
    allocate(bestPaths[i],m_MaxSteps);
  } 
  // we need to got from step n-1 to n in each room where we project all 
  // numprev saved paths using all policies P to form MP new n step paths
  // then save the best and one that contains each m_Depend for the room.
  PolicyPath prevPaths[maxprev];
  PolicyPath nextPaths[maxprev];
  Cure::LongArray nextPolicy;
  double bestsofar=0;
  bool bestset=false;
  nextPolicy.grow(maxprev);
  Cure::Matrix q(maxprev,2);
  double nextCosts[maxprev];
  for (long i=0; i<maxprev; i++){
    allocate(prevPaths[i],m_MaxSteps+1);
    allocate(nextPaths[i],m_MaxSteps+1);
    nextCosts[i]=1E99;

    nextPolicy.makeCell(i,0)=-1; //index into RoomPolicy
    nextPolicy.makeCell(i,1)=-1; //index into prevPaths
  }
  short numprev;
  short numnext;
  long numpol;
  for (int room=0; room<m_NumberOfRooms; room++){
    bestset=false;
    numprev=1;

    bestPaths[room]=prevPaths[0];
    numpol=m_RoomPolicy.columns(room);
    restore(restorePath);
    for (int k=0; k<m_NumberOfRooms;k++)
      if (k!=room)  eliminate(m_Rooms[k]->m_Index,1.0);
    save(prevPaths[0]);
    if (m_Rooms[room]->m_Probability>m_Threshold)
      for ( int step=0; step<m_MaxSteps; step++){

      numnext=m_Depend.columns(room)+1;
      for (long i=0; i<numnext; i++){
	nextPolicy(i,0)=-1; //index into RoomPolicy
	nextPolicy(i,1)=-1; //index into prevPaths
	nextCosts[i]=1E99;
      }

      double prob,c,cost;
      for (int k=0; k<numprev;k++){
	for (int ii=0;ii<numpol;ii++){ 
	  long poly=m_RoomPolicy(room,ii);
	  if (1){//((step>0)||(getProb(poly)>m_Threshold)){
	    prob=getProb(prevPaths[k],poly);
	    c=prevPaths[k].costs(poly);
	    if ((c>0)&&(prob>1E-6)){//m_RoomProbability(1,room))){	     
	      prob*=percentage; 
	      cost=prevPaths[k].expectedCost(prob, c/2, c);
	      prob=(1.0-prob)*prevPaths[k].m_Q;
	      if (m_Type&1)
		c=cost+(prob)*m_ExpectedCost;
	      else if (m_Type&2)
		c=cost/(1.0-prob);
	      else if (m_Type&4){
		//c=cost/(1.0-prob*prob);
		c*=(m_ProbabilityItsHere);
		c/=(m_ProbabilityItsHere-prob*(prob-1+m_ProbabilityItsHere));
	      }
		
	      for (long i=0; i<numnext; i++){
		if ((c<nextCosts[i])||(nextPolicy(i,0)==-1)){
		  if (i==0){
		    nextCosts[i]=c;
		    nextPolicy(i,0)=ii;
		    nextPolicy(i,1)=k;
		    q(i,0)=prob;
		    q(i,1)=cost;
		  }else{
		    //does prePaths +  m_PolicyConfigs(poly) contain 
		    // m_Depend(room,i-1)
		    bool polyok=prevPaths[k].contains(i-1);
		    if (!polyok){
		      long poly=m_RoomPolicy(room,ii);	  
		      int cols=m_PolicyDepend.columns(poly);
		      for (int j=0;j<cols;j++)

			if (m_PolicyDepend(poly,j)==i-1){
			  polyok=true;
			  break;
			}
		    }		  
		    if (polyok){
		      nextCosts[i]=c;
		      nextPolicy(i,0)=ii;
		      nextPolicy(i,1)=k;
		      q(i,0)=prob;
		      q(i,1)=cost;
		    }
		  }
		}
	      }
	    }
	  }
	}
      }
      // we may not have found any policy with prob left      
      for (long i=0; i<numnext; i++)
	if (nextPolicy(i,0)==-1){
	  for( long k=i+1;k<numnext;k++){
	    nextPolicy(k-1,0)=nextPolicy(k,0);
	    nextCosts[k-1]=nextCosts[k];
	    nextPolicy(k-1,1)=nextPolicy(k,1);
	    q(k-1,0)=q(k,0);

	    q(k-1,1)=q(k,1);
	  }
	  numnext--;
	  i--;
	}
      // we may have exactly the same path twice
      for (long i=0; i<numnext; i++)
	for (long j=i+1; j<numnext; j++)
	  if (nextPolicy(i,0)==nextPolicy(j,0))
	    if (nextPolicy(i,1)==nextPolicy(j,1)){
	      for( long k=j+1;k<numnext;k++){
		nextCosts[k-1]=nextCosts[k];
		nextPolicy(k-1,0)=nextPolicy(k,0);
		nextPolicy(k-1,1)=nextPolicy(k,1);
		q(k-1,0)=q(k,0);
		q(k-1,1)=q(k,1);
	      }
	      numnext--;
	      j--;
	    }
      // finally we need to set up the nextPaths that is the saved paths
      for (long i=0; i<numnext; i++){	
	PolicyPath *p=&prevPaths[nextPolicy(i,1)];
	restore(*p); // put Configs and Costs at state of p
	m_CurrentPolicy=m_RoomPolicy(room,nextPolicy(i,0));
	
	nextPaths[i].m_Q=q(i,0);
	nextPaths[i].m_ExpectedCost=q(i,1);
	for (int j=0;j<step;j++){
	  nextPaths[i].m_Index[j]=p->m_Index[j];
	}
	nextPaths[i].m_Index[step]=m_CurrentPolicy;
	nextPaths[i].m_Dependent=p->m_Dependent;
	for (int j=0;j<m_PolicyDepend.columns(m_CurrentPolicy);j++)
	  nextPaths[i].m_Dependent.
	    addUnique(0,m_PolicyDepend(m_CurrentPolicy,j));
	update(percentage);
	save(nextPaths[i]);
      }
      for (long i=0; i<numnext; i++){	
	prevPaths[i]=(nextPaths[i]);
      }
      numprev=numnext;
      if (numprev>0){
	if ((prevPaths[0].probs(m_Rooms[room]->m_Index)<m_Threshold)||
	    (step==(m_MaxSteps-1)))
	  if ((bestsofar>nextCosts[0])||(!bestset))
	    {
	      bestPaths[room]=prevPaths[0];
	      num[room]=step+1;
	      bestset=true;
	      bestsofar=nextCosts[0];
	    }
      } else {
	break;
      }
    }
  }
  
  
  //  std::cerr<<" ";
  //  for(int i=0; i<m_NumberOfRooms;i++)
  //std::cerr<<num[i]<<" ";
  //  restorePath.print();
  // for(int i=0; i<m_NumberOfRooms;i++)
  //  bestPaths[i].print();
  m_CurrentPolicy=restorepolicy;
  m_CurrentRoom=restoreroom;
  return multiRoomSearch(pol,num, bestPaths, restorePath,percentage);
}
void AVSPolicyManager::iterate(PolicyPath *paths,
			       short step,long &bestpol,
			       double &best, long &poly,
			       short *prevroom,short *roomcnt,
			       short *stepsInRoom,PolicyPath *bestPaths,
			       double percentage)
{
  
  for (int room=0; room<m_NumberOfRooms;room++){
    if (roomcnt[room]<stepsInRoom[room]){
      restore(paths[step]);
      m_CurrentRoom=prevroom[step];
      m_CurrentPolicy=
	bestPaths[room].m_Index[roomcnt[room]];
      roomcnt[room]++;
      if (step==0)poly=m_CurrentPolicy;
      double c=m_Costs(m_CurrentPolicy,0);
      double prob=getProb(m_CurrentPolicy);
      bool terminate=true;
      bool good=false;
      if ((c>0)&&(prob>1E-15)){
	prob*=percentage;
	c/=2.0;
	good=true;
	c=paths[step].m_Q*(c+(1.0-prob)*c+
			   m_RoomCosts(room,prevroom[step]));
	c=paths[step].m_ExpectedCost+c;
	prob=paths[step].m_Q*(1.0-prob);
	update(percentage);
	if (step<(m_MaxSteps-1)){
	  for (int k=0; k<m_NumberOfRooms;k++)
	    if (roomcnt[k]<(stepsInRoom[k])){
	      terminate=false;
	      break;
	    }
	}
      }else{
	if (prob>1E-12)c=-1;
	else{
	  if (step>0)
	    good=true;
	  c=paths[step].m_ExpectedCost;
	  prob=paths[step].m_Q*(1.0-prob);
	}
      }
      if (terminate){
	if (good){
	  if (step<(m_MaxSteps-1)){
	    for (int k=0; k<m_NumberOfRooms;k++)
	      if (roomcnt[k]<(stepsInRoom[k])){
		if (m_Rooms[k]->m_Probability>m_Threshold){
		  good=false;
		  break;
		}
	      }
	  }
	  if (good){
	    for (int k=0; k<m_NumberOfRooms;k++)
	      if (m_Rooms[k]->m_Probability>m_Threshold){
		good=false;
		break;
	      }
	    if (!good)
	      if (m_Type&1)
		c+=prob*m_ExpectedCost;
	      else if (m_Type&2)
		c/=(1.0-prob);
	      else if (m_Type&4){
		c*=(m_ProbabilityItsHere);
		c/=(m_ProbabilityItsHere-prob*(prob-1+m_ProbabilityItsHere));
	      }
	    if ((bestpol==-1)||(best>c)){
	      bestpol=poly;
	      best=c;
	    }
	  }
	}
      }else{
	save(paths[step+1]);
	prevroom[step+1]=room;
	paths[step+1].m_ExpectedCost=c;
	paths[step+1].m_Q=(prob);	
	iterate(paths,step+1,bestpol,best,poly,prevroom,roomcnt,
		stepsInRoom,bestPaths);
      }
      roomcnt[room]--;
    }
  }
}
bool AVSPolicyManager::multiRoomSearch(std::list<std::string> &pol, 
				       short *stepsInRoom,
				       PolicyPath *bestPaths,  
				       PolicyPath &restorePath,
				       double percentage) 
{
  bool ok=true;
  if (m_MaxSteps<1)ok=false;
  else{
    ok=false;
    for (int i=0;i<m_NumberOfRooms;i++)
      if (stepsInRoom[i]>0){
	ok=true;
	break;
      }
  }
  if (m_NumberOfRooms<2){
    if (m_NumberOfRooms<1){
      ok=false;
    }else{
      getPolicy(bestPaths[0].m_Index[0],pol);
      m_CurrentPolicy=bestPaths[0].m_Index[0];
      return true;
    }
  }
  if (!ok){
    restore(restorePath);
    return false;
  }

  long restorepolicy=m_CurrentPolicy;
  long restoreroom=m_CurrentRoom;
  PolicyPath paths[m_MaxSteps+1];
  for (int step=0; step<m_MaxSteps;step++){
    allocate(paths[step+1],m_MaxSteps);
  }
  paths[0]=restorePath;
  paths[0].m_ExpectedCost=0;
  paths[0].m_Q=1;
  short prevroom[m_MaxSteps+1];
  memset(prevroom+1,0,m_MaxSteps*sizeof(short));
  prevroom[0]=m_CurrentRoom;
  short roomcnt[m_NumberOfRooms];
  memset(roomcnt,0,m_NumberOfRooms*sizeof(short));  
  short step=0;
  long bestpoly=-1;
  long poly=-1;
  double best=0;
  m_Threshold+=(1-m_ProbabilityItsHere);
  //m_Threshold/=m_ProbabilityItsHere;
  iterate(paths,step,bestpoly, best, poly,prevroom,roomcnt,
	  stepsInRoom,bestPaths,percentage);
  restore(restorePath);
 
  m_CurrentPolicy=restorepolicy;
  m_CurrentRoom=restoreroom;
 
  if (bestpoly==-1)return false; 
  //  if (getProb(bestpoly<m_Threshold))return false;
  getPolicy(bestpoly,pol);
  m_CurrentPolicy=bestpoly;
  return true;
}
bool AVSPolicyManager::next1StepPolicy(std::list<std::string> &pol, 
					    std::string &currentCfg,
					    double threshold,
					    double percentage) 
{
  update(currentCfg,percentage);
  m_Threshold = threshold;
  bool ok=false;
  for (int i=0;i<m_NumberOfRooms;i++)
    if (m_Rooms[i]->m_Probability>m_Threshold){
      ok=true;
      break;
    }
  if (!ok)return false;

  Cure::LongArray policies,best;
  policies.add(0,0);
  long n=m_PolicyConfigs.rows();
  double q,c,b;
  long p=-1;
  for (int i=0;i<(n);i++)
    if (getProb(i)>1E-15){   
      policies(0,0)=i;
      c=expectedCost(q,policies,percentage);
      if(q>=0) 
	if (p==-1){
	  b=c;
	  p=i;
	}else if(b>c){
	  p=i;
	      b=c;
	}
    }
  
  if (p==-1)return false; 
  getPolicy(p,pol);
  m_CurrentPolicy=p;
  return true;
}
bool AVSPolicyManager::next2StepPolicy(std::list<std::string> &pol, 
					    std::string &currentCfg,
					    double threshold,
					    double percentage) 
{
  update(currentCfg,percentage);
  m_Threshold = threshold;
  bool ok=false;
  for (int i=0;i<m_NumberOfRooms;i++){
    if (m_Rooms[i]->m_Probability>m_Threshold){
      ok=true;
      break;
    }
  }
  if (!ok)return false;
  Cure::LongArray policies,best;
  policies.add(0,0);
  policies.add(0,0);
  long n=m_PolicyConfigs.rows();
  double q,c,b;
  long p=-1;
  for (int i=0;i<(n);i++)
    if (getProb(i)>1E-15){   
      policies(0,0)=i;
     
 for (int j=0;j<n;j++)
	if (getProb(j)>1E-15){
	  policies(0,1)=j;
	  c=expectedCost(q,policies,percentage);
	  if(q>=0) 
	    if (p==-1){
	      b=c;
	      p=i;
	    }else if(b>c){
	      p=i;
	      b=c;
	    }
	}
    }
  if (p==-1)return false; 
  getPolicy(p,pol);
  m_CurrentPolicy=p;
  return true;
}
bool AVSPolicyManager::next3StepPolicy(std::list<std::string> &pol, 

					    std::string &currentCfg,
					    double threshold,
					    double percentage) 
{
  update(currentCfg,percentage);
  m_Threshold = threshold;
  bool ok=false;
  for (int i=0;i<m_NumberOfRooms;i++)
    if (m_Rooms[i]->m_Probability>m_Threshold){
      ok=true;
      break;
    }
  if (!ok)return false;
  Cure::LongArray policies,best;
  policies.add(0,0);
  policies.add(0,0);
  policies.add(0,0);
  long n=m_PolicyConfigs.rows();
  double q,c,b;
  long p=-1;
  for (int i=0;i<(n);i++)
    if (getProb(i)>1E-15){   
      policies(0,0)=i;
      for (int j=0;j<n;j++)
	if (getProb(j)>1E-15){
	  policies(0,1)=j;
	  for (int k=0;k<n;k++)
	    if (getProb(k)>1E-15){
	      policies(0,2)=k;
	      c=expectedCost(q,policies,percentage);
	      if(q>=0) 
		if (p==-1){
		  b=c;
		  p=i;
		}else if(b>c){
		  p=i;
		  b=c;
		}
	    }
	}
    }
  if (p==-1)return false; 
  getPolicy(p,pol);
  m_CurrentPolicy=p;
  return true;
}
bool AVSPolicyManager::next4StepPolicy(std::list<std::string> &pol, 
					    std::string &currentCfg,
					    double threshold,
					    double percentage) 
{
  update(currentCfg,percentage);
  m_Threshold = threshold;
  bool ok=false;
  for (int i=0;i<m_NumberOfRooms;i++)
    if (m_Rooms[i]->m_Probability>m_Threshold){
      ok=true;
      break;
    }
  if (!ok)return false;
  Cure::LongArray policies,best;
  policies.add(0,0);
  policies.add(0,0);
  policies.add(0,0);
  policies.add(0,0);
  long n=m_PolicyConfigs.rows();
  double q,c,b;
  long p=-1;
  for (int i=0;i<(n);i++)
    if (getProb(i)>1E-15){   
      policies(0,0)=i;
      for (int j=0;j<n;j++)
	if (getProb(j)>1E-15){
	  policies(0,1)=j;
	  for (int k=0;k<n;k++)
	    if (getProb(k)>1E-15){
	      policies(0,2)=k;
	  for (int h=0;h<n;h++)
	    if (getProb(h)>1E-15){
	      policies(0,3)=h;
	      c=expectedCost(q,policies,percentage);
	      if(q>=0) 
		if (p==-1){
		  b=c;
		  p=i;
		}else if(b>c){
		  p=i;
		  b=c;
		}
	    }
	    }
	}
    }
  if (p==-1)return false; 
  getPolicy(p,pol);
  m_CurrentPolicy=p;
  return true;
}
double AVSPolicyManager::expectedCost(double &q,Cure::LongArray policies,
				      double percentage, double threshold)
{
  PolicyPath restorePath;
  long restorepolicy=m_CurrentPolicy;
  long restoreroom=m_CurrentRoom;
  allocate(restorePath,1);
  save(restorePath);
  q=1;
  m_Threshold=threshold;
  double costs=0;
  short prevroom=m_CurrentRoom;
  bool ok=false;
  for (int i=0;i<policies.columns(0);i++){
    m_CurrentPolicy=policies(0,i);
    double c=m_Costs(m_CurrentPolicy,0);
    double prob=getProb(m_CurrentPolicy);
    if (prob<m_Threshold){
      ok=false;
      for (short room=0; room<m_NumberOfRooms;room++)
	if (m_Rooms[room]->m_Probability>m_Threshold){
	  ok=true;
	  break;
	}
      if (!ok)break;
    }
    if ((c>0)&&(prob>0)){
      prob*=percentage;
      c+=q*(prob*c/2.0+(1.0-prob)*c);
      update(percentage);
      c+=m_RoomCosts(m_CurrentRoom,prevroom);
      prevroom=m_CurrentRoom;
      costs+=c;
      q*=(1.0-prob);
      
    }else {
      costs=1E121;
      q=-1;
      restore(restorePath);
      m_CurrentPolicy=restorepolicy;
      m_CurrentRoom=restoreroom;
      return costs;
    }
    //    std::cerr<<q<<" ";

  }
  ok=false;
  for (short room=0; room<m_NumberOfRooms;room++)
    if (m_Rooms[room]->m_Probability>m_Threshold){
      ok=true;
      break;
    }
  if (q<1.0){
    if (ok)
      costs/=(1.0-q);
  }else if (policies.columns(0)==0){
    q=-1;
    restore(restorePath);
    m_CurrentPolicy=restorepolicy;
    m_CurrentRoom=restoreroom;
    return 1E21;
  }

  restore(restorePath);
  m_CurrentPolicy=restorepolicy;
  m_CurrentRoom=restoreroom;
  return costs;
}
void AVSPolicyManager::resetToPrior(){
  for (long i=0;i<m_NumberOfConfigs;i++)
    m_Configs[i].resetProbability();
  m_CurrentPolicy=-1;
  for (int i=0;i<m_Costs.Rows;i++){
    m_Costs(i,0)=m_Costs(i,1);
  }
  m_Credit=false;
}

void AVSPolicyManager::eliminate(long index, double percentage){
  double p=0;
  m_Configs[index].eliminate(p);
  p*=percentage;
  p=1.0-p;
  if (p==0)return;
  p=1.0/p;
  percentage=1.0-percentage;
  for (long i=0;i<m_NumberOfConfigs;i++){
    m_Configs[i].normalize(p,percentage);
  }
}
// This assumes that the currentPolicy failed at the last step.	
void AVSPolicyManager::update(double percentage) 
{ 
  long currentstep=-1;
  long bestrow=-1;
  if (m_CurrentPolicy>-1){//figure out where we are and what happened
    currentstep=m_PolicyConfigs.columns(m_CurrentPolicy)-1;
    long index=m_PolicyConfigs(m_CurrentPolicy, currentstep);
    eliminate(index, percentage);
    m_CurrentRoom=m_PolicyRooms(m_CurrentPolicy,0);
    currentstep--;
    while (currentstep>0){ 
      long currentindex=m_PolicyConfigs(m_CurrentPolicy,currentstep);
      double best=0;
      if (!m_Credit(0,currentindex)){
	m_Credit.setBit(0,currentindex,true);
	for (long i=0;i<m_Costs.Rows;i++){
	  long top=m_PolicyConfigs.columns(i);
	  for (int j=0;j<(top);j++){
	    if (m_PolicyConfigs(i,j)==currentindex){
	      long index=m_PolicyConfigs(i,top-1);
	      double prob=m_Configs[index].m_Probability;
	      double c=m_Costs(i,0);
	      if (c>0){
		c-=m_CostCredit;
		if (c>m_MinimumCost)m_Costs(i,0)=c;
		else m_Costs(i,0)=m_MinimumCost;
	      }
	    }
	  }
	}
      }
      currentstep--;    
    }
  }
}

void AVSPolicyManager::update(std::string &currentCfg,
			      double percentage) 
{ 
  long currentstep=-1;
  long bestrow=-1;
  if (m_CurrentPolicy>-1){//figure out where we are and what happened
    long col=m_PolicyConfigs.columns(m_CurrentPolicy);
    for (long j=0;j<col;j++){

      long index=m_PolicyConfigs(m_CurrentPolicy, j);
      if (currentCfg==m_Configs[index].m_Configuration){
	currentstep=j;
	eliminate(index, percentage);
	m_CurrentRoom=m_PolicyRooms(m_CurrentPolicy,0);
	break;
      }
    }      
    if (currentstep!=-1){
      // Here we know the policy row and step that was tried and
      // failed.  we can look for new policies that use the current
      // successfully found configuration as a starting point for
      // further searching and reduce their cost.
      
      currentstep--;
      //For now I just use a heuristic here but we should do better
      //after this experiment.  We can in fact compute the cost of each
      //step and keep track of all knowledge to reduce costs of looking for
      //stuff we have: allready found.  So some knowledge may be acquiered 
      //by chance.  Like we saw the table while looking for the bookcase.
      while (currentstep>0){ 
	//step 0 means stay in the same room we already credit that
	long currentindex=m_PolicyConfigs(m_CurrentPolicy,currentstep);
	double best=0;
	//We need to find the subset of Policies that include this
	// Configuration They will get a cost credit from having found
	// this config now.
	if (!m_Credit(0,currentindex)){
	  m_Credit.setBit(0,currentindex,true);
	  for (long i=0;i<m_Costs.Rows;i++){
	    long top=m_PolicyConfigs.columns(i);
	    for (int j=0;j<(top);j++){
	      if (m_PolicyConfigs(i,j)==currentindex){
		long index=m_PolicyConfigs(i,top-1);
		double prob=m_Configs[index].m_Probability;
		double c=m_Costs(i,0);
		if (c>0){
		  c-=m_CostCredit;
		  if (c>m_MinimumCost)m_Costs(i,0)=c;

		  else m_Costs(i,0)=m_MinimumCost;
		}
	      }
	    }
	  }
	}
	currentstep--;    
      }
    }else{
      long k=findConfig(currentCfg);
      if (k>-1){
	eliminate(k,percentage);
      }
    }
  }
}
bool AVSPolicyManager::nextGreedyRatioPolicy(std::list<std::string> &pol, 
				  std::string &currentCfg,
				  double threshold,
				  double percentage) 
{
  update(currentCfg,percentage);
  m_Threshold = threshold;
  bool ok=false;
  for (int i=0;i<m_NumberOfRooms;i++)
    if (m_Rooms[i]->m_Probability>m_Threshold){
      ok=true;
      break;
    }
  if (!ok)return false;
 // Here we just take the best prob/cost ratio of all the policies

  double best=0;
  long bestrow=-1;
  for (long i=0;i<m_Costs.Rows;i++){
    long col=m_PolicyConfigs.columns(i);
    long index=m_PolicyConfigs(i,col-1);
    double prob=m_Configs[index].m_Probability;
    double c=m_Costs(i,0);
    if (c>0){
      if (prob>1E-15){
	c+=m_RoomCosts(m_CurrentRoom,m_PolicyRooms(i,0));
	//prob/=c;
	c/=prob;
	if ((bestrow==-1)||(best>c)){
	  best=c;
	  bestrow=i;
	}
      }
    }
  }
  if (bestrow==-1)return false; 

  getPolicy(bestrow,pol);
  m_CurrentPolicy=bestrow;
  return true;
  //Now the m_CurrentPolicy should point use to the row for the next policy 
}

bool AVSPolicyManager::nextGreedyCostPolicy(std::list<std::string> &pol, 
				  std::string &currentCfg,
				  double threshold,
				  double percentage) 
{
  update(currentCfg,percentage);
  m_Threshold = threshold;
  bool ok=false;
  bool roomok[m_NumberOfRooms];
  for (int i=0;i<m_NumberOfRooms;i++)
    if (m_Rooms[i]->m_Probability>m_Threshold){
      ok=true;
      roomok[i]=true;
    }else roomok[i]=false;
  if (!ok)return false;
  // Here we just take the best prob/cost ratio of all the policies
  double best=0;
  long bestrow=-1;
  for (long i=0;i<m_Costs.Rows;i++){ 
    short room=m_PolicyRooms(i,0);
    if (roomok[room]){
      long col=m_PolicyConfigs.columns(i);
      long index=m_PolicyConfigs(i,col-1);
      double prob=m_Configs[index].m_Probability;
      double c=m_Costs(i,0);
      if (c>0){
	c*=(1-prob/2);
	c-=(prob*m_ExpectedCost);
	if (prob>1E-15){
	  c+=m_RoomCosts(m_CurrentRoom,room);
	  if ((bestrow==-1)||(best>c)){
	    best=c;
	    bestrow=i;
	  }
	}
      }
    }
  }
  if (bestrow==-1)return false; 
  getPolicy(bestrow,pol);
  m_CurrentPolicy=bestrow;
  return true;
  //Now the m_CurrentPolicy should point use to the row for the next policy 
}

void AVSPolicyManager::evaluatePolicies(double threshold1 ){
  std::list<std::string> pol;
  long top=m_PolicyConfigs.rows();
  m_Costs.reallocateZero(top,2);
  for (long row=0;row<top;row++){
    getPolicy(row,pol);
    long col=m_PolicyConfigs.columns(row);
    long index=m_PolicyConfigs(row,col-1);
    double prob=m_Configs[index].m_Probability;
    m_Costs(row,1)=-1;
    if (prob>threshold1){
      policyPrint(pol);
      double c=m_evalpol->GetStrategyCost(pol);
      printf("got cost %f \n", c);
      if (c>=0){
	m_Costs(row,1)=c;
      }
    }
    m_Costs(row,0)=m_Costs(row,1);
  }
  m_Costs.print();
}
void AVSPolicyManager::
getFileConfigurations(std::list<std::string> &configurations,
		      Cure::Matrix &probabilities,
		      std::string searchTarget){
  configurations.clear();
  Cure::ConfigFileReader cfg;
  m_SearchTarget=searchTarget;
  if (cfg.init(m_ConfigurationsFilename)) {
    std::cerr<<"AVSPolicyManager ERROR: Failed to open "
	     <<m_ConfigurationsFilename<<"\n";
    return;
  }
  std::string cmd="";
  std::list<std::string> strings;
  cfg.getParamStrings(m_SearchTarget,true,strings,cmd);
  getConfiguration(strings,configurations,probabilities);
}
void AVSPolicyManager::
getConfiguration(std::list<std::string> &strings,
		 std::list<std::string> &configurations,
		 Cure::Matrix &probabilities){
  probabilities.reallocateZero(strings.size(),1);
  long k=0;
  std::string cmd="";
  for (std::list<std::string>::iterator pi = strings.begin();
       pi != strings.end(); pi++,k++) {
    CureCERR(80)<<(*pi)<<"\n";
    std::istringstream sst(*pi);
    std::ostringstream ss;
    sst>>probabilities(k,0);
    int i=0;   
    while (sst>>cmd){
      if (i)ss<<" "<<cmd;
      else ss<<cmd;
      i=1;
    }
    cmd=ss.str();
    configurations.push_back(cmd);
  }
}


void AVSPolicyManager::
getFileRoomCosts(std::list<std::string> &rooms,
		 Cure::Matrix &costs){
  rooms.clear();
  Cure::ConfigFileReader cfg;
  if (cfg.init(m_RoomsFilename)) {
    std::cerr<<"AVSPolicyManager ERROR: Failed to open room file "
	     <<m_RoomsFilename<<"\n";
    return;
  }
  std::string cmd="";
  std::list<std::string> strings;
  cfg.getParamStrings("ROOMS",true,strings,cmd);
  costs.reallocateZero(strings.size());
  long k=0;
  CureCERR(80)<<"AVSPolicyManager getting Rooms\n";
  for (std::list<std::string>::iterator pi = strings.begin();
       pi != strings.end(); pi++,k++) {
    CureCERR(80)<<(*pi)<<"\n";
    std::istringstream sst(*pi);
    sst>>cmd;
    rooms.push_back(cmd);
    long j=0;
    double d;
    while (sst>>d){
      costs(k,j)=d;
      j++;
    }
  }
}

