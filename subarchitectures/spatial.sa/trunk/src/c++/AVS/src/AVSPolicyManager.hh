#ifndef AVSPOLICYMANAGER
#define AVSPOLICYMANAGER

#include <sstream>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <list>
#include "Math/LongArray.hh"
#include "Math/Matrix.hh"
#include "Math/BinaryMatrix.hh"

class EvaluatePolicy {
public:
  virtual double GetStrategyCost(std::list<std::string> policy) {
    std::cout << "executing" << std::endl;
    return 0;
  }
};

class AVSPolicyManager {
  class Configuration {
  public:
    /**
     * The Configuration knows its location in the vector over all
     * configurations.
     */
    long m_Index;
    std::string m_Configuration;
    /** must be linked to all Configurations < this via subset links*/
    Configuration **m_Subsets;
    long m_NumberOfSubsets;
    /** Must contain all Configurations > this */
    Configuration **m_Supersets;
    long m_NumberOfSupersets;
    /** This is -1 if unknown */
    double m_Probability;
    double m_EliminatedProbability;
    double m_APrioriProbability;
    /** The index of the room which is right most token of the Configuration*/
    long m_Room;
    /** Is this a configuration that contains the searched for object */
    bool m_Final;
    Configuration() {
      init();
    }
    ~Configuration() {
      clear();
    }
    void init();
    void print();
    void operator =(Configuration &cfg);
    bool operator ==(Configuration &cfg) {
      return (m_Configuration == cfg.m_Configuration);
    }
    bool operator !=(Configuration &cfg) {
      return (m_Configuration != cfg.m_Configuration);
    }
    void setAPrioriProbability(double p) {
      m_APrioriProbability = p;
      m_Probability = p;
    }
    void resetProbability() {
      m_Probability = m_APrioriProbability;
    }
    void clear();
    /**
     * So this will iterate over all subsets with some double counting
     */
    Configuration * getSubset(long m);
    Configuration * subset(long &m);
    bool isSubset(Configuration &cfg);
    std::string getRoom() {
      if (m_NumberOfSupersets == 0)
        return m_Configuration;
      return m_Supersets[0]->getRoom();
    }
    double getRoomAPrioriProbability() {
      if (m_NumberOfSupersets == 0) {
        for (int i = 0; i < m_NumberOfSubsets; i++)
          if (m_Subsets[i]->m_Final) {
            return m_Subsets[i]->m_APrioriProbability;
          }
        return 0;
      }
      return m_Supersets[0]->getRoomAPrioriProbability();
    }

    double getRoomProbability() {
      if (m_NumberOfSupersets == 0) {
        for (int i = 0; i < m_NumberOfSubsets; i++)
          if (m_Subsets[i]->m_Final) {
            return m_Subsets[i]->m_Probability;
          }
        return 0;
      }
      return m_Supersets[0]->getRoomProbability();
    }
    Configuration * getRoomConfiguration() {
      if (m_NumberOfSupersets == 0) {
        for (int i = 0; i < m_NumberOfSubsets; i++)
          if (m_Subsets[i]->m_Final) {
            return m_Subsets[i];
          }
        return 0;
      }
      return m_Supersets[0]->getRoomConfiguration();
    }
    void setFinal(std::string &object);
    /**
     * @return true if cfg is a superset and is therefore added to m_Supersets.
     */
    bool addToSupersets(Configuration *cfg);
    /**
     * @return true if cfg is a subset and is therefore added to m_Subsets.
     */
    bool addToSubsets(Configuration *cfg);
    /**
     * This checks if cfg is a superset or subset of this and adds its
     * pointer to the corresponding list.
     */
    void add(Configuration *cfg) {
      if (addToSupersets(cfg))
        return;
      addToSubsets(cfg);
    }
    void pruneSubsets();
    void removeFromSubsets(int ind);
    /**
     * This is called when this configuration can be eliminated from
     * the possible configurations, That is it was search and the
     * object not found.
     *
     * @param p the total amount of eliminated probability is added to p
     * to allow normalization of all probabilities.   
     */

    void eliminate(double &p);
    void update(double p);
    void normalize(double p, double percentage);
    /**
     * A helper function for comparing configurations. 
     *
     * @return true if a is more general than b or equal to b.
     */
    bool moreOrEqualGeneral(std::string &a, std::string &b);
    bool lessOrEqualGeneral(std::string &a, std::string &b) {
      return moreOrEqualGeneral(b, a);
    }
    short size();
  };
  class PolicyPath {

  public:

    /** list of Policy indecies that constitute this path */
    long *m_Index;
    short m_NumberSteps;
    /** 
     * Stores the costs and probabilites needed to restore all
     * configurations to the state after this path
     */
    double *m_Element;
    long m_NumberProbs;
    long m_NumberCosts;

    /*
     *  This is P1 (cost of success at step 1) + (1-P1)(cost of fail at 1 +
     *  P2 (cost of success at 2) + (1-P2) (cost of fail at 2 +...
     *  PN (cost of success at N) + (1-PN)cost of fail at N)))...)
     *
     * EC_(N+1)=EC_N+Q_N*(PN(cost of success at N)+ (1-PN)(cost of fail at N))
     */
    double m_ExpectedCost;
    /*
     * This is (1-P1)(1-P2)...(1-PN).  The probability of fail after N steps.
     */
    double m_Q;
    Cure::LongArray m_Dependent;
    double *m_C;
    Cure::BinaryMatrix m_Credit;
    PolicyPath() {
      m_Index = 0;
      m_C = 0;
      m_Element = 0;
      m_ExpectedCost = 0;
      m_NumberCosts = 0;
      m_NumberProbs = 0;
      m_NumberSteps = 0;
      m_Q = 1;
    }
    ~PolicyPath() {
      if (m_Index)
        delete[] m_Index;
      if (m_Element)
        delete[] m_Element;
    }
    void allocate(long numofconfigs, long numofpolicies, int steps) {

      if (m_Index)
        delete[] m_Index;
      if (m_Element)
        delete[] m_Element;
      if ((numofconfigs) < 1)
        numofconfigs = 1;
      if (numofpolicies < 1)
        numofpolicies = 1;

      if (steps < 1)
        steps = 1;
      m_Element = new double[(numofconfigs + numofpolicies)];
      m_NumberProbs = numofconfigs;
      m_Credit.reallocate(1, numofconfigs, false);
      m_NumberCosts = numofpolicies;
      m_Index = new long[steps];
      m_NumberSteps = steps;
      m_C = m_Element + numofconfigs;
      for (int i = 0; i < steps; i++)
        m_Index[i] = -1;
    }
    void print();
    double expectedCost(double p, double cs, double cf) {
      return m_ExpectedCost + (m_Q * (p * cs + (1.0 - p) * cf));
    }
    void update(double p, double cs, double cf, long ind, short step) {
      m_Index[step] = ind;
      m_ExpectedCost += (m_Q * (p * cs + (1.0 - p) * cf));
      m_Q *= (1.0 - p);
    }
    void addDependent(long ind) {
      m_Dependent.add(0, ind);
    }
    bool contains(long ind) {
      return (m_Dependent.find(ind, 0) != -1);
    }

    double & costs(const long i) {
      return m_C[i];
    }
    double & probs(const long i) {
      return m_Element[i];
    }
    long & operator()(const int i) {
      return m_Index[i];
    }
    void operator +=(const double d) {
      m_ExpectedCost += d;
    }
    void operator -=(const double d) {
      m_ExpectedCost -= d;
    }
    void operator =(const double d) {
      m_ExpectedCost = d;
    }
    void operator =(PolicyPath &p) {
      m_ExpectedCost = p.m_ExpectedCost;
      m_Q = p.m_Q;
      m_Credit = p.m_Credit;
      long n = p.m_NumberSteps;
      if (n > m_NumberSteps) {
        if (m_Index)
          delete[] m_Index;
        m_Index = new long[n];
        m_NumberSteps = n;
      }
      memcpy(m_Index, p.m_Index, n * sizeof(long));
      for (int i = n; i < m_NumberSteps; i++)
        m_Index[i] = -1;
      n = p.m_NumberProbs + p.m_NumberCosts;
      if ((m_NumberProbs != p.m_NumberProbs) || (m_NumberCosts
          != p.m_NumberCosts)) {
        if (n > (m_NumberProbs + m_NumberCosts)) {
          if (m_Element)
            delete[] m_Element;
          m_Element = new double[n];
        }
        m_NumberProbs = p.m_NumberProbs;
        m_NumberCosts = p.m_NumberCosts;
        m_C = m_Element + m_NumberProbs;
      }
      memcpy(m_Element, p.m_Element, n * sizeof(double));
      m_Dependent = p.m_Dependent;
    }
    double operator ()(void) {
      return m_ExpectedCost;
    }
  };

  /* Actually an element of a list of policies*/
  class Policy {
  public:
    /** A list of configurations */
    std::list<std::string> m_Policy;
    Policy *m_Next;
    std::string m_Room;
    Policy() {
      m_Next = 0;
    }
    ~Policy() {
      clear();
    }
    void clear() {
      if (m_Next)
        delete m_Next;
      m_Next = 0;
    }
    void add(std::string &cfg) {
      if (m_Next) {
        m_Policy.push_back(cfg);
      }
    }
    Policy * addNew(std::string &cfg, bool room = false) {
      if (m_Next)
        return m_Next->addNew(cfg);
      m_Next = new Policy();
      m_Policy.push_back(cfg);
      if (room)
        m_Room = cfg;
    }
    void print();
  };

  class PolicyTree {
    /**
     * The purpose of the tree is to list all policies ending in a given 
     * configuration.  So it forms a tree where the mother contains the 
     * given configuration and the leaves has configuration of some room.
     *
     */
  public:

    /** Search moves to mother after this policy step.*/
    PolicyTree *m_Mother;
    /** Search was done on some child to get here.*/
    PolicyTree **m_Children;

    short m_NumberOfChildren;
    /** This is the configuration of this node of the tree.*/
    std::string m_Configuration;
    short m_Size;
    PolicyTree() {
      m_Mother = 0;
      m_Children = 0;
      m_Size = 0;
      m_NumberOfChildren = 0;
    }
    PolicyTree(PolicyTree *t) {
      m_Mother = t;
      m_Children = 0;
      m_Size = 0;
      m_NumberOfChildren = 0;
    }

    PolicyTree(std::string &tmp) {
      m_Mother = 0;
      m_Children = 0;
      m_NumberOfChildren = 0;
      m_Size = 0;
      setConfiguration(tmp);
    }
    ~PolicyTree() {
      clearChildren();
    }
    void setConfiguration(std::string tmp);
    void clearChildren() {
      if (m_NumberOfChildren == 0)
        return;
      for (short i = 0; i < m_NumberOfChildren; i++) {
        delete m_Children[i];
      }
      delete[] m_Children;
      m_Children = 0;
      m_NumberOfChildren = 0;
    }
    /**
     * This is the function that finally forms the list of all
     * policies to the mother node.
     */
    void listPolicies(Policy *policies);
    /**
     * This checks configuration and more to see if all subsets of the
     * configurations that include the room and the search target are
     * on one of the lists.  If not the missing strings are added to
     * more.
     */
    void completeList(std::list<std::string> &configurations, std::list<
        std::string> &more);
  };
  //START OF THE ACTUAL MANAGER OBJECT ITSELF
public:
  EvaluatePolicy* m_evalpol;
  /**
   * Defines the config index.
   *
   * This list includes all of final search configurations plus all
   * intermeadiate configuration that may be searched for by some
   * policy.  The final configurations have a Probability >=0 and
   * m_Final=true.  
   */
  Configuration *m_Configs;
  long m_NumberOfConfigs;
  std::string m_SearchTarget;
  /**
   * Depend index to config index.
   *
   * Defines the depend index which has a contex of a particular room.
   *
   * Each room i, has a set of configs that are supersets of two or
   * more final search configurations with Probabilities >0 and which
   * are either disjoint or have unequal probabilities.  The indecies
   * of this set of Configs are listed in row i of this.
   */
  Cure::LongArray m_Depend;
  /** 
   * Defines the policies and the policy index
   *
   * Rows are the Policy while Columns are the steps of the policy.
   * So (i,j) is the Index into m_Configs of the Configuration for
   * setp j of the ith Policy search.  This is set when listPolicies
   * is called.  So a Policy ends up as a vector of indecies.
   */
  Cure::LongArray m_PolicyConfigs;
  /** 
   * Go from a policy index to a room index.
   *
   * Each Policy of row i of m_PolicyConfigs has a room it searches.
   * The index of that room is contained in m_PolicyRooms(i,0).
   */
  Cure::LongArray m_PolicyRooms;

  /** 
   * Go from a room index to a policy index.
   *
   * The index of all policies in room i are contained in m_RoomPolicy(i,j).
   */
  Cure::LongArray m_RoomPolicy;
  /**
   * The a priori probabilities that the search object is in the room
   * is P_r = m_RoomProbability(0,room).
   * The stoping criteria for room is m_RoomProbability(1,room)<Q_ni.
   *
   * So m_RoomProbability(1,room)= (1.0 - gamma_r * P_r),
   * where gamma_r is the limit_(n -> infinity) gamma_rn 
   *
   * gamma_r(n+1)= 1 if gamma_rn were 1
   *             = max(gamma_n *alpha/sum_r(gamma_rn*P_r),1.0) otherwise  
   *
   * gamma_r0=1.0
   * alpha=(1-threshold)*m_ProbabilityItsHere;
   *
   * The Stopping criteria for room to room search is:
   * m_Threshold = (1.0-alpha)<Q_ni
   */
  Cure::Matrix m_RoomProbability;
  /**
   * m_ProbabilityItsHere = sum_r(m_RoomProbability(0,r))
   */
  double m_ProbabilityItsHere;
  double m_Threshold;
  /**
   * Policy index to list of depend indecies.
   *
   * Each Policy with index i may contain a search for the
   * configuration: m_Configs[m_Depend(m_PolicyRooms(i,0),j)].
   *  If it does then j is listed on row i of this.
   */
  Cure::LongArray m_PolicyDepend;
  /**
   * room/depend index to list of policy indecies
   *
   * For each m_Depend(room,j) m_DependPolicies[room] row j contains all the
   * rows from m_PolicyConfigs whose policy contains m_Depend(i,j).  
   */
  Cure::LongArray *m_DependPolicies;
  /** 
   * Policy index to cost.
   *
   * The cost of the ith policy is in m_Costs(i,0) 
   * The a priori cost of the ith policy is in m_Costs(i,1) 
   * (writting this now there is no difference but we may want to update costs)
   */
  Cure::Matrix m_Costs;
  /**
   * room to room costs matrix
   */
  Cure::Matrix m_RoomCosts;
  Configuration **m_Rooms;
  std::string m_ConfigurationsFilename;
  std::string m_RoomsFilename;
  short m_NumberOfRooms;
  /**
   * The row of m_PolicyConfigs for the current best policy.
   */
  long m_CurrentPolicy;
  /** This sets the maximum change in configuration length between steps */
  short m_MaxVaribles;
  /** Where we are now. Initialized to 0*/
  long m_CurrentRoom;
  /** 
   * This uniform credit will be given to all policies that have some step
   * skipped due to aquired knowledge (ie. we already found the table...).
   * Initial value is 1.0.
   */
  double m_CostCredit;
  Cure::BinaryMatrix m_Credit;
  double m_MinimumCost;
  /**
   * Sets how far ahead to look in the decsion tree to find the best policy.
   * The default is 10.
   */
  long m_MaxSteps;
  /**
   * For the GreedyCostPolicy this is the reward for finding the object.
   * For type 1 costs this is the constant expected cost remaining
   * after each step.  The default is 10.
   *
   */
  double m_ExpectedCost;
  /**
   * <cost| policy i>=C_ni + Q_ni * R_ni
   *
   * C_ni= C_(n-1)i + Q_(n-1)i[(cost of success)_ni * p_ni +
   *       (cost of fail)_ni * (1-p_ni)]
   *
   * Q_ni=Q_(n-1)i*(1-p_ni)
   *
   * R_ni=<cost | fail after step n with policy path i>  
   * Type 1: R_ni=m_ExpectedCost (a constant parameter)
   * Type 2: R_ni=C_ni/(1-Q_ni) (a constant estimated from i's nth step)
   * Type 4: R_ni=C_ni/(1-(Q_ni)²) (uniform dist. est. from i's nth step)
   */
  short m_Type;
  /**
   * The AVSPoliciyManager will load all the possible configurations
   * for the object search along with thier probabilities from the
   * Bayes Net.  It will then generate all policies for configurations above a
   * threshold probability.  
   *
   * It then evaluates the policies and choses the best one calling a
   * "coneGenerator" on each policy in turn to generate the costs for each.
   *
   * So this object needs a BN that generates alist of strings in the
   * form X IN/ON B IN/ON C ... IN RoomY along with probabilities.
   *
   * It then needs a ConeGenerator that takes a list of strings where
   * each strin is a configuration of a single step of a search such
   * as B IN RoomY and generates a cost for the search.
   *
   * There are only two functions to call here: 
   *
   * preCompute(double threshold); and
   * 
   * nextGreedyPolicy(std::list<std::string> &pol, std::string &currentCfg,
   *	         double threshold);
   *
   */
  AVSPolicyManager();
  ~AVSPolicyManager() {
    clearConfigs();
    delete[] m_DependPolicies;
  }
  void setCostCredit(double d) {
    m_CostCredit = d;
  }
  void setCurrentRoom(long r) {
    m_CurrentRoom = r;
  }
  void setConfigurationsFilename(std::string s) {
    m_ConfigurationsFilename = s;
  }
  void setRoomsFilename(std::string s) {
    m_RoomsFilename = s;
  }
  /**
   * This funcitonnmust be called first after creating the
   *  AVSPolicyManager.  It initializes the policies by find all
   *  policies and thier probabilities and costs.
   *
   * @param searchTarget the name of the object to find.
   *
   * @param threshold if the probability of the policy final configuration is 
   * above this value then the ratio of probability to cost will be used to
   * select the best policy.
   */
  void preCompute(std::string searchTarget, double threshold = 0) {
    listPolicies(searchTarget);
    evaluatePolicies(threshold);
  }
  /**
   * Same as preCompute but this expects to be given a list of
   * probabiities and configurations that are complete.  That is
   * all supersets are given.  Also the probabities should be complete  
   * and consistent so P(cup IN Room)>= P(cup ON Table IN Room) and so on.
   * If P(cup IN Room) < P(cup ON Table IN Room) + P(cup IN Box IN Room)
   * Then there needs to be a P(Cup IN Box ON Table IN Room) and so on
   *
   * @param strings this is a list of strings where each string is a
   * probability as a double between 0 and 1 followed by the configuration
   * string such as: 
   *
   * 0.3466 Cup ON Table IN Room1
   *
   * Keywords IN and ON must be in capitals. 
   *
   */
  void preCompute(std::list<std::string> &strings, double threshold = 0) {
    Cure::Matrix probabilities;
    std::list<std::string> configurations;
    getConfiguration(strings, configurations, probabilities);
    configure(configurations, probabilities);
    evaluatePolicies(threshold);
  }
  /**
   * Same as preCompute but this expects to be able to find a list of
   * probabiities and configurations that are mutually exclusive.
   * The missing configurations and all probabilities are then infered
   * from this list.
   */
  void preComputeExclusive(std::string searchTarget, double threshold = 0) {
    listExclusivePolicies(searchTarget);
    evaluatePolicies(threshold);
  }
  /**
   * Same as preComputeExclusive but this expects to be given a list of
   * probabiities and configurations that are mutually exclusive.
   * The missing configurations and all probabilities are then infered
   * from this list.
   *
   * @param strings this is a list of strings where each string is a
   * probability as a double between 0 and 1 followed by the configuration
   * string such as: 
   *
   * 0.3466 Cup ON Table IN Room1
   *
   * Keywords IN and ON must be in capitals. 
   *
   */
  void preComputeExclusive(std::list<std::string> &strings, double threshold =
      0) {
    Cure::Matrix probabilities;
    std::list<std::string> configurations;
    getConfiguration(strings, configurations, probabilities);
    configureExclusive(configurations, probabilities);
    evaluatePolicies(threshold);
  }
  double evaluate(std::list<std::string> &policy) {
    //return m_ConeGenerator->evaluate(policy)
    //IMPLEMENTATION NEEDED#############################################################################
    return 10.0 / ((policy.size()) * (policy.size()));
  }
  /**
   * This is called to start a whole new search for the same object 
   * skiping the preCompute.
   */
  void resetToPrior();
  /**
   * This is called after evaluatePolicies(..) and it is expected that
   * a policy has been carried out and failed at some step
   * corresponding to the string in currentCfg.  The string in
   * currentCfg should be one of the strings returned in policy when
   * calling this last time.  The probabilities of the various
   * configurations are updated based on assuming the subsets of the
   * currentCfg will all now have zero probability.  The costs are 
   * changed also. 
   * 
   * The best policy in terms of the ratio of probabiity to cost will
   * be found.  If none are above the probability threshold it returns false.
   *
   * @param policy the best policy is returned as a list of strings.  The
   * first string is allways just the room name.  It may contain one
   * of the recent configurations of the robot search.  The caller
   * needs to remember for instance which room and what objects where
   * already found in it.  This allows the policy to start executing
   * from a point beyond step 0.
   *
   * A policy is a list of strings. 
   *
   * The list contains the configurations for the policy steps in
   * order.  So each list starts with finding a room (ie configuration
   * = "RoomA" Then continues thru all intermeadiate search
   * configurations ending with a configuration = ObjectX ... IN
   * RoomA.  The shortest policy has two steps, {room, object IN
   * room}
   *
   * 
   *
   * @param currenCfg the configuration that failed from the
   * previously executed policy.   An empty string it this is the first call.
   *
   * @param threshold if the probability of the policy final configuration is 
   * above this value then the ratio of probability to cost will be used to
   * select the best policy.
   *
   * @param p This is the fraction of the currentCfg that should be
   * eliminated, in the range [0,1].
   * 
   * @return true if a policy was found else false
   */
  bool nextGreedyRatioPolicy(std::list<std::string> &policy,
      std::string &currentCfg, double threshold, double p = 1.0);
  /**
   * This is the same as nextGreedyRatioPolicy except that the
   * expected increment to a cost accumulation is minimized instead of
   * the ratio of cost to probability.  This includes an expected
   * m_Reward term as well.
   *
   */
  bool nextGreedyCostPolicy(std::list<std::string> &pol,
      std::string &currentCfg, double threshold, double percentage);
  bool nextNearlyBestPolicy(std::list<std::string> &pol,
      std::string &currentCfg, double threshold, double percentage);
  bool next1StepPolicy(std::list<std::string> &pol, std::string &currentCfg,
      double threshold, double percentage);
  bool next2StepPolicy(std::list<std::string> &pol, std::string &currentCfg,
      double threshold, double percentage);
  bool next3StepPolicy(std::list<std::string> &pol, std::string &currentCfg,
      double threshold, double percentage);
  bool next4StepPolicy(std::list<std::string> &pol, std::string &currentCfg,
      double threshold, double percentage);

  bool multiRoomSearch(std::list<std::string> &pol, short *stepsInRoom,
      PolicyPath *bestPaths, PolicyPath &restorePath, double percentage = .99);
  void iterate(PolicyPath *paths, short step, long &bestpol, double &best,
      long &poly, short *prevroom, short *roomcnt, short *stepsInRoom,
      PolicyPath *bestPaths, double percentage = .99);

  void update(std::string &currentCfg, double percentage);
  void update(double percentage);

  double expectedCost(double &q, Cure::LongArray policies, double percentage =
      .99, double threshold = .01);
  /**
   * ALL methods below this point are only used internal to this
   * object and not called from outside.
   */

  void makeDepend();
  void clearConfigs();
  void addToConfigs(std::string str);
  /** 
   * This adds the string if it is not found 
   *
   * @return the index of m_Configs that holds the string str.
   */
  long getConfig(std::string &str);
  /** 
   * This returns -1 if it is not found 
   * @return the index of m_Configs that holds the string str, or -1.
   */
  long findConfig(std::string &str);
  /**
   * This pushes the strings of the policy on row of m_PolicyConfigs into pol.
   *
   *
   */
  void getPolicy(long row, std::list<std::string> &pol);

  /** 
   *
   *
   * Here we need a way to get all configurations and their probablities
   *
   *  
   * @probabilities Each row gives the probability of the
   * cooresponding configuration of configurations. We only need the
   * probabilities for configurations that have the object in them.
   * Again these end up being transfered to the m_Configs elements
   * where they are modified by the search results.
   *
   *
   *
   * @param configurations This reuturns a list of all the compatable
   * configurations for the current object(s) being searched for.  So
   * the elements look like Cup IN BOX ON Table IN RoomA.  This is
   * just the starting point so what the Bayes Net gives us before we
   * setup our own data structures.  Each of these correspondes to a
   * "leaf" m_Config.  So a final search configuration for the object.
   */
  void getConfigurations(std::list<std::string> &configurations,
      Cure::Matrix &probabilities, std::string searchTarget);
  void getExclusiveConfigurations(std::list<std::string> &configurations,
      Cure::Matrix &probabilities, std::string searchTarget);
  void getConfiguration(std::list<std::string> &strings,
      std::list<std::string> &configurations, Cure::Matrix &probabilities);
  void configureExclusive(std::list<std::string> &configurations,
      Cure::Matrix &probabilities);
  void configure(std::list<std::string> &configurations,
      Cure::Matrix &probabilities);

  void getFileConfigurations(std::list<std::string> &configurations,
      std::string searchTarget);
  void getFileConfigurations(std::list<std::string> &configurations,
      Cure::Matrix &probabilities, std::string searchTarget);
  void getFileRoomCosts(std::list<std::string> &rooms, Cure::Matrix &costs);
  void addProbabilities(Cure::Matrix &probabilities,
      std::list<std::string> &more);
  /** called to update the probabilities after a failed policy search*/
  void eliminate(long index, double percentage = 1.0);
  /**
   *  This sets up the list of all possible policies
   *  and puts it in policies.
   */
  void listPolicies(std::string searchTarget);
  /**
   *  This sets up the list of all possible policies and puts it in
   *  policies.  It is called if the list of configurations and
   *  probabilities will be given in mutually exclusive format.  It
   *  then infers the missing configurations and the probabilities of
   *  overlapping ones.
   */
  void listExclusivePolicies(std::string searchTarget);
  /** 
   * This is called to evaluate the cost of the policies after
   *  creating the intial listPolicies(..).
   *
   * @param policies the returned list from listPolicies
   * 
   *
   * @param threshold1 if the probability of the policy's final
   *   configuration is above this value then the cost of the policy
   *   will be computed.  
   *
   */
  void evaluatePolicies(double threshold1 = 0);
  void save(PolicyPath &p) {
    for (long i = 0; i < m_NumberOfConfigs; i++) {
      p.probs(i) = m_Configs[i].m_Probability;
    }
    for (long i = 0; i < m_Costs.Rows; i++)
      p.costs(i) = m_Costs(i, 0);
    p.m_Credit = m_Credit;
  }
  void restore(PolicyPath &p) {
    for (long i = 0; i < m_NumberOfConfigs; i++)
      m_Configs[i].m_Probability = p.probs(i);
    for (long i = 0; i < m_Costs.Rows; i++)
      m_Costs(i, 0) = p.costs(i);
    m_Credit = p.m_Credit;
  }
  void allocate(PolicyPath &p, int steps) {
    p.allocate(m_NumberOfConfigs, m_Costs.Rows, steps);
  }
  double getProb(long poly) {
    long index = m_PolicyConfigs(poly, m_PolicyConfigs.columns(poly) - 1);
    return m_Configs[index].m_Probability;
  }
  double getProb(PolicyPath &p, long poly) {
    long index = m_PolicyConfigs(poly, m_PolicyConfigs.columns(poly) - 1);
    return p.probs(index);
  }
  //  double getRoomProb(PolicyPath &p,short room){
  // return p.probs(m_Rooms[room]->m_Index); 
  //}
  //  double getRoomProb(int room){
  //return m_Rooms[room]->m_Probability;
  //}
};

#endif

