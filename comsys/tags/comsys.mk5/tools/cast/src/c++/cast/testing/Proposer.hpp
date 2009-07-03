
#ifndef PROPOSER_HPP_
#define PROPOSER_HPP_

#include <cast/testing/AbstractTester.hpp>

namespace cast {

  class Proposer : 
    public ManagedProcess {

  public:
    Proposer(const std::string &_id);
    virtual ~Proposer();
  
    virtual void runComponent();
    virtual void configure(std::map<std::string,std::string> & _config);

  protected:
    virtual void taskAdopted(const std::string &_taskID);
    virtual void taskRejected(const std::string &_taskID);
  
  private:

    enum ExpectedResponse {
      ADOPTED, REJECTED, MIXED
    };

    int m_count;
    int m_expected;
    ExpectedResponse m_response;

    void randomSleep();

  };

} //namespace cast

#endif
