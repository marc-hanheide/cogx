#ifndef FAKE_MOTIVATION_SA_HPP_
#define FAKE_MOTIVATION_SA_HPP_

#include <string>

#include <cast/architecture.hpp>

class FakeMotivationSA : public cast::ManagedComponent {
private:
    std::string m_goal;
protected:
    virtual void runComponent();

public:
    FakeMotivationSA();

    virtual void configure(const cast::cdl::StringMap& _config, const Ice::Current& _current);
    virtual void planGenerated(const cast::cdl::WorkingMemoryChange& wmc);
};

#endif
