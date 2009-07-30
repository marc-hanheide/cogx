#ifndef FAKE_MOTIVATION_SA_HPP_
#define FAKE_MOTIVATION_SA_HPP_

#include <cast/architecture.hpp>

class FakeMotivationSA : public cast::ManagedComponent {
protected:
    virtual void runComponent();

public:
    virtual void planGenerated(const cast::cdl::WorkingMemoryChange& wmc);
};

#endif
