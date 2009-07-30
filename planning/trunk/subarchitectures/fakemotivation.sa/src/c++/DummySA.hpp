#ifndef DUMMY_SA_HPP_
#define DUMMY_SA_HPP_

#include <cast/architecture.hpp>

class DummySA : public cast::ManagedComponent {
protected:
    virtual void runComponent();

public:
    virtual void planGenerated(const cast::cdl::WorkingMemoryChange& wmc);
};

#endif
