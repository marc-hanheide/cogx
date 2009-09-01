#ifndef FAKE_BINDER_SA_HPP_
#define FAKE_BINDER_SA_HPP_

#include <cast/architecture.hpp>

class FakeBinderSA : public cast::ManagedComponent {
protected:
    virtual void runComponent();
};

#endif
