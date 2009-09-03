#ifndef PLANNER_CLIENT_HH
#define PLANNER_CLIENT_HH


#include "CAST_SCAT/cast_scat.hh"

/* Header is automatically generated by ICE (Internet Communications
 * Engine -- ZeroC, Inc.) package \program{slice2cpp}.*/
#include "PCogX.hpp"

#ifndef PLANNER_FACTORY_DESIGNATION
#define PLANNER_FACTORY_DESIGNATION ""
#endif

#ifndef PLANNER_CLIENT_DESIGNATION
#define PLANNER_CLIENT_DESIGNATION ""
#endif

using CAST_SCAT::Designator;
using CAST_SCAT::Designators;

class Planner_Client :
    public CAST_SCAT::procedure_implementation<Planner_Client>,
    public CAST_SCAT::procedure_call<>
{
public:
    typedef CAST_SCAT::procedure_implementation<Planner_Client> Implement;
    typedef CAST_SCAT::procedure_call<> Call;
    
    explicit Planner_Client(Designator&& name = PLANNER_CLIENT_DESIGNATION);
    
    void runComponent();
protected:
    void start();
};


#endif
