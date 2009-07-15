# **********************************************************************
#
# Copyright (c) 2003-2008 ZeroC, Inc. All rights reserved.
#
# This copy of Ice is licensed to you under the terms described in the
# ICE_LICENSE file included in this distribution.
#
# **********************************************************************

# Ice version 3.3.0
# Generated from file `Planner.ice'

import Ice, IcePy, __builtin__
import CDL_ice

# Included module cast
_M_cast = Ice.openModule('cast')

# Included module cast.cdl
_M_cast.cdl = Ice.openModule('cast.cdl')

# Included module cast.cdl.testing
_M_cast.cdl.testing = Ice.openModule('cast.cdl.testing')

# Included module cast.interfaces
_M_cast.interfaces = Ice.openModule('cast.interfaces')

# Included module cast.examples
_M_cast.examples = Ice.openModule('cast.examples')

# Included module cast.examples.autogen
_M_cast.examples.autogen = Ice.openModule('cast.examples.autogen')

# Start of module autogen
_M_autogen = Ice.openModule('autogen')
__name__ = 'autogen'

# Start of module autogen.Planner
_M_autogen.Planner = Ice.openModule('autogen.Planner')
__name__ = 'autogen.Planner'

if not _M_autogen.Planner.__dict__.has_key('_t_stringSeq'):
    _M_autogen.Planner._t_stringSeq = IcePy.defineSequence('::autogen::Planner::stringSeq', (), IcePy._t_string)

if not _M_autogen.Planner.__dict__.has_key('ObjectDeclaration'):
    _M_autogen.Planner.ObjectDeclaration = Ice.createTempClass()
    class ObjectDeclaration(Ice.Object):
        def __init__(self, name='', type=''):
            self.name = name
            self.type = type

        def ice_ids(self, current=None):
            return ('::Ice::Object', '::autogen::Planner::ObjectDeclaration')

        def ice_id(self, current=None):
            return '::autogen::Planner::ObjectDeclaration'

        def ice_staticId():
            return '::autogen::Planner::ObjectDeclaration'
        ice_staticId = staticmethod(ice_staticId)

        def __str__(self):
            return IcePy.stringify(self, _M_autogen.Planner._t_ObjectDeclaration)

        __repr__ = __str__

    _M_autogen.Planner.ObjectDeclarationPrx = Ice.createTempClass()
    class ObjectDeclarationPrx(Ice.ObjectPrx):

        def checkedCast(proxy, facetOrCtx=None, _ctx=None):
            return _M_autogen.Planner.ObjectDeclarationPrx.ice_checkedCast(proxy, '::autogen::Planner::ObjectDeclaration', facetOrCtx, _ctx)
        checkedCast = staticmethod(checkedCast)

        def uncheckedCast(proxy, facet=None):
            return _M_autogen.Planner.ObjectDeclarationPrx.ice_uncheckedCast(proxy, facet)
        uncheckedCast = staticmethod(uncheckedCast)

    _M_autogen.Planner._t_ObjectDeclarationPrx = IcePy.defineProxy('::autogen::Planner::ObjectDeclaration', ObjectDeclarationPrx)

    _M_autogen.Planner._t_ObjectDeclaration = IcePy.defineClass('::autogen::Planner::ObjectDeclaration', ObjectDeclaration, (), False, None, (), (
        ('name', (), IcePy._t_string),
        ('type', (), IcePy._t_string)
    ))
    ObjectDeclaration.ice_type = _M_autogen.Planner._t_ObjectDeclaration

    _M_autogen.Planner.ObjectDeclaration = ObjectDeclaration
    del ObjectDeclaration

    _M_autogen.Planner.ObjectDeclarationPrx = ObjectDeclarationPrx
    del ObjectDeclarationPrx

if not _M_autogen.Planner.__dict__.has_key('_t_objDeclSeq'):
    _M_autogen.Planner._t_objDeclSeq = IcePy.defineSequence('::autogen::Planner::objDeclSeq', (), _M_autogen.Planner._t_ObjectDeclaration)

if not _M_autogen.Planner.__dict__.has_key('ModalityEnum'):
    _M_autogen.Planner.ModalityEnum = Ice.createTempClass()
    class ModalityEnum(object):

        def __init__(self, val):
            assert(val >= 0 and val < 3)
            self.value = val

        def __str__(self):
            if self.value == 0:
                return 'FACTMOD'
            elif self.value == 1:
                return 'BELIEFMOD'
            elif self.value == 2:
                return 'KVALMOD'
            return None

        __repr__ = __str__

        def __hash__(self):
            return self.value

        def __cmp__(self, other):
            return cmp(self.value, other.value)

    ModalityEnum.FACTMOD = ModalityEnum(0)
    ModalityEnum.BELIEFMOD = ModalityEnum(1)
    ModalityEnum.KVALMOD = ModalityEnum(2)

    _M_autogen.Planner._t_ModalityEnum = IcePy.defineEnum('::autogen::Planner::ModalityEnum', ModalityEnum, (), (ModalityEnum.FACTMOD, ModalityEnum.BELIEFMOD, ModalityEnum.KVALMOD))

    _M_autogen.Planner.ModalityEnum = ModalityEnum
    del ModalityEnum

if not _M_autogen.Planner.__dict__.has_key('Fact'):
    _M_autogen.Planner.Fact = Ice.createTempClass()
    class Fact(Ice.Object):
        def __init__(self, modality=_M_autogen.Planner.ModalityEnum.FACTMOD, believers=None, name='', arguments=None, value=''):
            self.modality = modality
            self.believers = believers
            self.name = name
            self.arguments = arguments
            self.value = value

        def ice_ids(self, current=None):
            return ('::Ice::Object', '::autogen::Planner::Fact')

        def ice_id(self, current=None):
            return '::autogen::Planner::Fact'

        def ice_staticId():
            return '::autogen::Planner::Fact'
        ice_staticId = staticmethod(ice_staticId)

        def __str__(self):
            return IcePy.stringify(self, _M_autogen.Planner._t_Fact)

        __repr__ = __str__

    _M_autogen.Planner.FactPrx = Ice.createTempClass()
    class FactPrx(Ice.ObjectPrx):

        def checkedCast(proxy, facetOrCtx=None, _ctx=None):
            return _M_autogen.Planner.FactPrx.ice_checkedCast(proxy, '::autogen::Planner::Fact', facetOrCtx, _ctx)
        checkedCast = staticmethod(checkedCast)

        def uncheckedCast(proxy, facet=None):
            return _M_autogen.Planner.FactPrx.ice_uncheckedCast(proxy, facet)
        uncheckedCast = staticmethod(uncheckedCast)

    _M_autogen.Planner._t_FactPrx = IcePy.defineProxy('::autogen::Planner::Fact', FactPrx)

    _M_autogen.Planner._t_Fact = IcePy.defineClass('::autogen::Planner::Fact', Fact, (), False, None, (), (
        ('modality', (), _M_autogen.Planner._t_ModalityEnum),
        ('believers', (), _M_autogen.Planner._t_stringSeq),
        ('name', (), IcePy._t_string),
        ('arguments', (), _M_autogen.Planner._t_stringSeq),
        ('value', (), IcePy._t_string)
    ))
    Fact.ice_type = _M_autogen.Planner._t_Fact

    _M_autogen.Planner.Fact = Fact
    del Fact

    _M_autogen.Planner.FactPrx = FactPrx
    del FactPrx

if not _M_autogen.Planner.__dict__.has_key('_t_factSeq'):
    _M_autogen.Planner._t_factSeq = IcePy.defineSequence('::autogen::Planner::factSeq', (), _M_autogen.Planner._t_Fact)

if not _M_autogen.Planner.__dict__.has_key('PlanningState'):
    _M_autogen.Planner.PlanningState = Ice.createTempClass()
    class PlanningState(Ice.Object):
        def __init__(self, facts=None):
            self.facts = facts

        def ice_ids(self, current=None):
            return ('::Ice::Object', '::autogen::Planner::PlanningState')

        def ice_id(self, current=None):
            return '::autogen::Planner::PlanningState'

        def ice_staticId():
            return '::autogen::Planner::PlanningState'
        ice_staticId = staticmethod(ice_staticId)

        def __str__(self):
            return IcePy.stringify(self, _M_autogen.Planner._t_PlanningState)

        __repr__ = __str__

    _M_autogen.Planner.PlanningStatePrx = Ice.createTempClass()
    class PlanningStatePrx(Ice.ObjectPrx):

        def checkedCast(proxy, facetOrCtx=None, _ctx=None):
            return _M_autogen.Planner.PlanningStatePrx.ice_checkedCast(proxy, '::autogen::Planner::PlanningState', facetOrCtx, _ctx)
        checkedCast = staticmethod(checkedCast)

        def uncheckedCast(proxy, facet=None):
            return _M_autogen.Planner.PlanningStatePrx.ice_uncheckedCast(proxy, facet)
        uncheckedCast = staticmethod(uncheckedCast)

    _M_autogen.Planner._t_PlanningStatePrx = IcePy.defineProxy('::autogen::Planner::PlanningState', PlanningStatePrx)

    _M_autogen.Planner._t_PlanningState = IcePy.declareClass('::autogen::Planner::PlanningState')

    _M_autogen.Planner._t_PlanningState = IcePy.defineClass('::autogen::Planner::PlanningState', PlanningState, (), False, None, (), (('facts', (), _M_autogen.Planner._t_factSeq),))
    PlanningState.ice_type = _M_autogen.Planner._t_PlanningState

    _M_autogen.Planner.PlanningState = PlanningState
    del PlanningState

    _M_autogen.Planner.PlanningStatePrx = PlanningStatePrx
    del PlanningStatePrx

if not _M_autogen.Planner.__dict__.has_key('PlanningTask'):
    _M_autogen.Planner.PlanningTask = Ice.createTempClass()
    class PlanningTask(Ice.Object):
        def __init__(self, id=0, planningAgent='', objects=None, state=None, goal=''):
            if __builtin__.type(self) == _M_autogen.Planner.PlanningTask:
                raise RuntimeError('autogen.Planner.PlanningTask is an abstract class')
            self.id = id
            self.planningAgent = planningAgent
            self.objects = objects
            self.state = state
            self.goal = goal

        def ice_ids(self, current=None):
            return ('::Ice::Object', '::autogen::Planner::PlanningTask')

        def ice_id(self, current=None):
            return '::autogen::Planner::PlanningTask'

        def ice_staticId():
            return '::autogen::Planner::PlanningTask'
        ice_staticId = staticmethod(ice_staticId)

        #
        # Operation signatures.
        #
        # def loadMAPLTask(self, taskFile, domainFile, planningAgent, current=None):
        # def markChanged(self, current=None):
        # def activateChangeDetection(self, current=None):
        # def planAvailable(self, current=None):
        # def getPlan(self, current=None):

        def __str__(self):
            return IcePy.stringify(self, _M_autogen.Planner._t_PlanningTask)

        __repr__ = __str__

    _M_autogen.Planner.PlanningTaskPrx = Ice.createTempClass()
    class PlanningTaskPrx(Ice.ObjectPrx):

        def loadMAPLTask(self, taskFile, domainFile, planningAgent, _ctx=None):
            return _M_autogen.Planner.PlanningTask._op_loadMAPLTask.invoke(self, ((taskFile, domainFile, planningAgent), _ctx))

        def markChanged(self, _ctx=None):
            return _M_autogen.Planner.PlanningTask._op_markChanged.invoke(self, ((), _ctx))

        def activateChangeDetection(self, _ctx=None):
            return _M_autogen.Planner.PlanningTask._op_activateChangeDetection.invoke(self, ((), _ctx))

        def planAvailable(self, _ctx=None):
            return _M_autogen.Planner.PlanningTask._op_planAvailable.invoke(self, ((), _ctx))

        def getPlan(self, _ctx=None):
            return _M_autogen.Planner.PlanningTask._op_getPlan.invoke(self, ((), _ctx))

        def checkedCast(proxy, facetOrCtx=None, _ctx=None):
            return _M_autogen.Planner.PlanningTaskPrx.ice_checkedCast(proxy, '::autogen::Planner::PlanningTask', facetOrCtx, _ctx)
        checkedCast = staticmethod(checkedCast)

        def uncheckedCast(proxy, facet=None):
            return _M_autogen.Planner.PlanningTaskPrx.ice_uncheckedCast(proxy, facet)
        uncheckedCast = staticmethod(uncheckedCast)

    _M_autogen.Planner._t_PlanningTaskPrx = IcePy.defineProxy('::autogen::Planner::PlanningTask', PlanningTaskPrx)

    _M_autogen.Planner._t_PlanningTask = IcePy.declareClass('::autogen::Planner::PlanningTask')

    _M_autogen.Planner._t_PlanningTask = IcePy.defineClass('::autogen::Planner::PlanningTask', PlanningTask, (), True, None, (), (
        ('id', (), IcePy._t_int),
        ('planningAgent', (), IcePy._t_string),
        ('objects', (), _M_autogen.Planner._t_objDeclSeq),
        ('state', (), _M_autogen.Planner._t_PlanningState),
        ('goal', (), IcePy._t_string)
    ))
    PlanningTask.ice_type = _M_autogen.Planner._t_PlanningTask

    PlanningTask._op_loadMAPLTask = IcePy.Operation('loadMAPLTask', Ice.OperationMode.Normal, Ice.OperationMode.Normal, False, (), (((), IcePy._t_string), ((), IcePy._t_string), ((), IcePy._t_string)), (), None, ())
    PlanningTask._op_markChanged = IcePy.Operation('markChanged', Ice.OperationMode.Normal, Ice.OperationMode.Normal, False, (), (), (), None, ())
    PlanningTask._op_activateChangeDetection = IcePy.Operation('activateChangeDetection', Ice.OperationMode.Normal, Ice.OperationMode.Normal, False, (), (), (), None, ())
    PlanningTask._op_planAvailable = IcePy.Operation('planAvailable', Ice.OperationMode.Normal, Ice.OperationMode.Normal, False, (), (), (), IcePy._t_bool, ())
    PlanningTask._op_getPlan = IcePy.Operation('getPlan', Ice.OperationMode.Normal, Ice.OperationMode.Normal, False, (), (), (), IcePy._t_string, ())

    _M_autogen.Planner.PlanningTask = PlanningTask
    del PlanningTask

    _M_autogen.Planner.PlanningTaskPrx = PlanningTaskPrx
    del PlanningTaskPrx

if not _M_autogen.Planner.__dict__.has_key('PlannerServer'):
    _M_autogen.Planner.PlannerServer = Ice.createTempClass()
    class PlannerServer(_M_cast.interfaces.CASTComponent):
        def __init__(self):
            if __builtin__.type(self) == _M_autogen.Planner.PlannerServer:
                raise RuntimeError('autogen.Planner.PlannerServer is an abstract class')

        def ice_ids(self, current=None):
            return ('::Ice::Object', '::autogen::Planner::PlannerServer', '::cast::interfaces::CASTComponent')

        def ice_id(self, current=None):
            return '::autogen::Planner::PlannerServer'

        def ice_staticId():
            return '::autogen::Planner::PlannerServer'
        ice_staticId = staticmethod(ice_staticId)

        #
        # Operation signatures.
        #
        # def newTask(self, current=None):
        # def registerTask(self, task, current=None):
        # def printString(self, astring, current=None):

        def __str__(self):
            return IcePy.stringify(self, _M_autogen.Planner._t_PlannerServer)

        __repr__ = __str__

    _M_autogen.Planner.PlannerServerPrx = Ice.createTempClass()
    class PlannerServerPrx(_M_cast.interfaces.CASTComponentPrx):

        def newTask(self, _ctx=None):
            return _M_autogen.Planner.PlannerServer._op_newTask.invoke(self, ((), _ctx))

        def registerTask(self, task, _ctx=None):
            return _M_autogen.Planner.PlannerServer._op_registerTask.invoke(self, ((task, ), _ctx))

        def printString(self, astring, _ctx=None):
            return _M_autogen.Planner.PlannerServer._op_printString.invoke(self, ((astring, ), _ctx))

        def checkedCast(proxy, facetOrCtx=None, _ctx=None):
            return _M_autogen.Planner.PlannerServerPrx.ice_checkedCast(proxy, '::autogen::Planner::PlannerServer', facetOrCtx, _ctx)
        checkedCast = staticmethod(checkedCast)

        def uncheckedCast(proxy, facet=None):
            return _M_autogen.Planner.PlannerServerPrx.ice_uncheckedCast(proxy, facet)
        uncheckedCast = staticmethod(uncheckedCast)

    _M_autogen.Planner._t_PlannerServerPrx = IcePy.defineProxy('::autogen::Planner::PlannerServer', PlannerServerPrx)

    _M_autogen.Planner._t_PlannerServer = IcePy.defineClass('::autogen::Planner::PlannerServer', PlannerServer, (), True, None, (_M_cast.interfaces._t_CASTComponent,), ())
    PlannerServer.ice_type = _M_autogen.Planner._t_PlannerServer

    PlannerServer._op_newTask = IcePy.Operation('newTask', Ice.OperationMode.Normal, Ice.OperationMode.Normal, False, (), (), (), _M_autogen.Planner._t_PlanningTask, ())
    PlannerServer._op_registerTask = IcePy.Operation('registerTask', Ice.OperationMode.Normal, Ice.OperationMode.Normal, False, (), (((), _M_autogen.Planner._t_PlanningTask),), (), None, ())
    PlannerServer._op_printString = IcePy.Operation('printString', Ice.OperationMode.Normal, Ice.OperationMode.Normal, False, (), (((), IcePy._t_string),), (), None, ())

    _M_autogen.Planner.PlannerServer = PlannerServer
    del PlannerServer

    _M_autogen.Planner.PlannerServerPrx = PlannerServerPrx
    del PlannerServerPrx

# End of module autogen.Planner

__name__ = 'autogen'

# End of module autogen
