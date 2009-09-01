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

if not _M_autogen.Planner.__dict__.has_key('Completion'):
    _M_autogen.Planner.Completion = Ice.createTempClass()
    class Completion(object):

        def __init__(self, val):
            assert(val >= 0 and val < 5)
            self.value = val

        def __str__(self):
            if self.value == 0:
                return 'PENDING'
            elif self.value == 1:
                return 'INPROGRESS'
            elif self.value == 2:
                return 'ABORTED'
            elif self.value == 3:
                return 'FAILED'
            elif self.value == 4:
                return 'SUCCEEDED'
            return None

        __repr__ = __str__

        def __hash__(self):
            return self.value

        def __cmp__(self, other):
            return cmp(self.value, other.value)

    Completion.PENDING = Completion(0)
    Completion.INPROGRESS = Completion(1)
    Completion.ABORTED = Completion(2)
    Completion.FAILED = Completion(3)
    Completion.SUCCEEDED = Completion(4)

    _M_autogen.Planner._t_Completion = IcePy.defineEnum('::autogen::Planner::Completion', Completion, (), (Completion.PENDING, Completion.INPROGRESS, Completion.ABORTED, Completion.FAILED, Completion.SUCCEEDED))

    _M_autogen.Planner.Completion = Completion
    del Completion

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
        def __init__(self, id=0, goal='', plan='', objects='', state=''):
            self.id = id
            self.goal = goal
            self.plan = plan
            self.objects = objects
            self.state = state

        def ice_ids(self, current=None):
            return ('::Ice::Object', '::autogen::Planner::PlanningTask')

        def ice_id(self, current=None):
            return '::autogen::Planner::PlanningTask'

        def ice_staticId():
            return '::autogen::Planner::PlanningTask'
        ice_staticId = staticmethod(ice_staticId)

        def __str__(self):
            return IcePy.stringify(self, _M_autogen.Planner._t_PlanningTask)

        __repr__ = __str__

    _M_autogen.Planner.PlanningTaskPrx = Ice.createTempClass()
    class PlanningTaskPrx(Ice.ObjectPrx):

        def checkedCast(proxy, facetOrCtx=None, _ctx=None):
            return _M_autogen.Planner.PlanningTaskPrx.ice_checkedCast(proxy, '::autogen::Planner::PlanningTask', facetOrCtx, _ctx)
        checkedCast = staticmethod(checkedCast)

        def uncheckedCast(proxy, facet=None):
            return _M_autogen.Planner.PlanningTaskPrx.ice_uncheckedCast(proxy, facet)
        uncheckedCast = staticmethod(uncheckedCast)

    _M_autogen.Planner._t_PlanningTaskPrx = IcePy.defineProxy('::autogen::Planner::PlanningTask', PlanningTaskPrx)

    _M_autogen.Planner._t_PlanningTask = IcePy.defineClass('::autogen::Planner::PlanningTask', PlanningTask, (), False, None, (), (
        ('id', (), IcePy._t_int),
        ('goal', (), IcePy._t_string),
        ('plan', (), IcePy._t_string),
        ('objects', (), IcePy._t_string),
        ('state', (), IcePy._t_string)
    ))
    PlanningTask.ice_type = _M_autogen.Planner._t_PlanningTask

    _M_autogen.Planner.PlanningTask = PlanningTask
    del PlanningTask

    _M_autogen.Planner.PlanningTaskPrx = PlanningTaskPrx
    del PlanningTaskPrx

if not _M_autogen.Planner.__dict__.has_key('CppServer'):
    _M_autogen.Planner.CppServer = Ice.createTempClass()
    class CppServer(Ice.Object):
        def __init__(self):
            if __builtin__.type(self) == _M_autogen.Planner.CppServer:
                raise RuntimeError('autogen.Planner.CppServer is an abstract class')

        def ice_ids(self, current=None):
            return ('::Ice::Object', '::autogen::Planner::CppServer')

        def ice_id(self, current=None):
            return '::autogen::Planner::CppServer'

        def ice_staticId():
            return '::autogen::Planner::CppServer'
        ice_staticId = staticmethod(ice_staticId)

        #
        # Operation signatures.
        #
        # def deliverPlan(self, task, current=None):

        def __str__(self):
            return IcePy.stringify(self, _M_autogen.Planner._t_CppServer)

        __repr__ = __str__

    _M_autogen.Planner.CppServerPrx = Ice.createTempClass()
    class CppServerPrx(Ice.ObjectPrx):

        def deliverPlan(self, task, _ctx=None):
            return _M_autogen.Planner.CppServer._op_deliverPlan.invoke(self, ((task, ), _ctx))

        def checkedCast(proxy, facetOrCtx=None, _ctx=None):
            return _M_autogen.Planner.CppServerPrx.ice_checkedCast(proxy, '::autogen::Planner::CppServer', facetOrCtx, _ctx)
        checkedCast = staticmethod(checkedCast)

        def uncheckedCast(proxy, facet=None):
            return _M_autogen.Planner.CppServerPrx.ice_uncheckedCast(proxy, facet)
        uncheckedCast = staticmethod(uncheckedCast)

    _M_autogen.Planner._t_CppServerPrx = IcePy.defineProxy('::autogen::Planner::CppServer', CppServerPrx)

    _M_autogen.Planner._t_CppServer = IcePy.defineClass('::autogen::Planner::CppServer', CppServer, (), True, None, (), ())
    CppServer.ice_type = _M_autogen.Planner._t_CppServer

    CppServer._op_deliverPlan = IcePy.Operation('deliverPlan', Ice.OperationMode.Normal, Ice.OperationMode.Normal, False, (), (((), _M_autogen.Planner._t_PlanningTask),), (), None, ())

    _M_autogen.Planner.CppServer = CppServer
    del CppServer

    _M_autogen.Planner.CppServerPrx = CppServerPrx
    del CppServerPrx

if not _M_autogen.Planner.__dict__.has_key('PythonServer'):
    _M_autogen.Planner.PythonServer = Ice.createTempClass()
    class PythonServer(_M_cast.interfaces.CASTComponent):
        def __init__(self):
            if __builtin__.type(self) == _M_autogen.Planner.PythonServer:
                raise RuntimeError('autogen.Planner.PythonServer is an abstract class')

        def ice_ids(self, current=None):
            return ('::Ice::Object', '::autogen::Planner::PythonServer', '::cast::interfaces::CASTComponent')

        def ice_id(self, current=None):
            return '::autogen::Planner::PythonServer'

        def ice_staticId():
            return '::autogen::Planner::PythonServer'
        ice_staticId = staticmethod(ice_staticId)

        #
        # Operation signatures.
        #
        # def registerClient(self, client, current=None):
        # def registerTask(self, task, current=None):

        def __str__(self):
            return IcePy.stringify(self, _M_autogen.Planner._t_PythonServer)

        __repr__ = __str__

    _M_autogen.Planner.PythonServerPrx = Ice.createTempClass()
    class PythonServerPrx(_M_cast.interfaces.CASTComponentPrx):

        def registerClient(self, client, _ctx=None):
            return _M_autogen.Planner.PythonServer._op_registerClient.invoke(self, ((client, ), _ctx))

        def registerTask(self, task, _ctx=None):
            return _M_autogen.Planner.PythonServer._op_registerTask.invoke(self, ((task, ), _ctx))

        def checkedCast(proxy, facetOrCtx=None, _ctx=None):
            return _M_autogen.Planner.PythonServerPrx.ice_checkedCast(proxy, '::autogen::Planner::PythonServer', facetOrCtx, _ctx)
        checkedCast = staticmethod(checkedCast)

        def uncheckedCast(proxy, facet=None):
            return _M_autogen.Planner.PythonServerPrx.ice_uncheckedCast(proxy, facet)
        uncheckedCast = staticmethod(uncheckedCast)

    _M_autogen.Planner._t_PythonServerPrx = IcePy.defineProxy('::autogen::Planner::PythonServer', PythonServerPrx)

    _M_autogen.Planner._t_PythonServer = IcePy.defineClass('::autogen::Planner::PythonServer', PythonServer, (), True, None, (_M_cast.interfaces._t_CASTComponent,), ())
    PythonServer.ice_type = _M_autogen.Planner._t_PythonServer

    PythonServer._op_registerClient = IcePy.Operation('registerClient', Ice.OperationMode.Normal, Ice.OperationMode.Normal, False, (), (((), _M_autogen.Planner._t_CppServerPrx),), (), None, ())
    PythonServer._op_registerTask = IcePy.Operation('registerTask', Ice.OperationMode.Normal, Ice.OperationMode.Normal, False, (), (((), _M_autogen.Planner._t_PlanningTask),), (), None, ())

    _M_autogen.Planner.PythonServer = PythonServer
    del PythonServer

    _M_autogen.Planner.PythonServerPrx = PythonServerPrx
    del PythonServerPrx

# End of module autogen.Planner

__name__ = 'autogen'

# End of module autogen
