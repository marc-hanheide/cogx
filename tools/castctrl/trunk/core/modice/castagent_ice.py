# **********************************************************************
#
# Copyright (c) 2003-2009 ZeroC, Inc. All rights reserved.
#
# This copy of Ice is licensed to you under the terms described in the
# ICE_LICENSE file included in this distribution.
#
# **********************************************************************

# Ice version 3.3.1
# Generated from file `castagent.ice'

import Ice, IcePy, __builtin__

if not Ice.__dict__.has_key("_struct_marker"):
    Ice._struct_marker = object()

# Start of module icemodule.castcontrol
_M_icemodule = Ice.openModule('icemodule')
_M_icemodule.castcontrol = Ice.openModule('icemodule.castcontrol')
__name__ = 'icemodule.castcontrol'

# Start of module icemodule.castcontrol.CastAgent
_M_icemodule.castcontrol.CastAgent = Ice.openModule('icemodule.castcontrol.CastAgent')
__name__ = 'icemodule.castcontrol.CastAgent'

if not _M_icemodule.castcontrol.CastAgent.__dict__.has_key('CastMessage'):
    _M_icemodule.castcontrol.CastAgent.CastMessage = Ice.createTempClass()
    class CastMessage(object):
        def __init__(self, time=0.0, msgtype=0, message=''):
            self.time = time
            self.msgtype = msgtype
            self.message = message

        def __hash__(self):
            _h = 0
            _h = 5 * _h + __builtin__.hash(self.time)
            _h = 5 * _h + __builtin__.hash(self.msgtype)
            _h = 5 * _h + __builtin__.hash(self.message)
            return _h % 0x7fffffff

        def __cmp__(self, other):
            if other == None:
                return 1
            if self.time < other.time:
                return -1
            elif self.time > other.time:
                return 1
            if self.msgtype < other.msgtype:
                return -1
            elif self.msgtype > other.msgtype:
                return 1
            if self.message < other.message:
                return -1
            elif self.message > other.message:
                return 1
            return 0

        def __str__(self):
            return IcePy.stringify(self, _M_icemodule.castcontrol.CastAgent._t_CastMessage)

        __repr__ = __str__

    _M_icemodule.castcontrol.CastAgent._t_CastMessage = IcePy.defineStruct('::castcontrol::CastAgent::CastMessage', CastMessage, (), (
        ('time', (), IcePy._t_double),
        ('msgtype', (), IcePy._t_int),
        ('message', (), IcePy._t_string)
    ))

    _M_icemodule.castcontrol.CastAgent.CastMessage = CastMessage
    del CastMessage

if not _M_icemodule.castcontrol.CastAgent.__dict__.has_key('_t_CastMessageList'):
    _M_icemodule.castcontrol.CastAgent._t_CastMessageList = IcePy.defineSequence('::castcontrol::CastAgent::CastMessageList', (), _M_icemodule.castcontrol.CastAgent._t_CastMessage)

if not _M_icemodule.castcontrol.CastAgent.__dict__.has_key('ProcessInfo'):
    _M_icemodule.castcontrol.CastAgent.ProcessInfo = Ice.createTempClass()
    class ProcessInfo(object):
        def __init__(self, name='', status=0, error=0):
            self.name = name
            self.status = status
            self.error = error

        def __hash__(self):
            _h = 0
            _h = 5 * _h + __builtin__.hash(self.name)
            _h = 5 * _h + __builtin__.hash(self.status)
            _h = 5 * _h + __builtin__.hash(self.error)
            return _h % 0x7fffffff

        def __cmp__(self, other):
            if other == None:
                return 1
            if self.name < other.name:
                return -1
            elif self.name > other.name:
                return 1
            if self.status < other.status:
                return -1
            elif self.status > other.status:
                return 1
            if self.error < other.error:
                return -1
            elif self.error > other.error:
                return 1
            return 0

        def __str__(self):
            return IcePy.stringify(self, _M_icemodule.castcontrol.CastAgent._t_ProcessInfo)

        __repr__ = __str__

    _M_icemodule.castcontrol.CastAgent._t_ProcessInfo = IcePy.defineStruct('::castcontrol::CastAgent::ProcessInfo', ProcessInfo, (), (
        ('name', (), IcePy._t_string),
        ('status', (), IcePy._t_int),
        ('error', (), IcePy._t_int)
    ))

    _M_icemodule.castcontrol.CastAgent.ProcessInfo = ProcessInfo
    del ProcessInfo

if not _M_icemodule.castcontrol.CastAgent.__dict__.has_key('_t_ProcessList'):
    _M_icemodule.castcontrol.CastAgent._t_ProcessList = IcePy.defineSequence('::castcontrol::CastAgent::ProcessList', (), _M_icemodule.castcontrol.CastAgent._t_ProcessInfo)

if not _M_icemodule.castcontrol.CastAgent.__dict__.has_key('Agent'):
    _M_icemodule.castcontrol.CastAgent.Agent = Ice.createTempClass()
    class Agent(Ice.Object):
        def __init__(self):
            if __builtin__.type(self) == _M_icemodule.castcontrol.CastAgent.Agent:
                raise RuntimeError('icemodule.castcontrol.CastAgent.Agent is an abstract class')

        def ice_ids(self, current=None):
            return ('::Ice::Object', '::castcontrol::CastAgent::Agent')

        def ice_id(self, current=None):
            return '::castcontrol::CastAgent::Agent'

        def ice_staticId():
            return '::castcontrol::CastAgent::Agent'
        ice_staticId = staticmethod(ice_staticId)

        #
        # Operation signatures.
        #
        # def getProcessList(self, current=None):
        # def readMessages(self, processName, current=None):
        # def startProcess(self, name, current=None):
        # def stopProcess(self, name, current=None):
        # def setLog4jClientProperties(self, propText, current=None):
        # def setLog4jServerProperties(self, port, propText, current=None):
        # def getCmakeCache(self, current=None):
        # def applyCmakeCache(self, cmakeCache, current=None):
        # def startRsync(self, srcHost, current=None):
        # def stopRsync(self, current=None):
        # def startBuild(self, target, current=None):
        # def stopBuild(self, current=None):

        def __str__(self):
            return IcePy.stringify(self, _M_icemodule.castcontrol.CastAgent._t_Agent)

        __repr__ = __str__

    _M_icemodule.castcontrol.CastAgent.AgentPrx = Ice.createTempClass()
    class AgentPrx(Ice.ObjectPrx):

        def getProcessList(self, _ctx=None):
            return _M_icemodule.castcontrol.CastAgent.Agent._op_getProcessList.invoke(self, ((), _ctx))

        def readMessages(self, processName, _ctx=None):
            return _M_icemodule.castcontrol.CastAgent.Agent._op_readMessages.invoke(self, ((processName, ), _ctx))

        def startProcess(self, name, _ctx=None):
            return _M_icemodule.castcontrol.CastAgent.Agent._op_startProcess.invoke(self, ((name, ), _ctx))

        def stopProcess(self, name, _ctx=None):
            return _M_icemodule.castcontrol.CastAgent.Agent._op_stopProcess.invoke(self, ((name, ), _ctx))

        def setLog4jClientProperties(self, propText, _ctx=None):
            return _M_icemodule.castcontrol.CastAgent.Agent._op_setLog4jClientProperties.invoke(self, ((propText, ), _ctx))

        def setLog4jServerProperties(self, port, propText, _ctx=None):
            return _M_icemodule.castcontrol.CastAgent.Agent._op_setLog4jServerProperties.invoke(self, ((port, propText), _ctx))

        def getCmakeCache(self, _ctx=None):
            return _M_icemodule.castcontrol.CastAgent.Agent._op_getCmakeCache.invoke(self, ((), _ctx))

        def applyCmakeCache(self, cmakeCache, _ctx=None):
            return _M_icemodule.castcontrol.CastAgent.Agent._op_applyCmakeCache.invoke(self, ((cmakeCache, ), _ctx))

        def startRsync(self, srcHost, _ctx=None):
            return _M_icemodule.castcontrol.CastAgent.Agent._op_startRsync.invoke(self, ((srcHost, ), _ctx))

        def stopRsync(self, _ctx=None):
            return _M_icemodule.castcontrol.CastAgent.Agent._op_stopRsync.invoke(self, ((), _ctx))

        def startBuild(self, target, _ctx=None):
            return _M_icemodule.castcontrol.CastAgent.Agent._op_startBuild.invoke(self, ((target, ), _ctx))

        def stopBuild(self, _ctx=None):
            return _M_icemodule.castcontrol.CastAgent.Agent._op_stopBuild.invoke(self, ((), _ctx))

        def checkedCast(proxy, facetOrCtx=None, _ctx=None):
            return _M_icemodule.castcontrol.CastAgent.AgentPrx.ice_checkedCast(proxy, '::castcontrol::CastAgent::Agent', facetOrCtx, _ctx)
        checkedCast = staticmethod(checkedCast)

        def uncheckedCast(proxy, facet=None):
            return _M_icemodule.castcontrol.CastAgent.AgentPrx.ice_uncheckedCast(proxy, facet)
        uncheckedCast = staticmethod(uncheckedCast)

    _M_icemodule.castcontrol.CastAgent._t_AgentPrx = IcePy.defineProxy('::castcontrol::CastAgent::Agent', AgentPrx)

    _M_icemodule.castcontrol.CastAgent._t_Agent = IcePy.defineClass('::castcontrol::CastAgent::Agent', Agent, (), True, None, (), ())
    Agent.ice_type = _M_icemodule.castcontrol.CastAgent._t_Agent

    Agent._op_getProcessList = IcePy.Operation('getProcessList', Ice.OperationMode.Idempotent, Ice.OperationMode.Idempotent, False, (), (), (), _M_icemodule.castcontrol.CastAgent._t_ProcessList, ())
    Agent._op_readMessages = IcePy.Operation('readMessages', Ice.OperationMode.Normal, Ice.OperationMode.Normal, False, (), (((), IcePy._t_string),), (), _M_icemodule.castcontrol.CastAgent._t_CastMessageList, ())
    Agent._op_startProcess = IcePy.Operation('startProcess', Ice.OperationMode.Normal, Ice.OperationMode.Normal, False, (), (((), IcePy._t_string),), (), IcePy._t_int, ())
    Agent._op_stopProcess = IcePy.Operation('stopProcess', Ice.OperationMode.Normal, Ice.OperationMode.Normal, False, (), (((), IcePy._t_string),), (), IcePy._t_int, ())
    Agent._op_setLog4jClientProperties = IcePy.Operation('setLog4jClientProperties', Ice.OperationMode.Normal, Ice.OperationMode.Normal, False, (), (((), IcePy._t_string),), (), None, ())
    Agent._op_setLog4jServerProperties = IcePy.Operation('setLog4jServerProperties', Ice.OperationMode.Normal, Ice.OperationMode.Normal, False, (), (((), IcePy._t_int), ((), IcePy._t_string)), (), None, ())
    Agent._op_getCmakeCache = IcePy.Operation('getCmakeCache', Ice.OperationMode.Normal, Ice.OperationMode.Normal, False, (), (), (), IcePy._t_string, ())
    Agent._op_applyCmakeCache = IcePy.Operation('applyCmakeCache', Ice.OperationMode.Normal, Ice.OperationMode.Normal, False, (), (((), IcePy._t_string),), (), None, ())
    Agent._op_startRsync = IcePy.Operation('startRsync', Ice.OperationMode.Normal, Ice.OperationMode.Normal, False, (), (((), IcePy._t_string),), (), IcePy._t_bool, ())
    Agent._op_stopRsync = IcePy.Operation('stopRsync', Ice.OperationMode.Normal, Ice.OperationMode.Normal, False, (), (), (), IcePy._t_bool, ())
    Agent._op_startBuild = IcePy.Operation('startBuild', Ice.OperationMode.Normal, Ice.OperationMode.Normal, False, (), (((), IcePy._t_string),), (), IcePy._t_bool, ())
    Agent._op_stopBuild = IcePy.Operation('stopBuild', Ice.OperationMode.Normal, Ice.OperationMode.Normal, False, (), (), (), IcePy._t_bool, ())

    _M_icemodule.castcontrol.CastAgent.Agent = Agent
    del Agent

    _M_icemodule.castcontrol.CastAgent.AgentPrx = AgentPrx
    del AgentPrx

# End of module icemodule.castcontrol.CastAgent

__name__ = 'icemodule.castcontrol'

# End of module icemodule.castcontrol
