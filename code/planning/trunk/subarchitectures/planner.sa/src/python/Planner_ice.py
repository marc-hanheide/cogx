# **********************************************************************
#
# Copyright (c) 2003-2009 ZeroC, Inc. All rights reserved.
#
# This copy of Ice is licensed to you under the terms described in the
# ICE_LICENSE file included in this distribution.
#
# **********************************************************************

# Ice version 3.3.1
# Generated from file `Planner.ice'

import Ice, IcePy, __builtin__

if not Ice.__dict__.has_key("_struct_marker"):
    Ice._struct_marker = object()

# Start of module cast
_M_cast = Ice.openModule('cast')
__name__ = 'cast'

# Start of module cast.cdl
_M_cast.cdl = Ice.openModule('cast.cdl')
__name__ = 'cast.cdl'

_M_cast.cdl.CASTRELEASESTRING = "2.1.0rc1 (Transport Is Arranged)"

_M_cast.cdl.JAVASERVERPORT = 10111

_M_cast.cdl.CPPSERVERPORT = 10211

_M_cast.cdl.JAVACLIENTSERVERPORT = 10311

_M_cast.cdl.PYTHONSERVERPORT = 10411

_M_cast.cdl.SUBARCHIDKEY = "org.cognitivesystem.cast.subarchID"

_M_cast.cdl.COMPONENTNUMBERKEY = "org.cognitivesystem.cast.componentNumber"

_M_cast.cdl.CONFIGFILEKEY = "org.cognitivesystem.cast.config"

_M_cast.cdl.WMIDSKEY = "org.cognitivesystem.ast.wmids"

_M_cast.cdl.COMPONENTIDSKEY = "org.cognitivesystem.cast.compids"

_M_cast.cdl.LOGKEY = "--log"

_M_cast.cdl.DEBUGKEY = "--debug"

_M_cast.cdl.DEBUGEVENTSKEY = "--debug-events"

_M_cast.cdl.IGNORESAKEY = "--ignore"

if not _M_cast.cdl.__dict__.has_key('_t_StringMap'):
    _M_cast.cdl._t_StringMap = IcePy.defineDictionary('::cast::cdl::StringMap', (), IcePy._t_string, IcePy._t_string)

if not _M_cast.cdl.__dict__.has_key('ComponentLanguage'):
    _M_cast.cdl.ComponentLanguage = Ice.createTempClass()
    class ComponentLanguage(object):

        def __init__(self, val):
            assert(val >= 0 and val < 3)
            self.value = val

        def __str__(self):
            if self.value == 0:
                return 'CPP'
            elif self.value == 1:
                return 'JAVA'
            elif self.value == 2:
                return 'PYTHON'
            return None

        __repr__ = __str__

        def __hash__(self):
            return self.value

        def __cmp__(self, other):
            return cmp(self.value, other.value)

    ComponentLanguage.CPP = ComponentLanguage(0)
    ComponentLanguage.JAVA = ComponentLanguage(1)
    ComponentLanguage.PYTHON = ComponentLanguage(2)

    _M_cast.cdl._t_ComponentLanguage = IcePy.defineEnum('::cast::cdl::ComponentLanguage', ComponentLanguage, (), (ComponentLanguage.CPP, ComponentLanguage.JAVA, ComponentLanguage.PYTHON))

    _M_cast.cdl.ComponentLanguage = ComponentLanguage
    del ComponentLanguage

if not _M_cast.cdl.__dict__.has_key('ComponentDescription'):
    _M_cast.cdl.ComponentDescription = Ice.createTempClass()
    class ComponentDescription(object):
        def __init__(self, componentName='', className='', language=_M_cast.cdl.ComponentLanguage.CPP, hostName='', configuration=None, newProcess=False):
            self.componentName = componentName
            self.className = className
            self.language = language
            self.hostName = hostName
            self.configuration = configuration
            self.newProcess = newProcess

        def __hash__(self):
            _h = 0
            _h = 5 * _h + __builtin__.hash(self.componentName)
            _h = 5 * _h + __builtin__.hash(self.className)
            _h = 5 * _h + __builtin__.hash(self.language)
            _h = 5 * _h + __builtin__.hash(self.hostName)
            if self.configuration:
                for _i0 in self.configuration:
                    _h = 5 * _h + __builtin__.hash(_i0)
                    _h = 5 * _h + __builtin__.hash(self.configuration[_i0])
            _h = 5 * _h + __builtin__.hash(self.newProcess)
            return _h % 0x7fffffff

        def __cmp__(self, other):
            if other == None:
                return 1
            if self.componentName < other.componentName:
                return -1
            elif self.componentName > other.componentName:
                return 1
            if self.className < other.className:
                return -1
            elif self.className > other.className:
                return 1
            if self.language < other.language:
                return -1
            elif self.language > other.language:
                return 1
            if self.hostName < other.hostName:
                return -1
            elif self.hostName > other.hostName:
                return 1
            if self.configuration < other.configuration:
                return -1
            elif self.configuration > other.configuration:
                return 1
            if self.newProcess < other.newProcess:
                return -1
            elif self.newProcess > other.newProcess:
                return 1
            return 0

        def __str__(self):
            return IcePy.stringify(self, _M_cast.cdl._t_ComponentDescription)

        __repr__ = __str__

    _M_cast.cdl._t_ComponentDescription = IcePy.defineStruct('::cast::cdl::ComponentDescription', ComponentDescription, (), (
        ('componentName', (), IcePy._t_string),
        ('className', (), IcePy._t_string),
        ('language', (), _M_cast.cdl._t_ComponentLanguage),
        ('hostName', (), IcePy._t_string),
        ('configuration', (), _M_cast.cdl._t_StringMap),
        ('newProcess', (), IcePy._t_bool)
    ))

    _M_cast.cdl.ComponentDescription = ComponentDescription
    del ComponentDescription

if not _M_cast.cdl.__dict__.has_key('_t_ByteSeq'):
    _M_cast.cdl._t_ByteSeq = IcePy.defineSequence('::cast::cdl::ByteSeq', (), IcePy._t_byte)

if not _M_cast.cdl.__dict__.has_key('TestStructString'):
    _M_cast.cdl.TestStructString = Ice.createTempClass()
    class TestStructString(Ice.Object):
        def __init__(self, dummy=''):
            self.dummy = dummy

        def ice_ids(self, current=None):
            return ('::Ice::Object', '::cast::cdl::TestStructString')

        def ice_id(self, current=None):
            return '::cast::cdl::TestStructString'

        def ice_staticId():
            return '::cast::cdl::TestStructString'
        ice_staticId = staticmethod(ice_staticId)

        def __str__(self):
            return IcePy.stringify(self, _M_cast.cdl._t_TestStructString)

        __repr__ = __str__

    _M_cast.cdl.TestStructStringPrx = Ice.createTempClass()
    class TestStructStringPrx(Ice.ObjectPrx):

        def checkedCast(proxy, facetOrCtx=None, _ctx=None):
            return _M_cast.cdl.TestStructStringPrx.ice_checkedCast(proxy, '::cast::cdl::TestStructString', facetOrCtx, _ctx)
        checkedCast = staticmethod(checkedCast)

        def uncheckedCast(proxy, facet=None):
            return _M_cast.cdl.TestStructStringPrx.ice_uncheckedCast(proxy, facet)
        uncheckedCast = staticmethod(uncheckedCast)

    _M_cast.cdl._t_TestStructStringPrx = IcePy.defineProxy('::cast::cdl::TestStructString', TestStructStringPrx)

    _M_cast.cdl._t_TestStructString = IcePy.defineClass('::cast::cdl::TestStructString', TestStructString, (), False, None, (), (('dummy', (), IcePy._t_string),))
    TestStructString.ice_type = _M_cast.cdl._t_TestStructString

    _M_cast.cdl.TestStructString = TestStructString
    del TestStructString

    _M_cast.cdl.TestStructStringPrx = TestStructStringPrx
    del TestStructStringPrx

if not _M_cast.cdl.__dict__.has_key('TestStructInt'):
    _M_cast.cdl.TestStructInt = Ice.createTempClass()
    class TestStructInt(Ice.Object):
        def __init__(self, dummy=0):
            self.dummy = dummy

        def ice_ids(self, current=None):
            return ('::Ice::Object', '::cast::cdl::TestStructInt')

        def ice_id(self, current=None):
            return '::cast::cdl::TestStructInt'

        def ice_staticId():
            return '::cast::cdl::TestStructInt'
        ice_staticId = staticmethod(ice_staticId)

        def __str__(self):
            return IcePy.stringify(self, _M_cast.cdl._t_TestStructInt)

        __repr__ = __str__

    _M_cast.cdl.TestStructIntPrx = Ice.createTempClass()
    class TestStructIntPrx(Ice.ObjectPrx):

        def checkedCast(proxy, facetOrCtx=None, _ctx=None):
            return _M_cast.cdl.TestStructIntPrx.ice_checkedCast(proxy, '::cast::cdl::TestStructInt', facetOrCtx, _ctx)
        checkedCast = staticmethod(checkedCast)

        def uncheckedCast(proxy, facet=None):
            return _M_cast.cdl.TestStructIntPrx.ice_uncheckedCast(proxy, facet)
        uncheckedCast = staticmethod(uncheckedCast)

    _M_cast.cdl._t_TestStructIntPrx = IcePy.defineProxy('::cast::cdl::TestStructInt', TestStructIntPrx)

    _M_cast.cdl._t_TestStructInt = IcePy.defineClass('::cast::cdl::TestStructInt', TestStructInt, (), False, None, (), (('dummy', (), IcePy._t_int),))
    TestStructInt.ice_type = _M_cast.cdl._t_TestStructInt

    _M_cast.cdl.TestStructInt = TestStructInt
    del TestStructInt

    _M_cast.cdl.TestStructIntPrx = TestStructIntPrx
    del TestStructIntPrx

if not _M_cast.cdl.__dict__.has_key('WorkingMemoryAddress'):
    _M_cast.cdl.WorkingMemoryAddress = Ice.createTempClass()
    class WorkingMemoryAddress(object):
        def __init__(self, id='', subarchitecture=''):
            self.id = id
            self.subarchitecture = subarchitecture

        def __hash__(self):
            _h = 0
            _h = 5 * _h + __builtin__.hash(self.id)
            _h = 5 * _h + __builtin__.hash(self.subarchitecture)
            return _h % 0x7fffffff

        def __cmp__(self, other):
            if other == None:
                return 1
            if self.id < other.id:
                return -1
            elif self.id > other.id:
                return 1
            if self.subarchitecture < other.subarchitecture:
                return -1
            elif self.subarchitecture > other.subarchitecture:
                return 1
            return 0

        def __str__(self):
            return IcePy.stringify(self, _M_cast.cdl._t_WorkingMemoryAddress)

        __repr__ = __str__

    _M_cast.cdl._t_WorkingMemoryAddress = IcePy.defineStruct('::cast::cdl::WorkingMemoryAddress', WorkingMemoryAddress, (), (
        ('id', (), IcePy._t_string),
        ('subarchitecture', (), IcePy._t_string)
    ))

    _M_cast.cdl.WorkingMemoryAddress = WorkingMemoryAddress
    del WorkingMemoryAddress

if not _M_cast.cdl.__dict__.has_key('WorkingMemoryEntry'):
    _M_cast.cdl.WorkingMemoryEntry = Ice.createTempClass()
    class WorkingMemoryEntry(Ice.Object):
        def __init__(self, id='', type='', version=0, entry=None):
            self.id = id
            self.type = type
            self.version = version
            self.entry = entry

        def ice_ids(self, current=None):
            return ('::Ice::Object', '::cast::cdl::WorkingMemoryEntry')

        def ice_id(self, current=None):
            return '::cast::cdl::WorkingMemoryEntry'

        def ice_staticId():
            return '::cast::cdl::WorkingMemoryEntry'
        ice_staticId = staticmethod(ice_staticId)

        def __str__(self):
            return IcePy.stringify(self, _M_cast.cdl._t_WorkingMemoryEntry)

        __repr__ = __str__

    _M_cast.cdl.WorkingMemoryEntryPrx = Ice.createTempClass()
    class WorkingMemoryEntryPrx(Ice.ObjectPrx):

        def checkedCast(proxy, facetOrCtx=None, _ctx=None):
            return _M_cast.cdl.WorkingMemoryEntryPrx.ice_checkedCast(proxy, '::cast::cdl::WorkingMemoryEntry', facetOrCtx, _ctx)
        checkedCast = staticmethod(checkedCast)

        def uncheckedCast(proxy, facet=None):
            return _M_cast.cdl.WorkingMemoryEntryPrx.ice_uncheckedCast(proxy, facet)
        uncheckedCast = staticmethod(uncheckedCast)

    _M_cast.cdl._t_WorkingMemoryEntryPrx = IcePy.defineProxy('::cast::cdl::WorkingMemoryEntry', WorkingMemoryEntryPrx)

    _M_cast.cdl._t_WorkingMemoryEntry = IcePy.declareClass('::cast::cdl::WorkingMemoryEntry')

    _M_cast.cdl._t_WorkingMemoryEntry = IcePy.defineClass('::cast::cdl::WorkingMemoryEntry', WorkingMemoryEntry, (), False, None, (), (
        ('id', (), IcePy._t_string),
        ('type', (), IcePy._t_string),
        ('version', (), IcePy._t_int),
        ('entry', (), IcePy._t_Object)
    ))
    WorkingMemoryEntry.ice_type = _M_cast.cdl._t_WorkingMemoryEntry

    _M_cast.cdl.WorkingMemoryEntry = WorkingMemoryEntry
    del WorkingMemoryEntry

    _M_cast.cdl.WorkingMemoryEntryPrx = WorkingMemoryEntryPrx
    del WorkingMemoryEntryPrx

if not _M_cast.cdl.__dict__.has_key('_t_WorkingMemoryEntrySeq'):
    _M_cast.cdl._t_WorkingMemoryEntrySeq = IcePy.defineSequence('::cast::cdl::WorkingMemoryEntrySeq', (), _M_cast.cdl._t_WorkingMemoryEntry)

if not _M_cast.cdl.__dict__.has_key('WorkingMemoryOperation'):
    _M_cast.cdl.WorkingMemoryOperation = Ice.createTempClass()
    class WorkingMemoryOperation(object):

        def __init__(self, val):
            assert(val >= 0 and val < 5)
            self.value = val

        def __str__(self):
            if self.value == 0:
                return 'ADD'
            elif self.value == 1:
                return 'OVERWRITE'
            elif self.value == 2:
                return 'DELETE'
            elif self.value == 3:
                return 'GET'
            elif self.value == 4:
                return 'WILDCARD'
            return None

        __repr__ = __str__

        def __hash__(self):
            return self.value

        def __cmp__(self, other):
            return cmp(self.value, other.value)

    WorkingMemoryOperation.ADD = WorkingMemoryOperation(0)
    WorkingMemoryOperation.OVERWRITE = WorkingMemoryOperation(1)
    WorkingMemoryOperation.DELETE = WorkingMemoryOperation(2)
    WorkingMemoryOperation.GET = WorkingMemoryOperation(3)
    WorkingMemoryOperation.WILDCARD = WorkingMemoryOperation(4)

    _M_cast.cdl._t_WorkingMemoryOperation = IcePy.defineEnum('::cast::cdl::WorkingMemoryOperation', WorkingMemoryOperation, (), (WorkingMemoryOperation.ADD, WorkingMemoryOperation.OVERWRITE, WorkingMemoryOperation.DELETE, WorkingMemoryOperation.GET, WorkingMemoryOperation.WILDCARD))

    _M_cast.cdl.WorkingMemoryOperation = WorkingMemoryOperation
    del WorkingMemoryOperation

if not _M_cast.cdl.__dict__.has_key('WorkingMemoryChangeQueueBehaviour'):
    _M_cast.cdl.WorkingMemoryChangeQueueBehaviour = Ice.createTempClass()
    class WorkingMemoryChangeQueueBehaviour(object):

        def __init__(self, val):
            assert(val >= 0 and val < 2)
            self.value = val

        def __str__(self):
            if self.value == 0:
                return 'DISCARD'
            elif self.value == 1:
                return 'QUEUE'
            return None

        __repr__ = __str__

        def __hash__(self):
            return self.value

        def __cmp__(self, other):
            return cmp(self.value, other.value)

    WorkingMemoryChangeQueueBehaviour.DISCARD = WorkingMemoryChangeQueueBehaviour(0)
    WorkingMemoryChangeQueueBehaviour.QUEUE = WorkingMemoryChangeQueueBehaviour(1)

    _M_cast.cdl._t_WorkingMemoryChangeQueueBehaviour = IcePy.defineEnum('::cast::cdl::WorkingMemoryChangeQueueBehaviour', WorkingMemoryChangeQueueBehaviour, (), (WorkingMemoryChangeQueueBehaviour.DISCARD, WorkingMemoryChangeQueueBehaviour.QUEUE))

    _M_cast.cdl.WorkingMemoryChangeQueueBehaviour = WorkingMemoryChangeQueueBehaviour
    del WorkingMemoryChangeQueueBehaviour

if not _M_cast.cdl.__dict__.has_key('WorkingMemoryPermissions'):
    _M_cast.cdl.WorkingMemoryPermissions = Ice.createTempClass()
    class WorkingMemoryPermissions(object):

        def __init__(self, val):
            assert(val >= 0 and val < 6)
            self.value = val

        def __str__(self):
            if self.value == 0:
                return 'LOCKEDO'
            elif self.value == 1:
                return 'LOCKEDOD'
            elif self.value == 2:
                return 'LOCKEDODR'
            elif self.value == 3:
                return 'UNLOCKED'
            elif self.value == 4:
                return 'DOESNOTEXIST'
            elif self.value == 5:
                return 'ALREADYLOCKED'
            return None

        __repr__ = __str__

        def __hash__(self):
            return self.value

        def __cmp__(self, other):
            return cmp(self.value, other.value)

    WorkingMemoryPermissions.LOCKEDO = WorkingMemoryPermissions(0)
    WorkingMemoryPermissions.LOCKEDOD = WorkingMemoryPermissions(1)
    WorkingMemoryPermissions.LOCKEDODR = WorkingMemoryPermissions(2)
    WorkingMemoryPermissions.UNLOCKED = WorkingMemoryPermissions(3)
    WorkingMemoryPermissions.DOESNOTEXIST = WorkingMemoryPermissions(4)
    WorkingMemoryPermissions.ALREADYLOCKED = WorkingMemoryPermissions(5)

    _M_cast.cdl._t_WorkingMemoryPermissions = IcePy.defineEnum('::cast::cdl::WorkingMemoryPermissions', WorkingMemoryPermissions, (), (WorkingMemoryPermissions.LOCKEDO, WorkingMemoryPermissions.LOCKEDOD, WorkingMemoryPermissions.LOCKEDODR, WorkingMemoryPermissions.UNLOCKED, WorkingMemoryPermissions.DOESNOTEXIST, WorkingMemoryPermissions.ALREADYLOCKED))

    _M_cast.cdl.WorkingMemoryPermissions = WorkingMemoryPermissions
    del WorkingMemoryPermissions

if not _M_cast.cdl.__dict__.has_key('WorkingMemoryLockRequest'):
    _M_cast.cdl.WorkingMemoryLockRequest = Ice.createTempClass()
    class WorkingMemoryLockRequest(object):

        def __init__(self, val):
            assert(val >= 0 and val < 8)
            self.value = val

        def __str__(self):
            if self.value == 0:
                return 'REQUESTLOCKO'
            elif self.value == 1:
                return 'REQUESTLOCKOD'
            elif self.value == 2:
                return 'REQUESTLOCKODR'
            elif self.value == 3:
                return 'REQUESTTRYLOCKO'
            elif self.value == 4:
                return 'REQUESTTRYLOCKOD'
            elif self.value == 5:
                return 'REQUESTTRYLOCKODR'
            elif self.value == 6:
                return 'REQUESTUNLOCK'
            elif self.value == 7:
                return 'REQUESTSTATUS'
            return None

        __repr__ = __str__

        def __hash__(self):
            return self.value

        def __cmp__(self, other):
            return cmp(self.value, other.value)

    WorkingMemoryLockRequest.REQUESTLOCKO = WorkingMemoryLockRequest(0)
    WorkingMemoryLockRequest.REQUESTLOCKOD = WorkingMemoryLockRequest(1)
    WorkingMemoryLockRequest.REQUESTLOCKODR = WorkingMemoryLockRequest(2)
    WorkingMemoryLockRequest.REQUESTTRYLOCKO = WorkingMemoryLockRequest(3)
    WorkingMemoryLockRequest.REQUESTTRYLOCKOD = WorkingMemoryLockRequest(4)
    WorkingMemoryLockRequest.REQUESTTRYLOCKODR = WorkingMemoryLockRequest(5)
    WorkingMemoryLockRequest.REQUESTUNLOCK = WorkingMemoryLockRequest(6)
    WorkingMemoryLockRequest.REQUESTSTATUS = WorkingMemoryLockRequest(7)

    _M_cast.cdl._t_WorkingMemoryLockRequest = IcePy.defineEnum('::cast::cdl::WorkingMemoryLockRequest', WorkingMemoryLockRequest, (), (WorkingMemoryLockRequest.REQUESTLOCKO, WorkingMemoryLockRequest.REQUESTLOCKOD, WorkingMemoryLockRequest.REQUESTLOCKODR, WorkingMemoryLockRequest.REQUESTTRYLOCKO, WorkingMemoryLockRequest.REQUESTTRYLOCKOD, WorkingMemoryLockRequest.REQUESTTRYLOCKODR, WorkingMemoryLockRequest.REQUESTUNLOCK, WorkingMemoryLockRequest.REQUESTSTATUS))

    _M_cast.cdl.WorkingMemoryLockRequest = WorkingMemoryLockRequest
    del WorkingMemoryLockRequest

if not _M_cast.cdl.__dict__.has_key('FilterRestriction'):
    _M_cast.cdl.FilterRestriction = Ice.createTempClass()
    class FilterRestriction(object):

        def __init__(self, val):
            assert(val >= 0 and val < 2)
            self.value = val

        def __str__(self):
            if self.value == 0:
                return 'LOCALSA'
            elif self.value == 1:
                return 'ALLSA'
            return None

        __repr__ = __str__

        def __hash__(self):
            return self.value

        def __cmp__(self, other):
            return cmp(self.value, other.value)

    FilterRestriction.LOCALSA = FilterRestriction(0)
    FilterRestriction.ALLSA = FilterRestriction(1)

    _M_cast.cdl._t_FilterRestriction = IcePy.defineEnum('::cast::cdl::FilterRestriction', FilterRestriction, (), (FilterRestriction.LOCALSA, FilterRestriction.ALLSA))

    _M_cast.cdl.FilterRestriction = FilterRestriction
    del FilterRestriction

if not _M_cast.cdl.__dict__.has_key('ReceiverDeleteCondition'):
    _M_cast.cdl.ReceiverDeleteCondition = Ice.createTempClass()
    class ReceiverDeleteCondition(object):

        def __init__(self, val):
            assert(val >= 0 and val < 2)
            self.value = val

        def __str__(self):
            if self.value == 0:
                return 'DELETERECEIVER'
            elif self.value == 1:
                return 'DONOTDELETERECEIVER'
            return None

        __repr__ = __str__

        def __hash__(self):
            return self.value

        def __cmp__(self, other):
            return cmp(self.value, other.value)

    ReceiverDeleteCondition.DELETERECEIVER = ReceiverDeleteCondition(0)
    ReceiverDeleteCondition.DONOTDELETERECEIVER = ReceiverDeleteCondition(1)

    _M_cast.cdl._t_ReceiverDeleteCondition = IcePy.defineEnum('::cast::cdl::ReceiverDeleteCondition', ReceiverDeleteCondition, (), (ReceiverDeleteCondition.DELETERECEIVER, ReceiverDeleteCondition.DONOTDELETERECEIVER))

    _M_cast.cdl.ReceiverDeleteCondition = ReceiverDeleteCondition
    del ReceiverDeleteCondition

if not _M_cast.cdl.__dict__.has_key('_t_StringSeq'):
    _M_cast.cdl._t_StringSeq = IcePy.defineSequence('::cast::cdl::StringSeq', (), IcePy._t_string)

if not _M_cast.cdl.__dict__.has_key('WorkingMemoryChange'):
    _M_cast.cdl.WorkingMemoryChange = Ice.createTempClass()
    class WorkingMemoryChange(object):
        def __init__(self, operation=_M_cast.cdl.WorkingMemoryOperation.ADD, src='', address=Ice._struct_marker, type='', superTypes=None):
            self.operation = operation
            self.src = src
            if address is Ice._struct_marker:
                self.address = _M_cast.cdl.WorkingMemoryAddress()
            else:
                self.address = address
            self.type = type
            self.superTypes = superTypes

        def __hash__(self):
            _h = 0
            _h = 5 * _h + __builtin__.hash(self.operation)
            _h = 5 * _h + __builtin__.hash(self.src)
            _h = 5 * _h + __builtin__.hash(self.address)
            _h = 5 * _h + __builtin__.hash(self.type)
            if self.superTypes:
                for _i0 in self.superTypes:
                    _h = 5 * _h + __builtin__.hash(_i0)
            return _h % 0x7fffffff

        def __cmp__(self, other):
            if other == None:
                return 1
            if self.operation < other.operation:
                return -1
            elif self.operation > other.operation:
                return 1
            if self.src < other.src:
                return -1
            elif self.src > other.src:
                return 1
            if self.address < other.address:
                return -1
            elif self.address > other.address:
                return 1
            if self.type < other.type:
                return -1
            elif self.type > other.type:
                return 1
            if self.superTypes < other.superTypes:
                return -1
            elif self.superTypes > other.superTypes:
                return 1
            return 0

        def __str__(self):
            return IcePy.stringify(self, _M_cast.cdl._t_WorkingMemoryChange)

        __repr__ = __str__

    _M_cast.cdl._t_WorkingMemoryChange = IcePy.defineStruct('::cast::cdl::WorkingMemoryChange', WorkingMemoryChange, (), (
        ('operation', (), _M_cast.cdl._t_WorkingMemoryOperation),
        ('src', (), IcePy._t_string),
        ('address', (), _M_cast.cdl._t_WorkingMemoryAddress),
        ('type', (), IcePy._t_string),
        ('superTypes', (), _M_cast.cdl._t_StringSeq)
    ))

    _M_cast.cdl.WorkingMemoryChange = WorkingMemoryChange
    del WorkingMemoryChange

if not _M_cast.cdl.__dict__.has_key('WorkingMemoryChangeFilter'):
    _M_cast.cdl.WorkingMemoryChangeFilter = Ice.createTempClass()
    class WorkingMemoryChangeFilter(object):
        def __init__(self, operation=_M_cast.cdl.WorkingMemoryOperation.ADD, src='', address=Ice._struct_marker, type='', restriction=_M_cast.cdl.FilterRestriction.LOCALSA, origin=''):
            self.operation = operation
            self.src = src
            if address is Ice._struct_marker:
                self.address = _M_cast.cdl.WorkingMemoryAddress()
            else:
                self.address = address
            self.type = type
            self.restriction = restriction
            self.origin = origin

        def __hash__(self):
            _h = 0
            _h = 5 * _h + __builtin__.hash(self.operation)
            _h = 5 * _h + __builtin__.hash(self.src)
            _h = 5 * _h + __builtin__.hash(self.address)
            _h = 5 * _h + __builtin__.hash(self.type)
            _h = 5 * _h + __builtin__.hash(self.restriction)
            _h = 5 * _h + __builtin__.hash(self.origin)
            return _h % 0x7fffffff

        def __cmp__(self, other):
            if other == None:
                return 1
            if self.operation < other.operation:
                return -1
            elif self.operation > other.operation:
                return 1
            if self.src < other.src:
                return -1
            elif self.src > other.src:
                return 1
            if self.address < other.address:
                return -1
            elif self.address > other.address:
                return 1
            if self.type < other.type:
                return -1
            elif self.type > other.type:
                return 1
            if self.restriction < other.restriction:
                return -1
            elif self.restriction > other.restriction:
                return 1
            if self.origin < other.origin:
                return -1
            elif self.origin > other.origin:
                return 1
            return 0

        def __str__(self):
            return IcePy.stringify(self, _M_cast.cdl._t_WorkingMemoryChangeFilter)

        __repr__ = __str__

    _M_cast.cdl._t_WorkingMemoryChangeFilter = IcePy.defineStruct('::cast::cdl::WorkingMemoryChangeFilter', WorkingMemoryChangeFilter, (), (
        ('operation', (), _M_cast.cdl._t_WorkingMemoryOperation),
        ('src', (), IcePy._t_string),
        ('address', (), _M_cast.cdl._t_WorkingMemoryAddress),
        ('type', (), IcePy._t_string),
        ('restriction', (), _M_cast.cdl._t_FilterRestriction),
        ('origin', (), IcePy._t_string)
    ))

    _M_cast.cdl.WorkingMemoryChangeFilter = WorkingMemoryChangeFilter
    del WorkingMemoryChangeFilter

if not _M_cast.cdl.__dict__.has_key('CASTTime'):
    _M_cast.cdl.CASTTime = Ice.createTempClass()
    class CASTTime(object):
        def __init__(self, s=0, us=0):
            self.s = s
            self.us = us

        def __hash__(self):
            _h = 0
            _h = 5 * _h + __builtin__.hash(self.s)
            _h = 5 * _h + __builtin__.hash(self.us)
            return _h % 0x7fffffff

        def __cmp__(self, other):
            if other == None:
                return 1
            if self.s < other.s:
                return -1
            elif self.s > other.s:
                return 1
            if self.us < other.us:
                return -1
            elif self.us > other.us:
                return 1
            return 0

        def __str__(self):
            return IcePy.stringify(self, _M_cast.cdl._t_CASTTime)

        __repr__ = __str__

    _M_cast.cdl._t_CASTTime = IcePy.defineStruct('::cast::cdl::CASTTime', CASTTime, (), (
        ('s', (), IcePy._t_long),
        ('us', (), IcePy._t_long)
    ))

    _M_cast.cdl.CASTTime = CASTTime
    del CASTTime

if not _M_cast.cdl.__dict__.has_key('TaskOutcome'):
    _M_cast.cdl.TaskOutcome = Ice.createTempClass()
    class TaskOutcome(object):

        def __init__(self, val):
            assert(val >= 0 and val < 4)
            self.value = val

        def __str__(self):
            if self.value == 0:
                return 'ProcessingIncomplete'
            elif self.value == 1:
                return 'ProcessingComplete'
            elif self.value == 2:
                return 'ProcessingCompleteSuccess'
            elif self.value == 3:
                return 'ProcessingCompleteFailure'
            return None

        __repr__ = __str__

        def __hash__(self):
            return self.value

        def __cmp__(self, other):
            return cmp(self.value, other.value)

    TaskOutcome.ProcessingIncomplete = TaskOutcome(0)
    TaskOutcome.ProcessingComplete = TaskOutcome(1)
    TaskOutcome.ProcessingCompleteSuccess = TaskOutcome(2)
    TaskOutcome.ProcessingCompleteFailure = TaskOutcome(3)

    _M_cast.cdl._t_TaskOutcome = IcePy.defineEnum('::cast::cdl::TaskOutcome', TaskOutcome, (), (TaskOutcome.ProcessingIncomplete, TaskOutcome.ProcessingComplete, TaskOutcome.ProcessingCompleteSuccess, TaskOutcome.ProcessingCompleteFailure))

    _M_cast.cdl.TaskOutcome = TaskOutcome
    del TaskOutcome

if not _M_cast.cdl.__dict__.has_key('TaskManagementDecision'):
    _M_cast.cdl.TaskManagementDecision = Ice.createTempClass()
    class TaskManagementDecision(object):

        def __init__(self, val):
            assert(val >= 0 and val < 3)
            self.value = val

        def __str__(self):
            if self.value == 0:
                return 'TaskAdopted'
            elif self.value == 1:
                return 'TaskRejected'
            elif self.value == 2:
                return 'TaskWaiting'
            return None

        __repr__ = __str__

        def __hash__(self):
            return self.value

        def __cmp__(self, other):
            return cmp(self.value, other.value)

    TaskManagementDecision.TaskAdopted = TaskManagementDecision(0)
    TaskManagementDecision.TaskRejected = TaskManagementDecision(1)
    TaskManagementDecision.TaskWaiting = TaskManagementDecision(2)

    _M_cast.cdl._t_TaskManagementDecision = IcePy.defineEnum('::cast::cdl::TaskManagementDecision', TaskManagementDecision, (), (TaskManagementDecision.TaskAdopted, TaskManagementDecision.TaskRejected, TaskManagementDecision.TaskWaiting))

    _M_cast.cdl.TaskManagementDecision = TaskManagementDecision
    del TaskManagementDecision

# Start of module cast.cdl.testing
_M_cast.cdl.testing = Ice.openModule('cast.cdl.testing')
__name__ = 'cast.cdl.testing'

_M_cast.cdl.testing.CASTTESTPASS = 29

_M_cast.cdl.testing.CASTTESTFAIL = 30

if not _M_cast.cdl.testing.__dict__.has_key('CASTTestStruct'):
    _M_cast.cdl.testing.CASTTestStruct = Ice.createTempClass()
    class CASTTestStruct(Ice.Object):
        def __init__(self, count=0, change=Ice._struct_marker):
            self.count = count
            if change is Ice._struct_marker:
                self.change = _M_cast.cdl.WorkingMemoryChange()
            else:
                self.change = change

        def ice_ids(self, current=None):
            return ('::Ice::Object', '::cast::cdl::testing::CASTTestStruct')

        def ice_id(self, current=None):
            return '::cast::cdl::testing::CASTTestStruct'

        def ice_staticId():
            return '::cast::cdl::testing::CASTTestStruct'
        ice_staticId = staticmethod(ice_staticId)

        def __str__(self):
            return IcePy.stringify(self, _M_cast.cdl.testing._t_CASTTestStruct)

        __repr__ = __str__

    _M_cast.cdl.testing.CASTTestStructPrx = Ice.createTempClass()
    class CASTTestStructPrx(Ice.ObjectPrx):

        def checkedCast(proxy, facetOrCtx=None, _ctx=None):
            return _M_cast.cdl.testing.CASTTestStructPrx.ice_checkedCast(proxy, '::cast::cdl::testing::CASTTestStruct', facetOrCtx, _ctx)
        checkedCast = staticmethod(checkedCast)

        def uncheckedCast(proxy, facet=None):
            return _M_cast.cdl.testing.CASTTestStructPrx.ice_uncheckedCast(proxy, facet)
        uncheckedCast = staticmethod(uncheckedCast)

    _M_cast.cdl.testing._t_CASTTestStructPrx = IcePy.defineProxy('::cast::cdl::testing::CASTTestStruct', CASTTestStructPrx)

    _M_cast.cdl.testing._t_CASTTestStruct = IcePy.defineClass('::cast::cdl::testing::CASTTestStruct', CASTTestStruct, (), False, None, (), (
        ('count', (), IcePy._t_long),
        ('change', (), _M_cast.cdl._t_WorkingMemoryChange)
    ))
    CASTTestStruct.ice_type = _M_cast.cdl.testing._t_CASTTestStruct

    _M_cast.cdl.testing.CASTTestStruct = CASTTestStruct
    del CASTTestStruct

    _M_cast.cdl.testing.CASTTestStructPrx = CASTTestStructPrx
    del CASTTestStructPrx

if not _M_cast.cdl.testing.__dict__.has_key('TestDummyStruct'):
    _M_cast.cdl.testing.TestDummyStruct = Ice.createTempClass()
    class TestDummyStruct(Ice.Object):
        def __init__(self, dummy=''):
            self.dummy = dummy

        def ice_ids(self, current=None):
            return ('::Ice::Object', '::cast::cdl::testing::TestDummyStruct')

        def ice_id(self, current=None):
            return '::cast::cdl::testing::TestDummyStruct'

        def ice_staticId():
            return '::cast::cdl::testing::TestDummyStruct'
        ice_staticId = staticmethod(ice_staticId)

        def __str__(self):
            return IcePy.stringify(self, _M_cast.cdl.testing._t_TestDummyStruct)

        __repr__ = __str__

    _M_cast.cdl.testing.TestDummyStructPrx = Ice.createTempClass()
    class TestDummyStructPrx(Ice.ObjectPrx):

        def checkedCast(proxy, facetOrCtx=None, _ctx=None):
            return _M_cast.cdl.testing.TestDummyStructPrx.ice_checkedCast(proxy, '::cast::cdl::testing::TestDummyStruct', facetOrCtx, _ctx)
        checkedCast = staticmethod(checkedCast)

        def uncheckedCast(proxy, facet=None):
            return _M_cast.cdl.testing.TestDummyStructPrx.ice_uncheckedCast(proxy, facet)
        uncheckedCast = staticmethod(uncheckedCast)

    _M_cast.cdl.testing._t_TestDummyStructPrx = IcePy.defineProxy('::cast::cdl::testing::TestDummyStruct', TestDummyStructPrx)

    _M_cast.cdl.testing._t_TestDummyStruct = IcePy.defineClass('::cast::cdl::testing::TestDummyStruct', TestDummyStruct, (), False, None, (), (('dummy', (), IcePy._t_string),))
    TestDummyStruct.ice_type = _M_cast.cdl.testing._t_TestDummyStruct

    _M_cast.cdl.testing.TestDummyStruct = TestDummyStruct
    del TestDummyStruct

    _M_cast.cdl.testing.TestDummyStructPrx = TestDummyStructPrx
    del TestDummyStructPrx

# End of module cast.cdl.testing

__name__ = 'cast.cdl'

# End of module cast.cdl

__name__ = 'cast'

if not _M_cast.__dict__.has_key('CASTException'):
    _M_cast.CASTException = Ice.createTempClass()
    class CASTException(Ice.UserException):
        def __init__(self, message=''):
            self.message = message

        def ice_name(self):
            return 'cast::CASTException'

        def __str__(self):
            return IcePy.stringifyException(self)

        __repr__ = __str__

    _M_cast._t_CASTException = IcePy.defineException('::cast::CASTException', CASTException, (), None, (('message', (), IcePy._t_string),))
    CASTException.ice_type = _M_cast._t_CASTException

    _M_cast.CASTException = CASTException
    del CASTException

if not _M_cast.__dict__.has_key('ComponentCreationException'):
    _M_cast.ComponentCreationException = Ice.createTempClass()
    class ComponentCreationException(_M_cast.CASTException):
        def __init__(self, message=''):
            _M_cast.CASTException.__init__(self, message)

        def ice_name(self):
            return 'cast::ComponentCreationException'

        def __str__(self):
            return IcePy.stringifyException(self)

        __repr__ = __str__

    _M_cast._t_ComponentCreationException = IcePy.defineException('::cast::ComponentCreationException', ComponentCreationException, (), _M_cast._t_CASTException, ())
    ComponentCreationException.ice_type = _M_cast._t_ComponentCreationException

    _M_cast.ComponentCreationException = ComponentCreationException
    del ComponentCreationException

if not _M_cast.__dict__.has_key('SubarchitectureComponentException'):
    _M_cast.SubarchitectureComponentException = Ice.createTempClass()
    class SubarchitectureComponentException(_M_cast.CASTException):
        def __init__(self, message=''):
            _M_cast.CASTException.__init__(self, message)

        def ice_name(self):
            return 'cast::SubarchitectureComponentException'

        def __str__(self):
            return IcePy.stringifyException(self)

        __repr__ = __str__

    _M_cast._t_SubarchitectureComponentException = IcePy.defineException('::cast::SubarchitectureComponentException', SubarchitectureComponentException, (), _M_cast._t_CASTException, ())
    SubarchitectureComponentException.ice_type = _M_cast._t_SubarchitectureComponentException

    _M_cast.SubarchitectureComponentException = SubarchitectureComponentException
    del SubarchitectureComponentException

if not _M_cast.__dict__.has_key('UnknownSubarchitectureException'):
    _M_cast.UnknownSubarchitectureException = Ice.createTempClass()
    class UnknownSubarchitectureException(_M_cast.SubarchitectureComponentException):
        def __init__(self, message='', subarchitecture=''):
            _M_cast.SubarchitectureComponentException.__init__(self, message)
            self.subarchitecture = subarchitecture

        def ice_name(self):
            return 'cast::UnknownSubarchitectureException'

        def __str__(self):
            return IcePy.stringifyException(self)

        __repr__ = __str__

    _M_cast._t_UnknownSubarchitectureException = IcePy.defineException('::cast::UnknownSubarchitectureException', UnknownSubarchitectureException, (), _M_cast._t_SubarchitectureComponentException, (('subarchitecture', (), IcePy._t_string),))
    UnknownSubarchitectureException.ice_type = _M_cast._t_UnknownSubarchitectureException

    _M_cast.UnknownSubarchitectureException = UnknownSubarchitectureException
    del UnknownSubarchitectureException

if not _M_cast.__dict__.has_key('WMException'):
    _M_cast.WMException = Ice.createTempClass()
    class WMException(_M_cast.SubarchitectureComponentException):
        def __init__(self, message='', wma=Ice._struct_marker):
            _M_cast.SubarchitectureComponentException.__init__(self, message)
            if wma is Ice._struct_marker:
                self.wma = _M_cast.cdl.WorkingMemoryAddress()
            else:
                self.wma = wma

        def ice_name(self):
            return 'cast::WMException'

        def __str__(self):
            return IcePy.stringifyException(self)

        __repr__ = __str__

    _M_cast._t_WMException = IcePy.defineException('::cast::WMException', WMException, (), _M_cast._t_SubarchitectureComponentException, (('wma', (), _M_cast.cdl._t_WorkingMemoryAddress),))
    WMException.ice_type = _M_cast._t_WMException

    _M_cast.WMException = WMException
    del WMException

if not _M_cast.__dict__.has_key('DoesNotExistOnWMException'):
    _M_cast.DoesNotExistOnWMException = Ice.createTempClass()
    class DoesNotExistOnWMException(_M_cast.WMException):
        def __init__(self, message='', wma=Ice._struct_marker):
            _M_cast.WMException.__init__(self, message, wma)

        def ice_name(self):
            return 'cast::DoesNotExistOnWMException'

        def __str__(self):
            return IcePy.stringifyException(self)

        __repr__ = __str__

    _M_cast._t_DoesNotExistOnWMException = IcePy.defineException('::cast::DoesNotExistOnWMException', DoesNotExistOnWMException, (), _M_cast._t_WMException, ())
    DoesNotExistOnWMException.ice_type = _M_cast._t_DoesNotExistOnWMException

    _M_cast.DoesNotExistOnWMException = DoesNotExistOnWMException
    del DoesNotExistOnWMException

if not _M_cast.__dict__.has_key('AlreadyExistsOnWMException'):
    _M_cast.AlreadyExistsOnWMException = Ice.createTempClass()
    class AlreadyExistsOnWMException(_M_cast.WMException):
        def __init__(self, message='', wma=Ice._struct_marker):
            _M_cast.WMException.__init__(self, message, wma)

        def ice_name(self):
            return 'cast::AlreadyExistsOnWMException'

        def __str__(self):
            return IcePy.stringifyException(self)

        __repr__ = __str__

    _M_cast._t_AlreadyExistsOnWMException = IcePy.defineException('::cast::AlreadyExistsOnWMException', AlreadyExistsOnWMException, (), _M_cast._t_WMException, ())
    AlreadyExistsOnWMException.ice_type = _M_cast._t_AlreadyExistsOnWMException

    _M_cast.AlreadyExistsOnWMException = AlreadyExistsOnWMException
    del AlreadyExistsOnWMException

if not _M_cast.__dict__.has_key('ConsistencyException'):
    _M_cast.ConsistencyException = Ice.createTempClass()
    class ConsistencyException(_M_cast.WMException):
        def __init__(self, message='', wma=Ice._struct_marker):
            _M_cast.WMException.__init__(self, message, wma)

        def ice_name(self):
            return 'cast::ConsistencyException'

        def __str__(self):
            return IcePy.stringifyException(self)

        __repr__ = __str__

    _M_cast._t_ConsistencyException = IcePy.defineException('::cast::ConsistencyException', ConsistencyException, (), _M_cast._t_WMException, ())
    ConsistencyException.ice_type = _M_cast._t_ConsistencyException

    _M_cast.ConsistencyException = ConsistencyException
    del ConsistencyException

if not _M_cast.__dict__.has_key('PermissionException'):
    _M_cast.PermissionException = Ice.createTempClass()
    class PermissionException(_M_cast.WMException):
        def __init__(self, message='', wma=Ice._struct_marker):
            _M_cast.WMException.__init__(self, message, wma)

        def ice_name(self):
            return 'cast::PermissionException'

        def __str__(self):
            return IcePy.stringifyException(self)

        __repr__ = __str__

    _M_cast._t_PermissionException = IcePy.defineException('::cast::PermissionException', PermissionException, (), _M_cast._t_WMException, ())
    PermissionException.ice_type = _M_cast._t_PermissionException

    _M_cast.PermissionException = PermissionException
    del PermissionException

# Start of module cast.interfaces
_M_cast.interfaces = Ice.openModule('cast.interfaces')
__name__ = 'cast.interfaces'

if not _M_cast.interfaces.__dict__.has_key('WorkingMemory'):
    _M_cast.interfaces._t_WorkingMemory = IcePy.declareClass('::cast::interfaces::WorkingMemory')
    _M_cast.interfaces._t_WorkingMemoryPrx = IcePy.declareProxy('::cast::interfaces::WorkingMemory')

if not _M_cast.interfaces.__dict__.has_key('TaskManager'):
    _M_cast.interfaces._t_TaskManager = IcePy.declareClass('::cast::interfaces::TaskManager')
    _M_cast.interfaces._t_TaskManagerPrx = IcePy.declareProxy('::cast::interfaces::TaskManager')

if not _M_cast.interfaces.__dict__.has_key('ComponentManager'):
    _M_cast.interfaces._t_ComponentManager = IcePy.declareClass('::cast::interfaces::ComponentManager')
    _M_cast.interfaces._t_ComponentManagerPrx = IcePy.declareProxy('::cast::interfaces::ComponentManager')

if not _M_cast.interfaces.__dict__.has_key('TimeServer'):
    _M_cast.interfaces.TimeServer = Ice.createTempClass()
    class TimeServer(Ice.Object):
        def __init__(self):
            if __builtin__.type(self) == _M_cast.interfaces.TimeServer:
                raise RuntimeError('cast.interfaces.TimeServer is an abstract class')

        def ice_ids(self, current=None):
            return ('::Ice::Object', '::cast::interfaces::TimeServer')

        def ice_id(self, current=None):
            return '::cast::interfaces::TimeServer'

        def ice_staticId():
            return '::cast::interfaces::TimeServer'
        ice_staticId = staticmethod(ice_staticId)

        #
        # Operation signatures.
        #
        # def getCASTTime(self, current=None):
        # def fromTimeOfDayDouble(self, todsecs, current=None):
        # def fromTimeOfDay(self, secs, usecs, current=None):
        # def reset(self, current=None):

        def __str__(self):
            return IcePy.stringify(self, _M_cast.interfaces._t_TimeServer)

        __repr__ = __str__

    _M_cast.interfaces.TimeServerPrx = Ice.createTempClass()
    class TimeServerPrx(Ice.ObjectPrx):

        def getCASTTime(self, _ctx=None):
            return _M_cast.interfaces.TimeServer._op_getCASTTime.invoke(self, ((), _ctx))

        def fromTimeOfDayDouble(self, todsecs, _ctx=None):
            return _M_cast.interfaces.TimeServer._op_fromTimeOfDayDouble.invoke(self, ((todsecs, ), _ctx))

        def fromTimeOfDay(self, secs, usecs, _ctx=None):
            return _M_cast.interfaces.TimeServer._op_fromTimeOfDay.invoke(self, ((secs, usecs), _ctx))

        def reset(self, _ctx=None):
            return _M_cast.interfaces.TimeServer._op_reset.invoke(self, ((), _ctx))

        def checkedCast(proxy, facetOrCtx=None, _ctx=None):
            return _M_cast.interfaces.TimeServerPrx.ice_checkedCast(proxy, '::cast::interfaces::TimeServer', facetOrCtx, _ctx)
        checkedCast = staticmethod(checkedCast)

        def uncheckedCast(proxy, facet=None):
            return _M_cast.interfaces.TimeServerPrx.ice_uncheckedCast(proxy, facet)
        uncheckedCast = staticmethod(uncheckedCast)

    _M_cast.interfaces._t_TimeServerPrx = IcePy.defineProxy('::cast::interfaces::TimeServer', TimeServerPrx)

    _M_cast.interfaces._t_TimeServer = IcePy.defineClass('::cast::interfaces::TimeServer', TimeServer, (), True, None, (), ())
    TimeServer.ice_type = _M_cast.interfaces._t_TimeServer

    TimeServer._op_getCASTTime = IcePy.Operation('getCASTTime', Ice.OperationMode.Normal, Ice.OperationMode.Normal, False, (), (), (), _M_cast.cdl._t_CASTTime, ())
    TimeServer._op_fromTimeOfDayDouble = IcePy.Operation('fromTimeOfDayDouble', Ice.OperationMode.Normal, Ice.OperationMode.Normal, False, (), (((), IcePy._t_double),), (), _M_cast.cdl._t_CASTTime, ())
    TimeServer._op_fromTimeOfDay = IcePy.Operation('fromTimeOfDay', Ice.OperationMode.Normal, Ice.OperationMode.Normal, False, (), (((), IcePy._t_long), ((), IcePy._t_long)), (), _M_cast.cdl._t_CASTTime, ())
    TimeServer._op_reset = IcePy.Operation('reset', Ice.OperationMode.Normal, Ice.OperationMode.Normal, False, (), (), (), None, ())

    _M_cast.interfaces.TimeServer = TimeServer
    del TimeServer

    _M_cast.interfaces.TimeServerPrx = TimeServerPrx
    del TimeServerPrx

if not _M_cast.interfaces.__dict__.has_key('CASTComponent'):
    _M_cast.interfaces.CASTComponent = Ice.createTempClass()
    class CASTComponent(Ice.Object):
        def __init__(self):
            if __builtin__.type(self) == _M_cast.interfaces.CASTComponent:
                raise RuntimeError('cast.interfaces.CASTComponent is an abstract class')

        def ice_ids(self, current=None):
            return ('::Ice::Object', '::cast::interfaces::CASTComponent')

        def ice_id(self, current=None):
            return '::cast::interfaces::CASTComponent'

        def ice_staticId():
            return '::cast::interfaces::CASTComponent'
        ice_staticId = staticmethod(ice_staticId)

        #
        # Operation signatures.
        #
        # def beat(self, current=None):
        # def setID(self, id, current=None):
        # def getID(self, current=None):
        # def configure(self, config, current=None):
        # def start(self, current=None):
        # def run(self, current=None):
        # def stop(self, current=None):
        # def setComponentManager(self, man, current=None):
        # def setTimeServer(self, ts, current=None):
        # def destroy(self, current=None):

        def __str__(self):
            return IcePy.stringify(self, _M_cast.interfaces._t_CASTComponent)

        __repr__ = __str__

    _M_cast.interfaces.CASTComponentPrx = Ice.createTempClass()
    class CASTComponentPrx(Ice.ObjectPrx):

        def beat(self, _ctx=None):
            return _M_cast.interfaces.CASTComponent._op_beat.invoke(self, ((), _ctx))

        def setID(self, id, _ctx=None):
            return _M_cast.interfaces.CASTComponent._op_setID.invoke(self, ((id, ), _ctx))

        def getID(self, _ctx=None):
            return _M_cast.interfaces.CASTComponent._op_getID.invoke(self, ((), _ctx))

        def configure(self, config, _ctx=None):
            return _M_cast.interfaces.CASTComponent._op_configure.invoke(self, ((config, ), _ctx))

        def start(self, _ctx=None):
            return _M_cast.interfaces.CASTComponent._op_start.invoke(self, ((), _ctx))

        def run(self, _ctx=None):
            return _M_cast.interfaces.CASTComponent._op_run.invoke(self, ((), _ctx))

        def stop(self, _ctx=None):
            return _M_cast.interfaces.CASTComponent._op_stop.invoke(self, ((), _ctx))

        def setComponentManager(self, man, _ctx=None):
            return _M_cast.interfaces.CASTComponent._op_setComponentManager.invoke(self, ((man, ), _ctx))

        def setTimeServer(self, ts, _ctx=None):
            return _M_cast.interfaces.CASTComponent._op_setTimeServer.invoke(self, ((ts, ), _ctx))

        def destroy(self, _ctx=None):
            return _M_cast.interfaces.CASTComponent._op_destroy.invoke(self, ((), _ctx))

        def checkedCast(proxy, facetOrCtx=None, _ctx=None):
            return _M_cast.interfaces.CASTComponentPrx.ice_checkedCast(proxy, '::cast::interfaces::CASTComponent', facetOrCtx, _ctx)
        checkedCast = staticmethod(checkedCast)

        def uncheckedCast(proxy, facet=None):
            return _M_cast.interfaces.CASTComponentPrx.ice_uncheckedCast(proxy, facet)
        uncheckedCast = staticmethod(uncheckedCast)

    _M_cast.interfaces._t_CASTComponentPrx = IcePy.defineProxy('::cast::interfaces::CASTComponent', CASTComponentPrx)

    _M_cast.interfaces._t_CASTComponent = IcePy.defineClass('::cast::interfaces::CASTComponent', CASTComponent, (), True, None, (), ())
    CASTComponent.ice_type = _M_cast.interfaces._t_CASTComponent

    CASTComponent._op_beat = IcePy.Operation('beat', Ice.OperationMode.Idempotent, Ice.OperationMode.Idempotent, False, (), (), (), None, ())
    CASTComponent._op_setID = IcePy.Operation('setID', Ice.OperationMode.Normal, Ice.OperationMode.Normal, False, (), (((), IcePy._t_string),), (), None, ())
    CASTComponent._op_getID = IcePy.Operation('getID', Ice.OperationMode.Idempotent, Ice.OperationMode.Idempotent, False, (), (), (), IcePy._t_string, ())
    CASTComponent._op_configure = IcePy.Operation('configure', Ice.OperationMode.Normal, Ice.OperationMode.Normal, False, (), (((), _M_cast.cdl._t_StringMap),), (), None, ())
    CASTComponent._op_start = IcePy.Operation('start', Ice.OperationMode.Normal, Ice.OperationMode.Normal, False, (), (), (), None, ())
    CASTComponent._op_run = IcePy.Operation('run', Ice.OperationMode.Normal, Ice.OperationMode.Normal, False, (), (), (), None, ())
    CASTComponent._op_stop = IcePy.Operation('stop', Ice.OperationMode.Normal, Ice.OperationMode.Normal, False, (), (), (), None, ())
    CASTComponent._op_setComponentManager = IcePy.Operation('setComponentManager', Ice.OperationMode.Normal, Ice.OperationMode.Normal, False, (), (((), _M_cast.interfaces._t_ComponentManagerPrx),), (), None, ())
    CASTComponent._op_setTimeServer = IcePy.Operation('setTimeServer', Ice.OperationMode.Normal, Ice.OperationMode.Normal, False, (), (((), _M_cast.interfaces._t_TimeServerPrx),), (), None, ())
    CASTComponent._op_destroy = IcePy.Operation('destroy', Ice.OperationMode.Normal, Ice.OperationMode.Normal, False, (), (), (), None, ())

    _M_cast.interfaces.CASTComponent = CASTComponent
    del CASTComponent

    _M_cast.interfaces.CASTComponentPrx = CASTComponentPrx
    del CASTComponentPrx

if not _M_cast.interfaces.__dict__.has_key('WorkingMemoryAttachedComponent'):
    _M_cast.interfaces.WorkingMemoryAttachedComponent = Ice.createTempClass()
    class WorkingMemoryAttachedComponent(_M_cast.interfaces.CASTComponent):
        def __init__(self):
            if __builtin__.type(self) == _M_cast.interfaces.WorkingMemoryAttachedComponent:
                raise RuntimeError('cast.interfaces.WorkingMemoryAttachedComponent is an abstract class')

        def ice_ids(self, current=None):
            return ('::Ice::Object', '::cast::interfaces::CASTComponent', '::cast::interfaces::WorkingMemoryAttachedComponent')

        def ice_id(self, current=None):
            return '::cast::interfaces::WorkingMemoryAttachedComponent'

        def ice_staticId():
            return '::cast::interfaces::WorkingMemoryAttachedComponent'
        ice_staticId = staticmethod(ice_staticId)

        #
        # Operation signatures.
        #
        # def setWorkingMemory(self, wm, current=None):

        def __str__(self):
            return IcePy.stringify(self, _M_cast.interfaces._t_WorkingMemoryAttachedComponent)

        __repr__ = __str__

    _M_cast.interfaces.WorkingMemoryAttachedComponentPrx = Ice.createTempClass()
    class WorkingMemoryAttachedComponentPrx(_M_cast.interfaces.CASTComponentPrx):

        def setWorkingMemory(self, wm, _ctx=None):
            return _M_cast.interfaces.WorkingMemoryAttachedComponent._op_setWorkingMemory.invoke(self, ((wm, ), _ctx))

        def checkedCast(proxy, facetOrCtx=None, _ctx=None):
            return _M_cast.interfaces.WorkingMemoryAttachedComponentPrx.ice_checkedCast(proxy, '::cast::interfaces::WorkingMemoryAttachedComponent', facetOrCtx, _ctx)
        checkedCast = staticmethod(checkedCast)

        def uncheckedCast(proxy, facet=None):
            return _M_cast.interfaces.WorkingMemoryAttachedComponentPrx.ice_uncheckedCast(proxy, facet)
        uncheckedCast = staticmethod(uncheckedCast)

    _M_cast.interfaces._t_WorkingMemoryAttachedComponentPrx = IcePy.defineProxy('::cast::interfaces::WorkingMemoryAttachedComponent', WorkingMemoryAttachedComponentPrx)

    _M_cast.interfaces._t_WorkingMemoryAttachedComponent = IcePy.defineClass('::cast::interfaces::WorkingMemoryAttachedComponent', WorkingMemoryAttachedComponent, (), True, None, (_M_cast.interfaces._t_CASTComponent,), ())
    WorkingMemoryAttachedComponent.ice_type = _M_cast.interfaces._t_WorkingMemoryAttachedComponent

    WorkingMemoryAttachedComponent._op_setWorkingMemory = IcePy.Operation('setWorkingMemory', Ice.OperationMode.Normal, Ice.OperationMode.Normal, False, (), (((), _M_cast.interfaces._t_WorkingMemoryPrx),), (), None, ())

    _M_cast.interfaces.WorkingMemoryAttachedComponent = WorkingMemoryAttachedComponent
    del WorkingMemoryAttachedComponent

    _M_cast.interfaces.WorkingMemoryAttachedComponentPrx = WorkingMemoryAttachedComponentPrx
    del WorkingMemoryAttachedComponentPrx

if not _M_cast.interfaces.__dict__.has_key('WorkingMemoryReaderComponent'):
    _M_cast.interfaces.WorkingMemoryReaderComponent = Ice.createTempClass()
    class WorkingMemoryReaderComponent(_M_cast.interfaces.WorkingMemoryAttachedComponent):
        def __init__(self):
            if __builtin__.type(self) == _M_cast.interfaces.WorkingMemoryReaderComponent:
                raise RuntimeError('cast.interfaces.WorkingMemoryReaderComponent is an abstract class')

        def ice_ids(self, current=None):
            return ('::Ice::Object', '::cast::interfaces::CASTComponent', '::cast::interfaces::WorkingMemoryAttachedComponent', '::cast::interfaces::WorkingMemoryReaderComponent')

        def ice_id(self, current=None):
            return '::cast::interfaces::WorkingMemoryReaderComponent'

        def ice_staticId():
            return '::cast::interfaces::WorkingMemoryReaderComponent'
        ice_staticId = staticmethod(ice_staticId)

        #
        # Operation signatures.
        #
        # def receiveChangeEvent(self, wmc, current=None):

        def __str__(self):
            return IcePy.stringify(self, _M_cast.interfaces._t_WorkingMemoryReaderComponent)

        __repr__ = __str__

    _M_cast.interfaces.WorkingMemoryReaderComponentPrx = Ice.createTempClass()
    class WorkingMemoryReaderComponentPrx(_M_cast.interfaces.WorkingMemoryAttachedComponentPrx):

        def receiveChangeEvent(self, wmc, _ctx=None):
            return _M_cast.interfaces.WorkingMemoryReaderComponent._op_receiveChangeEvent.invoke(self, ((wmc, ), _ctx))

        def checkedCast(proxy, facetOrCtx=None, _ctx=None):
            return _M_cast.interfaces.WorkingMemoryReaderComponentPrx.ice_checkedCast(proxy, '::cast::interfaces::WorkingMemoryReaderComponent', facetOrCtx, _ctx)
        checkedCast = staticmethod(checkedCast)

        def uncheckedCast(proxy, facet=None):
            return _M_cast.interfaces.WorkingMemoryReaderComponentPrx.ice_uncheckedCast(proxy, facet)
        uncheckedCast = staticmethod(uncheckedCast)

    _M_cast.interfaces._t_WorkingMemoryReaderComponentPrx = IcePy.defineProxy('::cast::interfaces::WorkingMemoryReaderComponent', WorkingMemoryReaderComponentPrx)

    _M_cast.interfaces._t_WorkingMemoryReaderComponent = IcePy.defineClass('::cast::interfaces::WorkingMemoryReaderComponent', WorkingMemoryReaderComponent, (), True, None, (_M_cast.interfaces._t_WorkingMemoryAttachedComponent,), ())
    WorkingMemoryReaderComponent.ice_type = _M_cast.interfaces._t_WorkingMemoryReaderComponent

    WorkingMemoryReaderComponent._op_receiveChangeEvent = IcePy.Operation('receiveChangeEvent', Ice.OperationMode.Normal, Ice.OperationMode.Normal, False, (), (((), _M_cast.cdl._t_WorkingMemoryChange),), (), None, ())

    _M_cast.interfaces.WorkingMemoryReaderComponent = WorkingMemoryReaderComponent
    del WorkingMemoryReaderComponent

    _M_cast.interfaces.WorkingMemoryReaderComponentPrx = WorkingMemoryReaderComponentPrx
    del WorkingMemoryReaderComponentPrx

if not _M_cast.interfaces.__dict__.has_key('ManagedComponent'):
    _M_cast.interfaces.ManagedComponent = Ice.createTempClass()
    class ManagedComponent(_M_cast.interfaces.WorkingMemoryReaderComponent):
        def __init__(self):
            if __builtin__.type(self) == _M_cast.interfaces.ManagedComponent:
                raise RuntimeError('cast.interfaces.ManagedComponent is an abstract class')

        def ice_ids(self, current=None):
            return ('::Ice::Object', '::cast::interfaces::CASTComponent', '::cast::interfaces::ManagedComponent', '::cast::interfaces::WorkingMemoryAttachedComponent', '::cast::interfaces::WorkingMemoryReaderComponent')

        def ice_id(self, current=None):
            return '::cast::interfaces::ManagedComponent'

        def ice_staticId():
            return '::cast::interfaces::ManagedComponent'
        ice_staticId = staticmethod(ice_staticId)

        #
        # Operation signatures.
        #
        # def setTaskManager(self, tm, current=None):
        # def taskDecision(self, id, decision, current=None):

        def __str__(self):
            return IcePy.stringify(self, _M_cast.interfaces._t_ManagedComponent)

        __repr__ = __str__

    _M_cast.interfaces.ManagedComponentPrx = Ice.createTempClass()
    class ManagedComponentPrx(_M_cast.interfaces.WorkingMemoryReaderComponentPrx):

        def setTaskManager(self, tm, _ctx=None):
            return _M_cast.interfaces.ManagedComponent._op_setTaskManager.invoke(self, ((tm, ), _ctx))

        def taskDecision(self, id, decision, _ctx=None):
            return _M_cast.interfaces.ManagedComponent._op_taskDecision.invoke(self, ((id, decision), _ctx))

        def checkedCast(proxy, facetOrCtx=None, _ctx=None):
            return _M_cast.interfaces.ManagedComponentPrx.ice_checkedCast(proxy, '::cast::interfaces::ManagedComponent', facetOrCtx, _ctx)
        checkedCast = staticmethod(checkedCast)

        def uncheckedCast(proxy, facet=None):
            return _M_cast.interfaces.ManagedComponentPrx.ice_uncheckedCast(proxy, facet)
        uncheckedCast = staticmethod(uncheckedCast)

    _M_cast.interfaces._t_ManagedComponentPrx = IcePy.defineProxy('::cast::interfaces::ManagedComponent', ManagedComponentPrx)

    _M_cast.interfaces._t_ManagedComponent = IcePy.defineClass('::cast::interfaces::ManagedComponent', ManagedComponent, (), True, None, (_M_cast.interfaces._t_WorkingMemoryReaderComponent,), ())
    ManagedComponent.ice_type = _M_cast.interfaces._t_ManagedComponent

    ManagedComponent._op_setTaskManager = IcePy.Operation('setTaskManager', Ice.OperationMode.Normal, Ice.OperationMode.Normal, False, (), (((), _M_cast.interfaces._t_TaskManagerPrx),), (), None, ())
    ManagedComponent._op_taskDecision = IcePy.Operation('taskDecision', Ice.OperationMode.Normal, Ice.OperationMode.Normal, False, (), (((), IcePy._t_string), ((), _M_cast.cdl._t_TaskManagementDecision)), (), None, ())

    _M_cast.interfaces.ManagedComponent = ManagedComponent
    del ManagedComponent

    _M_cast.interfaces.ManagedComponentPrx = ManagedComponentPrx
    del ManagedComponentPrx

if not _M_cast.interfaces.__dict__.has_key('UnmanagedComponent'):
    _M_cast.interfaces.UnmanagedComponent = Ice.createTempClass()
    class UnmanagedComponent(_M_cast.interfaces.WorkingMemoryAttachedComponent):
        def __init__(self):
            if __builtin__.type(self) == _M_cast.interfaces.UnmanagedComponent:
                raise RuntimeError('cast.interfaces.UnmanagedComponent is an abstract class')

        def ice_ids(self, current=None):
            return ('::Ice::Object', '::cast::interfaces::CASTComponent', '::cast::interfaces::UnmanagedComponent', '::cast::interfaces::WorkingMemoryAttachedComponent')

        def ice_id(self, current=None):
            return '::cast::interfaces::UnmanagedComponent'

        def ice_staticId():
            return '::cast::interfaces::UnmanagedComponent'
        ice_staticId = staticmethod(ice_staticId)

        def __str__(self):
            return IcePy.stringify(self, _M_cast.interfaces._t_UnmanagedComponent)

        __repr__ = __str__

    _M_cast.interfaces.UnmanagedComponentPrx = Ice.createTempClass()
    class UnmanagedComponentPrx(_M_cast.interfaces.WorkingMemoryAttachedComponentPrx):

        def checkedCast(proxy, facetOrCtx=None, _ctx=None):
            return _M_cast.interfaces.UnmanagedComponentPrx.ice_checkedCast(proxy, '::cast::interfaces::UnmanagedComponent', facetOrCtx, _ctx)
        checkedCast = staticmethod(checkedCast)

        def uncheckedCast(proxy, facet=None):
            return _M_cast.interfaces.UnmanagedComponentPrx.ice_uncheckedCast(proxy, facet)
        uncheckedCast = staticmethod(uncheckedCast)

    _M_cast.interfaces._t_UnmanagedComponentPrx = IcePy.defineProxy('::cast::interfaces::UnmanagedComponent', UnmanagedComponentPrx)

    _M_cast.interfaces._t_UnmanagedComponent = IcePy.defineClass('::cast::interfaces::UnmanagedComponent', UnmanagedComponent, (), True, None, (_M_cast.interfaces._t_WorkingMemoryAttachedComponent,), ())
    UnmanagedComponent.ice_type = _M_cast.interfaces._t_UnmanagedComponent

    _M_cast.interfaces.UnmanagedComponent = UnmanagedComponent
    del UnmanagedComponent

    _M_cast.interfaces.UnmanagedComponentPrx = UnmanagedComponentPrx
    del UnmanagedComponentPrx

if not _M_cast.interfaces.__dict__.has_key('TaskManager'):
    _M_cast.interfaces.TaskManager = Ice.createTempClass()
    class TaskManager(_M_cast.interfaces.WorkingMemoryReaderComponent):
        def __init__(self):
            if __builtin__.type(self) == _M_cast.interfaces.TaskManager:
                raise RuntimeError('cast.interfaces.TaskManager is an abstract class')

        def ice_ids(self, current=None):
            return ('::Ice::Object', '::cast::interfaces::CASTComponent', '::cast::interfaces::TaskManager', '::cast::interfaces::WorkingMemoryAttachedComponent', '::cast::interfaces::WorkingMemoryReaderComponent')

        def ice_id(self, current=None):
            return '::cast::interfaces::TaskManager'

        def ice_staticId():
            return '::cast::interfaces::TaskManager'
        ice_staticId = staticmethod(ice_staticId)

        #
        # Operation signatures.
        #
        # def proposeTask(self, component, taskID, taskName, current=None):
        # def retractTask(self, component, taskID, current=None):
        # def taskComplete(self, component, taskID, outcome, current=None):
        # def addManagedComponent(self, comp, current=None):

        def __str__(self):
            return IcePy.stringify(self, _M_cast.interfaces._t_TaskManager)

        __repr__ = __str__

    _M_cast.interfaces.TaskManagerPrx = Ice.createTempClass()
    class TaskManagerPrx(_M_cast.interfaces.WorkingMemoryReaderComponentPrx):

        def proposeTask(self, component, taskID, taskName, _ctx=None):
            return _M_cast.interfaces.TaskManager._op_proposeTask.invoke(self, ((component, taskID, taskName), _ctx))

        def retractTask(self, component, taskID, _ctx=None):
            return _M_cast.interfaces.TaskManager._op_retractTask.invoke(self, ((component, taskID), _ctx))

        def taskComplete(self, component, taskID, outcome, _ctx=None):
            return _M_cast.interfaces.TaskManager._op_taskComplete.invoke(self, ((component, taskID, outcome), _ctx))

        def addManagedComponent(self, comp, _ctx=None):
            return _M_cast.interfaces.TaskManager._op_addManagedComponent.invoke(self, ((comp, ), _ctx))

        def checkedCast(proxy, facetOrCtx=None, _ctx=None):
            return _M_cast.interfaces.TaskManagerPrx.ice_checkedCast(proxy, '::cast::interfaces::TaskManager', facetOrCtx, _ctx)
        checkedCast = staticmethod(checkedCast)

        def uncheckedCast(proxy, facet=None):
            return _M_cast.interfaces.TaskManagerPrx.ice_uncheckedCast(proxy, facet)
        uncheckedCast = staticmethod(uncheckedCast)

    _M_cast.interfaces._t_TaskManagerPrx = IcePy.defineProxy('::cast::interfaces::TaskManager', TaskManagerPrx)

    _M_cast.interfaces._t_TaskManager = IcePy.defineClass('::cast::interfaces::TaskManager', TaskManager, (), True, None, (_M_cast.interfaces._t_WorkingMemoryReaderComponent,), ())
    TaskManager.ice_type = _M_cast.interfaces._t_TaskManager

    TaskManager._op_proposeTask = IcePy.Operation('proposeTask', Ice.OperationMode.Normal, Ice.OperationMode.Normal, False, (), (((), IcePy._t_string), ((), IcePy._t_string), ((), IcePy._t_string)), (), None, ())
    TaskManager._op_retractTask = IcePy.Operation('retractTask', Ice.OperationMode.Normal, Ice.OperationMode.Normal, False, (), (((), IcePy._t_string), ((), IcePy._t_string)), (), None, ())
    TaskManager._op_taskComplete = IcePy.Operation('taskComplete', Ice.OperationMode.Normal, Ice.OperationMode.Normal, False, (), (((), IcePy._t_string), ((), IcePy._t_string), ((), _M_cast.cdl._t_TaskOutcome)), (), None, ())
    TaskManager._op_addManagedComponent = IcePy.Operation('addManagedComponent', Ice.OperationMode.Normal, Ice.OperationMode.Normal, False, (), (((), _M_cast.interfaces._t_ManagedComponentPrx),), (), None, ())

    _M_cast.interfaces.TaskManager = TaskManager
    del TaskManager

    _M_cast.interfaces.TaskManagerPrx = TaskManagerPrx
    del TaskManagerPrx

if not _M_cast.interfaces.__dict__.has_key('WorkingMemory'):
    _M_cast.interfaces.WorkingMemory = Ice.createTempClass()
    class WorkingMemory(_M_cast.interfaces.CASTComponent):
        def __init__(self):
            if __builtin__.type(self) == _M_cast.interfaces.WorkingMemory:
                raise RuntimeError('cast.interfaces.WorkingMemory is an abstract class')

        def ice_ids(self, current=None):
            return ('::Ice::Object', '::cast::interfaces::CASTComponent', '::cast::interfaces::WorkingMemory')

        def ice_id(self, current=None):
            return '::cast::interfaces::WorkingMemory'

        def ice_staticId():
            return '::cast::interfaces::WorkingMemory'
        ice_staticId = staticmethod(ice_staticId)

        #
        # Operation signatures.
        #
        # def exists(self, id, subarch, current=None):
        # def getVersionNumber(self, id, subarch, current=None):
        # def getPermissions(self, id, subarch, current=None):
        # def lockEntry(self, id, subarch, component, permissions, current=None):
        # def tryLockEntry(self, id, subarch, component, permissions, current=None):
        # def unlockEntry(self, id, subarch, component, current=None):
        # def setWorkingMemory(self, wm, subarch, current=None):
        # def addToWorkingMemory(self, id, subarch, type, component, entry, current=None):
        # def overwriteWorkingMemory(self, id, subarch, type, component, entry, current=None):
        # def deleteFromWorkingMemory(self, id, subarch, component, current=None):
        # def getWorkingMemoryEntry(self, id, subarch, component, current=None):
        # def getWorkingMemoryEntries(self, type, subarch, count, component, current=None):
        # def registerComponentFilter(self, filter, current=None):
        # def removeComponentFilter(self, filter, current=None):
        # def registerWorkingMemoryFilter(self, filter, subarch, current=None):
        # def removeWorkingMemoryFilter(self, filter, current=None):
        # def addReader(self, reader, current=None):
        # def receiveChangeEvent(self, wmc, current=None):

        def __str__(self):
            return IcePy.stringify(self, _M_cast.interfaces._t_WorkingMemory)

        __repr__ = __str__

    _M_cast.interfaces.WorkingMemoryPrx = Ice.createTempClass()
    class WorkingMemoryPrx(_M_cast.interfaces.CASTComponentPrx):

        def exists(self, id, subarch, _ctx=None):
            return _M_cast.interfaces.WorkingMemory._op_exists.invoke(self, ((id, subarch), _ctx))

        def getVersionNumber(self, id, subarch, _ctx=None):
            return _M_cast.interfaces.WorkingMemory._op_getVersionNumber.invoke(self, ((id, subarch), _ctx))

        def getPermissions(self, id, subarch, _ctx=None):
            return _M_cast.interfaces.WorkingMemory._op_getPermissions.invoke(self, ((id, subarch), _ctx))

        def lockEntry(self, id, subarch, component, permissions, _ctx=None):
            return _M_cast.interfaces.WorkingMemory._op_lockEntry.invoke(self, ((id, subarch, component, permissions), _ctx))

        def tryLockEntry(self, id, subarch, component, permissions, _ctx=None):
            return _M_cast.interfaces.WorkingMemory._op_tryLockEntry.invoke(self, ((id, subarch, component, permissions), _ctx))

        def unlockEntry(self, id, subarch, component, _ctx=None):
            return _M_cast.interfaces.WorkingMemory._op_unlockEntry.invoke(self, ((id, subarch, component), _ctx))

        def setWorkingMemory(self, wm, subarch, _ctx=None):
            return _M_cast.interfaces.WorkingMemory._op_setWorkingMemory.invoke(self, ((wm, subarch), _ctx))

        def addToWorkingMemory(self, id, subarch, type, component, entry, _ctx=None):
            return _M_cast.interfaces.WorkingMemory._op_addToWorkingMemory.invoke(self, ((id, subarch, type, component, entry), _ctx))

        def overwriteWorkingMemory(self, id, subarch, type, component, entry, _ctx=None):
            return _M_cast.interfaces.WorkingMemory._op_overwriteWorkingMemory.invoke(self, ((id, subarch, type, component, entry), _ctx))

        def deleteFromWorkingMemory(self, id, subarch, component, _ctx=None):
            return _M_cast.interfaces.WorkingMemory._op_deleteFromWorkingMemory.invoke(self, ((id, subarch, component), _ctx))

        def getWorkingMemoryEntry(self, id, subarch, component, _ctx=None):
            return _M_cast.interfaces.WorkingMemory._op_getWorkingMemoryEntry.invoke(self, ((id, subarch, component), _ctx))

        def getWorkingMemoryEntries(self, type, subarch, count, component, _ctx=None):
            return _M_cast.interfaces.WorkingMemory._op_getWorkingMemoryEntries.invoke(self, ((type, subarch, count, component), _ctx))

        def registerComponentFilter(self, filter, _ctx=None):
            return _M_cast.interfaces.WorkingMemory._op_registerComponentFilter.invoke(self, ((filter, ), _ctx))

        def removeComponentFilter(self, filter, _ctx=None):
            return _M_cast.interfaces.WorkingMemory._op_removeComponentFilter.invoke(self, ((filter, ), _ctx))

        def registerWorkingMemoryFilter(self, filter, subarch, _ctx=None):
            return _M_cast.interfaces.WorkingMemory._op_registerWorkingMemoryFilter.invoke(self, ((filter, subarch), _ctx))

        def removeWorkingMemoryFilter(self, filter, _ctx=None):
            return _M_cast.interfaces.WorkingMemory._op_removeWorkingMemoryFilter.invoke(self, ((filter, ), _ctx))

        def addReader(self, reader, _ctx=None):
            return _M_cast.interfaces.WorkingMemory._op_addReader.invoke(self, ((reader, ), _ctx))

        def receiveChangeEvent(self, wmc, _ctx=None):
            return _M_cast.interfaces.WorkingMemory._op_receiveChangeEvent.invoke(self, ((wmc, ), _ctx))

        def checkedCast(proxy, facetOrCtx=None, _ctx=None):
            return _M_cast.interfaces.WorkingMemoryPrx.ice_checkedCast(proxy, '::cast::interfaces::WorkingMemory', facetOrCtx, _ctx)
        checkedCast = staticmethod(checkedCast)

        def uncheckedCast(proxy, facet=None):
            return _M_cast.interfaces.WorkingMemoryPrx.ice_uncheckedCast(proxy, facet)
        uncheckedCast = staticmethod(uncheckedCast)

    _M_cast.interfaces._t_WorkingMemoryPrx = IcePy.defineProxy('::cast::interfaces::WorkingMemory', WorkingMemoryPrx)

    _M_cast.interfaces._t_WorkingMemory = IcePy.defineClass('::cast::interfaces::WorkingMemory', WorkingMemory, (), True, None, (_M_cast.interfaces._t_CASTComponent,), ())
    WorkingMemory.ice_type = _M_cast.interfaces._t_WorkingMemory

    WorkingMemory._op_exists = IcePy.Operation('exists', Ice.OperationMode.Idempotent, Ice.OperationMode.Idempotent, False, (), (((), IcePy._t_string), ((), IcePy._t_string)), (), IcePy._t_bool, (_M_cast._t_UnknownSubarchitectureException,))
    WorkingMemory._op_getVersionNumber = IcePy.Operation('getVersionNumber', Ice.OperationMode.Idempotent, Ice.OperationMode.Idempotent, False, (), (((), IcePy._t_string), ((), IcePy._t_string)), (), IcePy._t_int, (_M_cast._t_DoesNotExistOnWMException, _M_cast._t_UnknownSubarchitectureException))
    WorkingMemory._op_getPermissions = IcePy.Operation('getPermissions', Ice.OperationMode.Idempotent, Ice.OperationMode.Idempotent, False, (), (((), IcePy._t_string), ((), IcePy._t_string)), (), _M_cast.cdl._t_WorkingMemoryPermissions, (_M_cast._t_DoesNotExistOnWMException, _M_cast._t_UnknownSubarchitectureException))
    WorkingMemory._op_lockEntry = IcePy.Operation('lockEntry', Ice.OperationMode.Normal, Ice.OperationMode.Normal, False, (), (((), IcePy._t_string), ((), IcePy._t_string), ((), IcePy._t_string), ((), _M_cast.cdl._t_WorkingMemoryPermissions)), (), None, (_M_cast._t_DoesNotExistOnWMException, _M_cast._t_UnknownSubarchitectureException))
    WorkingMemory._op_tryLockEntry = IcePy.Operation('tryLockEntry', Ice.OperationMode.Normal, Ice.OperationMode.Normal, False, (), (((), IcePy._t_string), ((), IcePy._t_string), ((), IcePy._t_string), ((), _M_cast.cdl._t_WorkingMemoryPermissions)), (), IcePy._t_bool, (_M_cast._t_DoesNotExistOnWMException, _M_cast._t_UnknownSubarchitectureException))
    WorkingMemory._op_unlockEntry = IcePy.Operation('unlockEntry', Ice.OperationMode.Normal, Ice.OperationMode.Normal, False, (), (((), IcePy._t_string), ((), IcePy._t_string), ((), IcePy._t_string)), (), None, (_M_cast._t_DoesNotExistOnWMException, _M_cast._t_ConsistencyException, _M_cast._t_UnknownSubarchitectureException))
    WorkingMemory._op_setWorkingMemory = IcePy.Operation('setWorkingMemory', Ice.OperationMode.Normal, Ice.OperationMode.Normal, False, (), (((), _M_cast.interfaces._t_WorkingMemoryPrx), ((), IcePy._t_string)), (), None, ())
    WorkingMemory._op_addToWorkingMemory = IcePy.Operation('addToWorkingMemory', Ice.OperationMode.Normal, Ice.OperationMode.Normal, False, (), (((), IcePy._t_string), ((), IcePy._t_string), ((), IcePy._t_string), ((), IcePy._t_string), ((), IcePy._t_Object)), (), None, (_M_cast._t_AlreadyExistsOnWMException, _M_cast._t_UnknownSubarchitectureException))
    WorkingMemory._op_overwriteWorkingMemory = IcePy.Operation('overwriteWorkingMemory', Ice.OperationMode.Normal, Ice.OperationMode.Normal, False, (), (((), IcePy._t_string), ((), IcePy._t_string), ((), IcePy._t_string), ((), IcePy._t_string), ((), IcePy._t_Object)), (), None, (_M_cast._t_DoesNotExistOnWMException, _M_cast._t_UnknownSubarchitectureException))
    WorkingMemory._op_deleteFromWorkingMemory = IcePy.Operation('deleteFromWorkingMemory', Ice.OperationMode.Normal, Ice.OperationMode.Normal, False, (), (((), IcePy._t_string), ((), IcePy._t_string), ((), IcePy._t_string)), (), None, (_M_cast._t_DoesNotExistOnWMException, _M_cast._t_UnknownSubarchitectureException))
    WorkingMemory._op_getWorkingMemoryEntry = IcePy.Operation('getWorkingMemoryEntry', Ice.OperationMode.Normal, Ice.OperationMode.Normal, False, (), (((), IcePy._t_string), ((), IcePy._t_string), ((), IcePy._t_string)), (), _M_cast.cdl._t_WorkingMemoryEntry, (_M_cast._t_DoesNotExistOnWMException, _M_cast._t_UnknownSubarchitectureException))
    WorkingMemory._op_getWorkingMemoryEntries = IcePy.Operation('getWorkingMemoryEntries', Ice.OperationMode.Normal, Ice.OperationMode.Normal, False, (), (((), IcePy._t_string), ((), IcePy._t_string), ((), IcePy._t_int), ((), IcePy._t_string)), (((), _M_cast.cdl._t_WorkingMemoryEntrySeq),), None, (_M_cast._t_UnknownSubarchitectureException,))
    WorkingMemory._op_registerComponentFilter = IcePy.Operation('registerComponentFilter', Ice.OperationMode.Normal, Ice.OperationMode.Normal, False, (), (((), _M_cast.cdl._t_WorkingMemoryChangeFilter),), (), None, ())
    WorkingMemory._op_removeComponentFilter = IcePy.Operation('removeComponentFilter', Ice.OperationMode.Normal, Ice.OperationMode.Normal, False, (), (((), _M_cast.cdl._t_WorkingMemoryChangeFilter),), (), None, ())
    WorkingMemory._op_registerWorkingMemoryFilter = IcePy.Operation('registerWorkingMemoryFilter', Ice.OperationMode.Normal, Ice.OperationMode.Normal, False, (), (((), _M_cast.cdl._t_WorkingMemoryChangeFilter), ((), IcePy._t_string)), (), None, ())
    WorkingMemory._op_removeWorkingMemoryFilter = IcePy.Operation('removeWorkingMemoryFilter', Ice.OperationMode.Normal, Ice.OperationMode.Normal, False, (), (((), _M_cast.cdl._t_WorkingMemoryChangeFilter),), (), None, ())
    WorkingMemory._op_addReader = IcePy.Operation('addReader', Ice.OperationMode.Normal, Ice.OperationMode.Normal, False, (), (((), _M_cast.interfaces._t_WorkingMemoryReaderComponentPrx),), (), None, ())
    WorkingMemory._op_receiveChangeEvent = IcePy.Operation('receiveChangeEvent', Ice.OperationMode.Normal, Ice.OperationMode.Normal, False, (), (((), _M_cast.cdl._t_WorkingMemoryChange),), (), None, ())

    _M_cast.interfaces.WorkingMemory = WorkingMemory
    del WorkingMemory

    _M_cast.interfaces.WorkingMemoryPrx = WorkingMemoryPrx
    del WorkingMemoryPrx

if not _M_cast.interfaces.__dict__.has_key('ComponentManager'):
    _M_cast.interfaces.ComponentManager = Ice.createTempClass()
    class ComponentManager(Ice.Object):
        def __init__(self):
            if __builtin__.type(self) == _M_cast.interfaces.ComponentManager:
                raise RuntimeError('cast.interfaces.ComponentManager is an abstract class')

        def ice_ids(self, current=None):
            return ('::Ice::Object', '::cast::interfaces::ComponentManager')

        def ice_id(self, current=None):
            return '::cast::interfaces::ComponentManager'

        def ice_staticId():
            return '::cast::interfaces::ComponentManager'
        ice_staticId = staticmethod(ice_staticId)

        #
        # Operation signatures.
        #
        # def addComponentDescription(self, description, current=None):
        # def getComponentDescription(self, componentID, current=None):

        def __str__(self):
            return IcePy.stringify(self, _M_cast.interfaces._t_ComponentManager)

        __repr__ = __str__

    _M_cast.interfaces.ComponentManagerPrx = Ice.createTempClass()
    class ComponentManagerPrx(Ice.ObjectPrx):

        def addComponentDescription(self, description, _ctx=None):
            return _M_cast.interfaces.ComponentManager._op_addComponentDescription.invoke(self, ((description, ), _ctx))

        def getComponentDescription(self, componentID, _ctx=None):
            return _M_cast.interfaces.ComponentManager._op_getComponentDescription.invoke(self, ((componentID, ), _ctx))

        def checkedCast(proxy, facetOrCtx=None, _ctx=None):
            return _M_cast.interfaces.ComponentManagerPrx.ice_checkedCast(proxy, '::cast::interfaces::ComponentManager', facetOrCtx, _ctx)
        checkedCast = staticmethod(checkedCast)

        def uncheckedCast(proxy, facet=None):
            return _M_cast.interfaces.ComponentManagerPrx.ice_uncheckedCast(proxy, facet)
        uncheckedCast = staticmethod(uncheckedCast)

    _M_cast.interfaces._t_ComponentManagerPrx = IcePy.defineProxy('::cast::interfaces::ComponentManager', ComponentManagerPrx)

    _M_cast.interfaces._t_ComponentManager = IcePy.defineClass('::cast::interfaces::ComponentManager', ComponentManager, (), True, None, (), ())
    ComponentManager.ice_type = _M_cast.interfaces._t_ComponentManager

    ComponentManager._op_addComponentDescription = IcePy.Operation('addComponentDescription', Ice.OperationMode.Idempotent, Ice.OperationMode.Idempotent, False, (), (((), _M_cast.cdl._t_ComponentDescription),), (), None, ())
    ComponentManager._op_getComponentDescription = IcePy.Operation('getComponentDescription', Ice.OperationMode.Idempotent, Ice.OperationMode.Idempotent, False, (), (((), IcePy._t_string),), (), _M_cast.cdl._t_ComponentDescription, ())

    _M_cast.interfaces.ComponentManager = ComponentManager
    del ComponentManager

    _M_cast.interfaces.ComponentManagerPrx = ComponentManagerPrx
    del ComponentManagerPrx

if not _M_cast.interfaces.__dict__.has_key('ComponentFactory'):
    _M_cast.interfaces.ComponentFactory = Ice.createTempClass()
    class ComponentFactory(Ice.Object):
        def __init__(self):
            if __builtin__.type(self) == _M_cast.interfaces.ComponentFactory:
                raise RuntimeError('cast.interfaces.ComponentFactory is an abstract class')

        def ice_ids(self, current=None):
            return ('::Ice::Object', '::cast::interfaces::ComponentFactory')

        def ice_id(self, current=None):
            return '::cast::interfaces::ComponentFactory'

        def ice_staticId():
            return '::cast::interfaces::ComponentFactory'
        ice_staticId = staticmethod(ice_staticId)

        #
        # Operation signatures.
        #
        # def newComponent(self, id, type, newProcess, current=None):
        # def newManagedComponent(self, id, type, newProcess, current=None):
        # def newUnmanagedComponent(self, id, type, newProcess, current=None):
        # def newWorkingMemory(self, id, type, newProcess, current=None):
        # def newTaskManager(self, id, type, newProcess, current=None):

        def __str__(self):
            return IcePy.stringify(self, _M_cast.interfaces._t_ComponentFactory)

        __repr__ = __str__

    _M_cast.interfaces.ComponentFactoryPrx = Ice.createTempClass()
    class ComponentFactoryPrx(Ice.ObjectPrx):

        def newComponent(self, id, type, newProcess, _ctx=None):
            return _M_cast.interfaces.ComponentFactory._op_newComponent.invoke(self, ((id, type, newProcess), _ctx))

        def newManagedComponent(self, id, type, newProcess, _ctx=None):
            return _M_cast.interfaces.ComponentFactory._op_newManagedComponent.invoke(self, ((id, type, newProcess), _ctx))

        def newUnmanagedComponent(self, id, type, newProcess, _ctx=None):
            return _M_cast.interfaces.ComponentFactory._op_newUnmanagedComponent.invoke(self, ((id, type, newProcess), _ctx))

        def newWorkingMemory(self, id, type, newProcess, _ctx=None):
            return _M_cast.interfaces.ComponentFactory._op_newWorkingMemory.invoke(self, ((id, type, newProcess), _ctx))

        def newTaskManager(self, id, type, newProcess, _ctx=None):
            return _M_cast.interfaces.ComponentFactory._op_newTaskManager.invoke(self, ((id, type, newProcess), _ctx))

        def checkedCast(proxy, facetOrCtx=None, _ctx=None):
            return _M_cast.interfaces.ComponentFactoryPrx.ice_checkedCast(proxy, '::cast::interfaces::ComponentFactory', facetOrCtx, _ctx)
        checkedCast = staticmethod(checkedCast)

        def uncheckedCast(proxy, facet=None):
            return _M_cast.interfaces.ComponentFactoryPrx.ice_uncheckedCast(proxy, facet)
        uncheckedCast = staticmethod(uncheckedCast)

    _M_cast.interfaces._t_ComponentFactoryPrx = IcePy.defineProxy('::cast::interfaces::ComponentFactory', ComponentFactoryPrx)

    _M_cast.interfaces._t_ComponentFactory = IcePy.defineClass('::cast::interfaces::ComponentFactory', ComponentFactory, (), True, None, (), ())
    ComponentFactory.ice_type = _M_cast.interfaces._t_ComponentFactory

    ComponentFactory._op_newComponent = IcePy.Operation('newComponent', Ice.OperationMode.Normal, Ice.OperationMode.Normal, False, (), (((), IcePy._t_string), ((), IcePy._t_string), ((), IcePy._t_bool)), (), _M_cast.interfaces._t_CASTComponentPrx, (_M_cast._t_ComponentCreationException,))
    ComponentFactory._op_newManagedComponent = IcePy.Operation('newManagedComponent', Ice.OperationMode.Normal, Ice.OperationMode.Normal, False, (), (((), IcePy._t_string), ((), IcePy._t_string), ((), IcePy._t_bool)), (), _M_cast.interfaces._t_ManagedComponentPrx, (_M_cast._t_ComponentCreationException,))
    ComponentFactory._op_newUnmanagedComponent = IcePy.Operation('newUnmanagedComponent', Ice.OperationMode.Normal, Ice.OperationMode.Normal, False, (), (((), IcePy._t_string), ((), IcePy._t_string), ((), IcePy._t_bool)), (), _M_cast.interfaces._t_UnmanagedComponentPrx, (_M_cast._t_ComponentCreationException,))
    ComponentFactory._op_newWorkingMemory = IcePy.Operation('newWorkingMemory', Ice.OperationMode.Normal, Ice.OperationMode.Normal, False, (), (((), IcePy._t_string), ((), IcePy._t_string), ((), IcePy._t_bool)), (), _M_cast.interfaces._t_WorkingMemoryPrx, (_M_cast._t_ComponentCreationException,))
    ComponentFactory._op_newTaskManager = IcePy.Operation('newTaskManager', Ice.OperationMode.Normal, Ice.OperationMode.Normal, False, (), (((), IcePy._t_string), ((), IcePy._t_string), ((), IcePy._t_bool)), (), _M_cast.interfaces._t_TaskManagerPrx, (_M_cast._t_ComponentCreationException,))

    _M_cast.interfaces.ComponentFactory = ComponentFactory
    del ComponentFactory

    _M_cast.interfaces.ComponentFactoryPrx = ComponentFactoryPrx
    del ComponentFactoryPrx

# End of module cast.interfaces

__name__ = 'cast'

# Start of module cast.examples
_M_cast.examples = Ice.openModule('cast.examples')
__name__ = 'cast.examples'

# Start of module cast.examples.autogen
_M_cast.examples.autogen = Ice.openModule('cast.examples.autogen')
__name__ = 'cast.examples.autogen'

if not _M_cast.examples.autogen.__dict__.has_key('WordServer'):
    _M_cast.examples.autogen.WordServer = Ice.createTempClass()
    class WordServer(Ice.Object):
        def __init__(self):
            if __builtin__.type(self) == _M_cast.examples.autogen.WordServer:
                raise RuntimeError('cast.examples.autogen.WordServer is an abstract class')

        def ice_ids(self, current=None):
            return ('::Ice::Object', '::cast::examples::autogen::WordServer')

        def ice_id(self, current=None):
            return '::cast::examples::autogen::WordServer'

        def ice_staticId():
            return '::cast::examples::autogen::WordServer'
        ice_staticId = staticmethod(ice_staticId)

        #
        # Operation signatures.
        #
        # def getNewWord(self, current=None):

        def __str__(self):
            return IcePy.stringify(self, _M_cast.examples.autogen._t_WordServer)

        __repr__ = __str__

    _M_cast.examples.autogen.WordServerPrx = Ice.createTempClass()
    class WordServerPrx(Ice.ObjectPrx):

        def getNewWord(self, _ctx=None):
            return _M_cast.examples.autogen.WordServer._op_getNewWord.invoke(self, ((), _ctx))

        def checkedCast(proxy, facetOrCtx=None, _ctx=None):
            return _M_cast.examples.autogen.WordServerPrx.ice_checkedCast(proxy, '::cast::examples::autogen::WordServer', facetOrCtx, _ctx)
        checkedCast = staticmethod(checkedCast)

        def uncheckedCast(proxy, facet=None):
            return _M_cast.examples.autogen.WordServerPrx.ice_uncheckedCast(proxy, facet)
        uncheckedCast = staticmethod(uncheckedCast)

    _M_cast.examples.autogen._t_WordServerPrx = IcePy.defineProxy('::cast::examples::autogen::WordServer', WordServerPrx)

    _M_cast.examples.autogen._t_WordServer = IcePy.defineClass('::cast::examples::autogen::WordServer', WordServer, (), True, None, (), ())
    WordServer.ice_type = _M_cast.examples.autogen._t_WordServer

    WordServer._op_getNewWord = IcePy.Operation('getNewWord', Ice.OperationMode.Normal, Ice.OperationMode.Normal, False, (), (), (), IcePy._t_string, ())

    _M_cast.examples.autogen.WordServer = WordServer
    del WordServer

    _M_cast.examples.autogen.WordServerPrx = WordServerPrx
    del WordServerPrx

if not _M_cast.examples.autogen.__dict__.has_key('WordServerAsComponent'):
    _M_cast.examples.autogen.WordServerAsComponent = Ice.createTempClass()
    class WordServerAsComponent(_M_cast.interfaces.CASTComponent):
        def __init__(self):
            if __builtin__.type(self) == _M_cast.examples.autogen.WordServerAsComponent:
                raise RuntimeError('cast.examples.autogen.WordServerAsComponent is an abstract class')

        def ice_ids(self, current=None):
            return ('::Ice::Object', '::cast::examples::autogen::WordServerAsComponent', '::cast::interfaces::CASTComponent')

        def ice_id(self, current=None):
            return '::cast::examples::autogen::WordServerAsComponent'

        def ice_staticId():
            return '::cast::examples::autogen::WordServerAsComponent'
        ice_staticId = staticmethod(ice_staticId)

        #
        # Operation signatures.
        #
        # def getNewWord(self, current=None):

        def __str__(self):
            return IcePy.stringify(self, _M_cast.examples.autogen._t_WordServerAsComponent)

        __repr__ = __str__

    _M_cast.examples.autogen.WordServerAsComponentPrx = Ice.createTempClass()
    class WordServerAsComponentPrx(_M_cast.interfaces.CASTComponentPrx):

        def getNewWord(self, _ctx=None):
            return _M_cast.examples.autogen.WordServerAsComponent._op_getNewWord.invoke(self, ((), _ctx))

        def checkedCast(proxy, facetOrCtx=None, _ctx=None):
            return _M_cast.examples.autogen.WordServerAsComponentPrx.ice_checkedCast(proxy, '::cast::examples::autogen::WordServerAsComponent', facetOrCtx, _ctx)
        checkedCast = staticmethod(checkedCast)

        def uncheckedCast(proxy, facet=None):
            return _M_cast.examples.autogen.WordServerAsComponentPrx.ice_uncheckedCast(proxy, facet)
        uncheckedCast = staticmethod(uncheckedCast)

    _M_cast.examples.autogen._t_WordServerAsComponentPrx = IcePy.defineProxy('::cast::examples::autogen::WordServerAsComponent', WordServerAsComponentPrx)

    _M_cast.examples.autogen._t_WordServerAsComponent = IcePy.defineClass('::cast::examples::autogen::WordServerAsComponent', WordServerAsComponent, (), True, None, (_M_cast.interfaces._t_CASTComponent,), ())
    WordServerAsComponent.ice_type = _M_cast.examples.autogen._t_WordServerAsComponent

    WordServerAsComponent._op_getNewWord = IcePy.Operation('getNewWord', Ice.OperationMode.Normal, Ice.OperationMode.Normal, False, (), (), (), IcePy._t_string, ())

    _M_cast.examples.autogen.WordServerAsComponent = WordServerAsComponent
    del WordServerAsComponent

    _M_cast.examples.autogen.WordServerAsComponentPrx = WordServerAsComponentPrx
    del WordServerAsComponentPrx

# End of module cast.examples.autogen

__name__ = 'cast.examples'

# End of module cast.examples

__name__ = 'cast'

# End of module cast

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
