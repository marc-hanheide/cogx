#!/usr/bin/env python
# vim: set fileencoding=utf-8 ft=python ts=8 sw=4 et :vim #
# Author: Marko Mahnič
# Created: March 2011

class CProperty(object):
    def __init__(self, name, label=None, default=None):
        if label == None: label = name
        self.name = name
        self.label = label
        self._value = default
        self.validation = None
        self.mruEnabled = False
        self.mruHistory = None
        #self.group = None

    def _updateMru(self):
        if not self.mruEnabled: return
        if self.mruHistory == None:
            self.mruHistory = []
        try:
            self.mruHistory.remove(self.value)
        except: pass
        self.mruHistory.insert(0, self.value)

    # A property with overridable getter/setter
    def __fget_value(self): return self._get_value()
    def __fset_value(self, value):
        self._set_value(value)
        if self.mruEnabled:
            self._updateMru()

    value = property(__fget_value, __fset_value)

    # property implementation; getter returns a string by default
    def _get_value(self): return self._value
    def _set_value(self, value): self._value = value

    # XXX Maybe we should support 2 types of values: value as entered by the user (a string) and value as required by the program (a Python object)

class CIntProperty(CProperty):
    def __init__(self, name, label=None, default=0, range = None):
        super(CIntProperty, self).__init__(name, label, default)
        self.range = range
        self.validation = "int"

    # XXX ATM Don't support empty values
    def _set_value(self, value):
        if value == None or value == "":
            self._value = 0
        else: self._value = int(value)

class CFloatProperty(CProperty):
    def __init__(self, name, label=None, default=0, range = None):
        super(CFloatProperty, self).__init__(name, label, default)
        self.range = range
        self.validation = "double"

    # XXX ATM Don't support empty values
    def _set_value(self, value):
        if value == None or value == "":
            self._value = 0
        else: self._value = float(value)

class CStringProperty(CProperty):
    def __init__(self, name, label=None, default=""):
        super(CStringProperty, self).__init__(name, label, default)

class CStringItemProperty(CProperty):
    def __init__(self, name, label=None, default="", items=[]):
        super(CStringItemProperty, self).__init__(name, label, default)
        self.items = items
        if not self.value in self.items:
            self.value = ""

class CFilenameProperty(CProperty):
    # @param filter: tr("Name (mask mask ...);;Name (mask ...) ...")
    def __init__(self, name, label=None, default="", filter="All files (*)", allowEmpty=False):
        super(CFilenameProperty, self).__init__(name, label, default)
        self.filter = filter
        self.mruEnabled = True
        self.allowEmpty = allowEmpty

    #def _get_value(self):
    #    if self._value == None or self._value == "<none>" or self._value == "":
    #        return "<none>"

    #def _set_value(self, value):
    #    if value == None or value == "<none>" or value == "":
    #        value = "<none>"
    #    self._value = value

class CPropertySet(object):
    def __init__(self, name, **kwargs):
        self.name = name
        self.label = name
        self.properties = []
        if 'label' in kwargs: self.label = kwargs['label']

    def getProperty(self, name):
        for p in self.properties:
            if name == p.name: return p
        return None

    def integerField(self, name, label=None, range=None, default=None, group=None):
        p = CIntProperty(name, label, default, range)
        self.properties.append(p)

    def floatField(self, name, label=None, range=None, default=None, group=None):
        p = CFloatProperty(name, label, default, range)
        self.properties.append(p)

    def stringField(self, name, label=None, default=None, group=None):
        p = CStringProperty(name, label, default)
        self.properties.append(p)

    def stringItemField(self, name, label=None, items=None, default=None, group=None):
        if default == None and items != None:
            default = items[0]
        p = CStringItemProperty(name, label, default, items)
        self.properties.append(p)

    def filenameField(self, name, label=None, filter=None, default=None, group=None, allowEmpty=False):
        p = CFilenameProperty(name, label, default, filter, allowEmpty)
        self.properties.append(p)
