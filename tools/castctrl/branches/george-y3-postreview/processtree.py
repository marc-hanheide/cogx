#!/usr/bin/env python
# vim:set fileencoding=utf-8 sw=4 ts=8 et:vim
# Author:  Marko Mahnič
# Created: jun 2009 

import os, sys
from PyQt4 import QtCore, QtGui
from core import procman

class CTreeItem:
    def __init__(self, parent):
        self.parentItem = parent

    def parent(self):
        return self.parentItem

    def getChildList(self):
        return []

    def row(self):
        try:
            if self.parentItem:
                return self.parentItem.getChildList().index(self)
        except ValueError:
            print "CTreeItem::row FAILED in self.parentItem.getChildList().index(self)"
            return 0
        return 0

    def color(self):
        return ""

    def propagateChange(self, child):
        if self.parentItem: self.parentItem.propagateChange(self)

class CProcessItem(CTreeItem, procman.CProcessObserver):
    def __init__(self, process, parent):
        CTreeItem.__init__(self, parent)
        self.process = process
        self.process.observers.append(self)

    def childCount(self):
        return 0

    def child(self, row):
        return None

    def columnCount(self):
        return 2

    def data(self, column):
        if column == 0: return self.process.name
        if column == 1: return self.process.getStatusStr()
        return ""

    def color(self):
        st = self.process.getStatusLevel()
        if st >= 2: return "red"
        if st > 0: return "blue"
        return ""

    def notifyStatusChange(self, process, oldStatus, newStatus):
        self.propagateChange(self)
        pass

class CHostItem(CTreeItem):
    def __init__(self, processManager, parent=None):
        CTreeItem.__init__(self, parent)
        self.host = processManager
        self.children = []
        self.prepareChildern()

    def prepareChildern(self):
        self.children = [CProcessItem(p, self) for p in self.host.proclist]

    def childCount(self):
        return len(self.children)

    def child(self, row):
        return self.children[row]

    def getChildList(self):
        return self.children

    def columnCount(self):
        return 2

    def data(self, column):
        if column == 0: return self.host.name
        elif column == 1: return self.host.getStatusStr()
        return ""

    def color(self):
        st = self.host.getStatusLevel()
        if st == 0:
            if len(self.children) < 1: return ""
            st = max( child.process.getStatusLevel() for child in self.children )
        if st >= 2: return "red"
        if st > 0: return "blue"
        return ""

class CRootItem(CTreeItem):
    def __init__(self, model):
        CTreeItem.__init__(self, None)
        self.hosts = []
        self.model = model

    def addHost(self, processManager):
        self.hosts.append(CHostItem(processManager, self))

    def removeHost(self, processManager):
        for item in list(self.hosts):
            if item.host == processManager:
                self.hosts.remove(item)

    def childCount(self):
        return len(self.hosts)

    def child(self, row):
        return self.hosts[row]

    def getChildList(self):
        return self.hosts

    def columnCount(self):
        return 2

    def data(self, column):
        if column == 0: return "Process"
        if column == 1: return "Status"
        return ""

    def propagateChange(self, child):
        if self.model != None:
            # a somewhat dirty solution
            self.model.emit(QtCore.SIGNAL("layoutChanged()"))

class CProcessTreeModel(QtCore.QAbstractItemModel):
    def __init__(self, parent=None):
        QtCore.QAbstractItemModel.__init__(self, parent)
        self.rootItem = CRootItem(self)

    def addHost(self, processManager):
        self.rootItem.addHost(processManager)
        self.reset()
            
    def index(self, row, column, parent=QtCore.QModelIndex()):
        if row < 0 or column < 0 or row >= self.rowCount(parent): # or column >= self.columnCount(parent):
            return QtCore.QModelIndex()

        if not parent.isValid():
            parentItem = self.rootItem
        else:
            parentItem = parent.internalPointer()

        childItem = parentItem.child(row)
        if childItem:
            return self.createIndex(row, column, childItem)
        else:
            return QtCore.QModelIndex()
        pass

    def parent(self, index):
        if not index.isValid():
            return QtCore.QModelIndex()

        childItem = index.internalPointer()
        parentItem = childItem.parent()

        if parentItem == self.rootItem:
            return QtCore.QModelIndex()

        return self.createIndex(parentItem.row(), 0, parentItem)

    def rowCount(self, parent=QtCore.QModelIndex()):
        if not parent.isValid():
            parentItem = self.rootItem
        else:
            parentItem = parent.internalPointer()

        return parentItem.childCount()

    def columnCount(self, parent=QtCore.QModelIndex()):
        if parent.isValid():
            return parent.internalPointer().columnCount()
        else:
            return self.rootItem.columnCount()

    def data(self, index, role=QtCore.Qt.DisplayRole):
        if not index.isValid(): return QtCore.QVariant()
        if role == QtCore.Qt.DisplayRole:
            item = index.internalPointer()
            return QtCore.QVariant(item.data(index.column()))
        if role == QtCore.Qt.TextColorRole:
            item = index.internalPointer()
            return QtCore.QVariant(QtGui.QColor(item.color()))
        return QtCore.QVariant()

    def headerData(self, section, orientation, role):
        if orientation == QtCore.Qt.Horizontal and role == QtCore.Qt.DisplayRole:
            return QtCore.QVariant(self.rootItem.data(section))

        return QtCore.QVariant()

    #def hasChildren(self):
    #    pass

