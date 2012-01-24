#!/usr/bin/python
# vim:set fileencoding=utf-8 sw=4 ts=8 et:vim #
# Author: Marko Mahnič
# Created: March 2011

import os, sys
from PyQt4 import QtCore, QtGui
from PyQt4.QtCore import Qt
from editors import ICustomEditorBase, CTextEditor, CFilenameEditor, CStringItemEditor
import properties

class QLineEditorCreator(QtGui.QItemEditorCreatorBase):
    def __init__(self): super(QLineEditorCreator, self).__init__()
    def createWidget(self, parent): return QtGui.QLineEdit(parent)

# The editor factory will create fields for some of the standard variant field types.
# Extedned types like FilenameEditor will be created by the CItemDelegate in
# createEditor().
class EditorFactory(QtGui.QItemEditorFactory):
    def __init__(self):
        super(EditorFactory, self).__init__()
        lec = QLineEditorCreator()
        self.registerEditor(QtCore.QVariant.Double, lec)
        self.registerEditor(QtCore.QVariant.Int, lec)
        self.registerEditor(QtCore.QVariant.String, lec)

class CItemDelegate(QtGui.QStyledItemDelegate):
    editorCreated = QtCore.pyqtSignal(QtGui.QWidget)

    def __init__(self, parent=None):
        QtGui.QStyledItemDelegate.__init__(self, parent)

    def sizeHint(self, option, index):
        if not index.isValid():
            return QtCore.QSize()
        obj = index.internalPointer()
        if isinstance(obj, CPropertyNode):
            editor_height = 22 # XXX: depends on font and decorations
            sz = super(CItemDelegate, self).sizeHint(option, index)
            if sz.height() < editor_height:
                sz.setHeight(editor_height)
            return sz

        return super(CItemDelegate, self).sizeHint(option, index)

    def getType(self, data):
        if data.type() == QtCore.QVariant.List:
            ldata = data.toList()
            if len(ldata) > 0: return ldata[0].toString()
        return data.type()

    def createEditor(self, parent, styleOption, modelIndex):
        if not modelIndex.isValid(): return None

        data = modelIndex.data(role=Qt.EditRole)
        wtype = self.getType(data)
        rv = None
        if wtype == "text": rv = CTextEditor(parent)
        elif wtype == "filename": rv = CFilenameEditor(parent)
        elif wtype == "stringitem": rv = CStringItemEditor(parent)
        else: rv = super(CItemDelegate, self).createEditor(parent, styleOption, modelIndex)

        if rv != None:
            self.editorCreated.emit(rv)

        return rv

    def setEditorData(self, editWidget, modelIndex):
        if not modelIndex.isValid(): return

        if isinstance(editWidget, ICustomEditorBase):
            data = modelIndex.data(role=Qt.EditRole)
            editWidget.setEditData(data)
            return

        super(CItemDelegate, self).setEditorData(editWidget, modelIndex)

    def setModelData(self, editWidget, model, modelIndex):
        if not modelIndex.isValid(): return

        if isinstance(editWidget, ICustomEditorBase):
            model.setData(modelIndex, editWidget.editData())
            return

        super(CItemDelegate, self).setModelData(editWidget, model, modelIndex)

class CTreeNode:
    def __init__(self, parent):
        self.parentNode = parent

    def parent(self):
        return self.parentNode

    def getChildList(self):
        return []

    def childCount(self):
        return len(self.getChildList())

    def child(self, row):
        return self.getChildList()[row]

    def row(self):
        if self.parentNode:
            return self.parentNode.getChildList().index(self)
        return 0

    def columnCount(self):
        return 1

    def span(self):
        return QtCore.QSize(1, 1)

    # param value is ['type', 'value', ['value-history']]
    # type may be empty; history is optional
    def setData(self, value, role=Qt.EditRole):
        if role == Qt.EditRole:
            lval = value.toList()
            print "Setting value:", lval[1].toString()

    # returns ['type', propertyObj, extra... ]
    # history is optional
    def data(self, column, role=Qt.DisplayRole):
        if column >= self.columnCount(): return None
        val = "%d: %s" % (column, ("%s" % self.__class__).split('.')[-1])
        if role == Qt.EditRole:
            data = QtCore.QVariant(["text", val])
            return data

        return val

# Properties are stored in a tree view. They are grouped by application (server).
#   - level 0: CRootNode, root item (not visible)
#   - level 1: CApplicationNode, application items
#   - level 2: CPropertyNode, application settings, name-value pairs with optional extra handlers
#
#   A CPropertyNode has a member that points to the real property.
#   Maybe it would be better (easier) to have a separate class for each type of
#   property.
class CPropertyNode(CTreeNode):
    def __init__(self, parent):
        CTreeNode.__init__(self, parent)
        self.ctrltype = "text"
        self.formatter = None # A function that will format items for display
        self.property = None  # The real property that contains the data

    def columnCount(self):
        return 2

    def setData(self, value, role=Qt.EditRole):
        if role == Qt.EditRole and self.property != None:
            lval = value.toList()

            # XXX: To apply the value only on OK, self.value should be set, instead
            self.property.value = "%s" % lval[1].toString()

            # TODO: add to MRU history, if applicable

    def data(self, column, role=Qt.DisplayRole):
        if column >= self.columnCount(): return None
        if role == Qt.CheckStateRole: return None
        if self.property == None: return "NA"

        if column == 0:
            return QtCore.QVariant(self.property.label)

        if role == Qt.EditRole:
            # TODO???: instead of ctrltype + property + formatter send self.
            data = QtCore.QVariant([self.ctrltype, self.property, self.formatter])
            return data

        if role == Qt.DisplayRole or role == Qt.ToolTipRole:
            if self.formatter != None:
                return self.formatter(self.property.value)
            return self.property.value

        return None


class CProcessNode(CTreeNode):
    def __init__(self, parent, server=None):
        CTreeNode.__init__(self, parent)
        self.server = server
        self.propertyList = []

    def getChildList(self):
        return self.propertyList

    def columnCount(self):
        return 1

    def span(self):
        return QtCore.QSize(2, 1)

    def data(self, column, role=Qt.DisplayRole):
        if role == Qt.DisplayRole:
            if column >= self.columnCount(): return None
            if self.server == None: return "NA"

            if column == 0:
                return self.server.label

            return ""

        if role == Qt.BackgroundRole:
            pal = QtGui.QApplication.palette()
            brush = pal.brush(QtGui.QPalette.Button)
            return brush

        if role == Qt.CheckStateRole:
            if self.server == None: return None
            return Qt.Checked if self.server.enabled else Qt.Unchecked

        return None

    def setData(self, value, role=Qt.EditRole):
        if role == Qt.CheckStateRole and self.server != None:
            self.server.enabled = value.toBool()

class CRootNode(CTreeNode):
    def __init__(self, model):
        CTreeNode.__init__(self, None)
        self.model = model
        self.processList = []

    def childCount(self):
        return len(self.processList)

    def child(self, row):
        return self.processList[row]

    def getChildList(self):
        return self.processList

    def columnCount(self):
        return 2 # max from all children

    def headerData(self, column):
        if column == 0: return "Property"
        if column == 1: return "Value"
        if column == 2: return "Extra"
        return ""


class CPropertyTreeModel(QtCore.QAbstractItemModel):
    def __init__(self, parent=None):
        QtCore.QAbstractItemModel.__init__(self, parent)
        self.rootNode = CRootNode(self)

        def mkTextNode(parent=None):
            p = CPropertyNode(parent)
            p.ctrltype = "text"
            return p

        def mkStringItemNode(parent=None):
            p = CPropertyNode(parent)
            p.ctrltype = "stringitem"
            return p

        def mkFilenameNode(parent=None):
            def pathDisplay(path):
                # TODO: make path relative to castcontrol root
                if path == None: return ""
                relpath = os.path.dirname(path)
                return "%s @ %s" % (os.path.basename(path), relpath)
            p = CPropertyNode(parent)
            p.ctrltype = "filename"
            p.formatter = pathDisplay
            return p

        self.propertyTypeMap = [
                (properties.CStringProperty, mkTextNode),
                (properties.CIntProperty, mkTextNode),
                (properties.CFloatProperty, mkTextNode),
                (properties.CStringItemProperty, mkStringItemNode),
                (properties.CFilenameProperty, mkFilenameNode),
               ]

    def flags(self, index):
        if index.isValid():
            obj = index.internalPointer()
            flags = super(CPropertyTreeModel, self).flags(index)
            flags = flags & ~Qt.ItemIsUserCheckable
            if isinstance(obj, CPropertyNode) and index.column() == 1:
                flags = flags | Qt.ItemIsEditable
            elif isinstance(obj, CProcessNode) and index.column() == 0:
                flags = flags | Qt.ItemIsUserCheckable
        return flags

    def index(self, row, column, parent=QtCore.QModelIndex()):
        if row < 0 or column < 0 or row >= self.rowCount(parent): # or column >= self.columnCount(parent):
            return QtCore.QModelIndex()

        if not parent.isValid():
            parentNode = self.rootNode
        else:
            parentNode = parent.internalPointer()

        childNode = parentNode.child(row)
        if childNode:
            return self.createIndex(row, column, childNode)
        else:
            return QtCore.QModelIndex()
        pass

    def parent(self, index):
        if not index.isValid():
            return QtCore.QModelIndex()

        childNode = index.internalPointer()
        parentNode = childNode.parent()

        if parentNode == self.rootNode:
            return QtCore.QModelIndex()

        return self.createIndex(parentNode.row(), 0, parentNode)

    def rowCount(self, parent=QtCore.QModelIndex()):
        if not parent.isValid():
            parentNode = self.rootNode
        else:
            parentNode = parent.internalPointer()

        return parentNode.childCount()

    def columnCount(self, parent=QtCore.QModelIndex()):
        if parent.isValid():
            return parent.internalPointer().columnCount()
        else:
            return self.rootNode.columnCount()

    ## Qt4.6; Qt4.7 Note: Currently, span is not used. Use setFirstColumnSpanned instead.
    #def span(self, index):
    #    if index.isValid():
    #        return index.internalPointer().span()
    #    else:
    #        return self.rootNode.span()

    def setData(self, modelIndex, value, role=Qt.EditRole):
        if role == Qt.EditRole:
            print "model.setData", role, value.toList()
            try:
                item = modelIndex.internalPointer()
                item.setData(value, role)
                return True
            except Exception as ex:
                print ex
                return False

        if role == Qt.CheckStateRole:
            try:
                item = modelIndex.internalPointer()
                item.setData(value, role)
                return True
            except Exception as ex:
                print ex
                return False

        return super(CPropertyTreeModel, self).setData(modelIndex, value, role)


    def data(self, index, role=Qt.DisplayRole):
        if not index.isValid(): return QtCore.QVariant()
        if role == Qt.DisplayRole or role == Qt.EditRole or role == Qt.ToolTipRole or role == Qt.CheckStateRole:
            item = index.internalPointer()
            return QtCore.QVariant(item.data(index.column(), role))

        if role == Qt.BackgroundRole:
            item = index.internalPointer()
            return QtCore.QVariant(item.data(index.column(), role))

        #if role == Qt.TextColorRole:
        #    item = index.internalPointer()
        #    return QtCore.QVariant(QtGui.QColor(item.color()))
        return QtCore.QVariant()


    def headerData(self, section, orientation, role):
        if orientation == Qt.Horizontal and role == Qt.DisplayRole:
            return QtCore.QVariant(self.rootNode.headerData(section))

        return QtCore.QVariant()


    def prepareServerRows(self, treeView):
        p = QtCore.QModelIndex()
        for i in xrange(len(self.rootNode.processList)):
            treeView.setFirstColumnSpanned(i, p, True)

            # Doesnt work. http://lists.trolltech.com/qt-interest/2005-08/msg00516.html
            # self.setData(self.createIndex(i, 0, p), Qt.Checked, Qt.CheckStateRole)

        pass

    # The servers are the nodes directly below the RootNode
    def addServers(self, serverList):
        for srv in serverList:
            prg = CProcessNode(self.rootNode, srv)
            for prop in srv.properties:
                found = False
                # Ugly type mapping, but this way the Model (CProperty) doesn't need
                # to know anything about the View (CPropertyNode)
                for tm in self.propertyTypeMap:
                    if isinstance(prop, tm[0]):
                        pn = tm[1](prg)
                        pn.property = prop
                        prg.propertyList.append(pn)
                        found = True
                        break
                if not found:
                    print "Property of type %s is not supported." % prop.__class__
                    pass # XXX: what now?
            self.rootNode.processList.append(prg)


