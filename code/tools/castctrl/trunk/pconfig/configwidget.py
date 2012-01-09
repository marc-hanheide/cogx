#!/usr/bin/python
# vim:set fileencoding=utf-8 sw=4 ts=8 et:vim #
# Author: Marko Mahnič
# Created: March 2011

import os, sys
from PyQt4 import QtCore, QtGui
from PyQt4.QtCore import Qt
from editors import ICustomEditorBase, CTextEditor, CFilenameEditor, CStringItemEditor
from propeditor import *

class CConfigWidget(QtGui.QWidget):
    def __init__(self, parent):
        QtGui.QWidget.__init__(self, parent)
        self.createUi()
        self.installEditors()
        self.editBuddy = None # (HACK) eg. Apply button that is enabled when an editor is active

    def installEditors(self):
        self.itemDelegate = CItemDelegate(self)
        self.editFactory = EditorFactory()
        self.itemDelegate.setItemEditorFactory(self.editFactory)
        self.connect(self.itemDelegate,
                QtCore.SIGNAL("closeEditor(QWidget*,QAbstractItemDelegate::EndEditHint)"),
                self.onEditorClosed);
        self.connect(self.itemDelegate,
                QtCore.SIGNAL("editorCreated()"),
                self.onEditorCreated);
        self.wItems.setItemDelegate(self.itemDelegate)
        qet = QtGui.QAbstractItemView
        # XXX: Activation variant could be user configurable
        # self.wItems.setEditTriggers(qet.DoubleClicked | qet.SelectedClicked | qet.EditKeyPressed)
        self.wItems.setEditTriggers(qet.SelectedClicked | qet.EditKeyPressed | qet.CurrentChanged)

    def createUi(self):
        self.mProperties = CPropertyTreeModel()
        #self.demoServers_load()
        # self.demoServers_make()
        # self.demoData()

        self.wItems = QtGui.QTreeView(parent=self)
        self.wItems.setAllColumnsShowFocus(True)
        self.wItems.setRootIsDecorated(True)
        self.wItems.setSelectionBehavior(QtGui.QAbstractItemView.SelectItems)
        #self.wItems.setIndentation(12)
        #self.wItems.setStyleSheet("""
        #QTreeView::item {
        #    border: 1px solid #d9d9d9; border-top-color: transparent; border-left-color:transparent;
        #}
        #QTreeView::item:hover {
        #    background: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 #e7effd, stop: 1 #cbdaf1);
        #    border: 1px solid #bfcde4;
        #}
        #""");
        self.wItems.setModel(self.mProperties)
        self.mProperties.prepareServerRows(self.wItems)

        horz = QtGui.QVBoxLayout(self)
        horz.setContentsMargins(0, 0, -1, 0)
        horz.addWidget(self.wItems)


    def updateHeader(self):
        self.wItems.expandAll()
        self.wItems.resizeColumnToContents(0)
        #maxWidth = self.wItems.width() / 3
        #if self.wItems.columnWidth(0) > maxWidth:
        #   self.wItems.setColumnWidth(0, maxWidth)
        self.wItems.collapseAll()


    def addServers(self, servers):
        self.mProperties.addServers(servers)
        self.mProperties.prepareServerRows(self.wItems)

    # (HACK)
    def setEditBuddy(self, buddy):
        self.editBuddy = buddy
        if buddy != None:
            buddy.setEnabled(False)
            buddy.setVisible(False) # TODO: it doesn't work, yet, so hide it

    # (HACK)
    def onEditorCreated(self):
        if self.editBuddy != None:
            self.editBuddy.setEnabled(True)

    # (HACK)
    def onEditorClosed(self, wEditor, endEditHint):
        if self.editBuddy != None:
            self.editBuddy.setEnabled(False)

    #def loadServers(self, fname):
    #    import manager
    #    am = manager.CAppManager()
    #    srvrs = am.discoverServers(fname)
    #    self.mProperties.addServers(srvrs)
    #    self.mProperties.prepareServerRows(self.wItems)


    #def demoServers_load(self):
    #    import manager
    #    am = manager.CAppManager()
    #    srvrs = am.discoverServers("mainservers.txt")
    #    self.mProperties.addServers(srvrs)
    #    srvrs = am.discoverServers("cogxservers.txt")
    #    self.mProperties.addServers(srvrs)

    #def demoData(self):
    #    import extserver
    #    prg = CProcessNode(self.mProperties.rootNode)
    #    prop = CPropertyNode(prg)
    #    prop.property = extserver.CStringProperty("Good")
    #    prg.propertyList.append(prop)
    #    prop = CPropertyNode(prg)
    #    prop.property = extserver.CFilenameProperty("Filename")
    #    prg.propertyList.append(prop)
    #    self.mProperties.rootNode.processList.append(prg)

    #    prg = CProcessNode(self.mProperties.rootNode)
    #    prop = CPropertyNode(prg)
    #    prop.property = extserver.CStringProperty("Good", label="Good 2")
    #    prg.propertyList.append(prop)
    #    prop = CPropertyNode(prg)
    #    prop.property = extserver.CFilenameProperty("Filename", label="Filename 2")
    #    prg.propertyList.append(prop)
    #    self.mProperties.rootNode.processList.append(prg)

    #def demoServers_make(self):
    #    build = extserver.Server("BUILD", label="Build", group="Build")
    #    build.stringField("BUILDDIR", label="Build directory", default="${COGX_ROOT}/BUILD")
    #    build.stringItemField("PROFILE", label="Profile", items=["Debug", "Release"], default="Release")
    #    build.setCommand("make [TARGET]", workdir="[BUILDDIR]")

    #    player = extserver.Server("PLAYER", label="Player")
    #    player.filenameField("CONFIG",
    #            label="Configuration", filter="Player Config (*.cfg);;All files (*)")
    #    player.integerField("PORT", label="Port", default=0, range=(1000, 40000))
    #    player.floatField("WEIGHT", label="Weight", default=0, range=(1000, 40000))
    #    player.setCommand("player [CONFIG]")

    #    pbot = extserver.Server("PEEKABOT", label="Peekabot")
    #    pbot.filenameField("CONFIG",
    #            label="Configuration", filter="Peekaobt Config (*.cfg);;All files (*)")
    #    pbot.setCommand("peekabot")

    #    disp=extserver.Server("DISPLAY", label="Standalone Display Server")
    #    pbot.filenameField("CONFIG",
    #            label="Configuration", filter="Peekaobt Config (*.cfg)")
    #    disp.setCommand("${COGX_ROOT}/output/bin/display-server")

    #    self.mProperties.addServers([build, player, pbot, disp])


if __name__== "__main__":
    app = QtGui.QApplication(sys.argv)
    myapp = CConfigWidget(None)
    myapp.show()
    sys.exit(app.exec_())

