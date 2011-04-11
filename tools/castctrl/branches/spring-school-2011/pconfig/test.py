#!/usr/bin/python
# vim:set fileencoding=utf-8 sw=4 ts=8 et:vim #

import os, sys
from PyQt4 import QtCore, QtGui
from PyQt4.QtCore import Qt
from editors import ICustomEditorBase, CTextEditor, CFilenameEditor, CStringItemEditor
#import extserver
from propeditor import *

if __name__== "__main__":
    class Window(QtGui.QWidget):
        def __init__(self, parent):
            QtGui.QWidget.__init__(self, parent)
            self.createUi()
            self.installEditors()

        def installEditors(self):
            self.itemDelegate = CItemDelegate(self)
            self.editFactory = EditorFactory()
            self.itemDelegate.setItemEditorFactory(self.editFactory)
            self.wItems.setItemDelegate(self.itemDelegate)
            qet = QtGui.QAbstractItemView
            self.wItems.setEditTriggers(qet.DoubleClicked | qet.SelectedClicked | qet.EditKeyPressed)

        def createUi(self):
            self.mProperties = CPropertyTreeModel()
            self.demoServers_load()
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


        def demoServers_load(self):
            import manager
            am = manager.CAppManager()
            srvrs = am.discoverServers("mainservers.txt")
            self.mProperties.addServers(srvrs)
            srvrs = am.discoverServers("cogxservers.txt")
            self.mProperties.addServers(srvrs)

        def demoData(self):
            import extserver
            prg = CProcessNode(self.mProperties.rootNode)
            prop = CPropertyNode(prg)
            prop.property = extserver.CStringProperty("Good")
            prg.propertyList.append(prop)
            prop = CPropertyNode(prg)
            prop.property = extserver.CFilenameProperty("Filename")
            prg.propertyList.append(prop)
            self.mProperties.rootNode.processList.append(prg)

            prg = CProcessNode(self.mProperties.rootNode)
            prop = CPropertyNode(prg)
            prop.property = extserver.CStringProperty("Good", label="Good 2")
            prg.propertyList.append(prop)
            prop = CPropertyNode(prg)
            prop.property = extserver.CFilenameProperty("Filename", label="Filename 2")
            prg.propertyList.append(prop)
            self.mProperties.rootNode.processList.append(prg)

        def demoServers_make(self):
            build = extserver.Server("BUILD", label="Build", group="Build")
            build.stringField("BUILDDIR", label="Build directory", default="${COGX_ROOT}/BUILD")
            build.stringItemField("PROFILE", label="Profile", items=["Debug", "Release"], default="Release")
            build.setCommand("make [TARGET]", workdir="[BUILDDIR]")

            player = extserver.Server("PLAYER", label="Player")
            player.filenameField("CONFIG",
                    label="Configuration", filter="Player Config (*.cfg);;All files (*)")
            player.integerField("PORT", label="Port", default=0, range=(1000, 40000))
            player.floatField("WEIGHT", label="Weight", default=0, range=(1000, 40000))
            player.setCommand("player [CONFIG]")

            pbot = extserver.Server("PEEKABOT", label="Peekabot")
            pbot.filenameField("CONFIG",
                    label="Configuration", filter="Peekaobt Config (*.cfg);;All files (*)")
            pbot.setCommand("peekabot")

            disp=extserver.Server("DISPLAY", label="Standalone Display Server")
            pbot.filenameField("CONFIG",
                    label="Configuration", filter="Peekaobt Config (*.cfg)")
            disp.setCommand("${COGX_ROOT}/output/bin/display-server")

            self.mProperties.addServers([build, player, pbot, disp])


    app = QtGui.QApplication(sys.argv)
    myapp = Window(None)
    myapp.show()
    sys.exit(app.exec_())

