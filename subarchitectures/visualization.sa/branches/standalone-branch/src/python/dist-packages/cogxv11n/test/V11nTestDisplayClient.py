#!/usr/bin/env python
# vim:set fileencoding=utf-8 sw=4 ts=8 et:vim
#
# Author:  Marko Mahnič
# Created: Jul 2010 
#
# © Copyright 2010 Marko Mahnič. 
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 2 of the License, or
# (at your option) any later version.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

import castinit    # this will add stuff to sys.path
import cast.core
import time, random
import cogxv11n.core.DisplayClient as DisplayClient

class MyDisplayClient(DisplayClient.CDisplayClient):
    def __init__(self, castComponent):
        DisplayClient.CDisplayClient.__init__(self)
        self.m_test = castComponent

    def handleEvent(self, event):
        if self.m_test == None: return
        t = self.m_test
        if event.sourceId == "cb.test.onoff":
            if event.data == "0": t.m_ckTestValue = 0
            else: t.m_ckTestValue = 2
            if t.m_ckTestValue > 0: t.appendMessage("Check box " + event.sourceId + " is ON")
            else: t.appendMessage("Check box " + event.sourceId + " is OFF")

        if event.sourceId == "button.test":
           t.appendMessage("Button " + event.sourceId + " PRESSED")

    def getControlState(self, ctrlId):
        if self.m_test == None: return
        t = self.m_test
        if ctrlId == "cb.test.onoff":
            if t.m_ckTestValue != 0: return "2"
            else: return "0"
        return ""

    def handleForm(self, id, partId, fields):
        if self.m_test == None: return
        t = self.m_test
        #t.log("PYTHON handleForm " + id + ":" + partId)
        if id == "v11n.python.setHtmlForm" and partId == "101":
            if fields.has_key("textfield"):
                t.m_textField = fields["textfield"]
                t.log("Got textfield: " + t.m_textField);
                t.appendMessage(t.m_textField);

    def getFormData(self, id, partId, fields):
        if self.m_test == None: return False
        t = self.m_test
        #t.log("PYTHON getFormData " + id + ":" + partId)
        if id == "v11n.python.setHtmlForm" and partId == "101":
            fields["textfield"] = t.m_textField
            return True

        return False

class V11nTestDisplayClient(cast.core.CASTComponent):
    def __init__(self):
        cast.core.CASTComponent.__init__(self)
        self.m_display = MyDisplayClient(self)
        self.m_moveCount = 0
        self.m_moverBoxRot = 0
        self.m_GraphData = []
        self.m_textField = "A message from Python"
        self.m_ckTestValue = 2
        self._msgid = 9000

    def configureComponent(self, config):
       self.m_display.configureDisplayClient(config)

    def startComponent(self):
        self.m_display.connectIceClient(self)
        self.m_display.installEventReceiver()

    def runComponent(self):
        self.makeHtml()
        self.makeHtmlForm()
        self.makePusher()
        self.makeSvgGraph()
        while self.isRunning():
            time.sleep(0.1)
            self.movePusher()
            self.updateSvgGraph()

    def makeHtml(self):
        # A multi-part HTML document.
        # Parts will be added every time the form (setHtmlForm below) is submitted (see handleForm).
        self.m_display.setHtml("v11n.python.setHtml", "001", "This is a message from V11nTestDisplayClient.");

        # Test of gui elements
        # Messages will be added to the document when events happen (see handleEvent).
        self.m_display.addCheckBox("v11n.python.setHtml", "cb.test.onoff", "Test On Off");
        self.m_display.addButton("v11n.python.setHtml", "button.test", "Test Button");

    def makeHtmlForm(self):
        # A simple form.
        # Events will be handled in MyDisplayClient.handleForm().
        # Form data will be retreived in MyDisplayClient.getFormData().
        self.m_display.setHtmlForm("v11n.python.setHtmlForm", "101",
              "Edit me: <input type='text' name='textfield' value='Empty' />");

    # Called from handleForm() (after a form is submitted)
    # Appends a text message (a html chunk) to an HTML object.
    # Chunks are sorted by their string ID (but the order depends on C++ std::map implementation).
    def appendMessage(self, message):
        self._msgid += 1
        self.m_display.setHtml("v11n.python.setHtml", "%04d" % self._msgid, "<br>" + message);

    def makePusher(self):
        f = open("subarchitectures/visualization.sa/src/c++/core/object/gllua/test/pusher.luagl")
        script = "".join(f.readlines())
        f.close()
        self.m_display.setLuaGlObject("v11n.python.Pusher", "Pusher", script)

    def movePusher(self):
        ss = ""
        self.m_moveCount = (self.m_moveCount + 1) % 100
        if self.m_moveCount % 2 == 0:
            mdir = random.randint(0, 3)
            if   mdir == 0: ss += "move(1,0);\n"
            elif mdir == 1: ss += "move(0,1);\n"
            elif mdir == 2: ss += "move(-1,0);\n"
            elif mdir == 3: ss += "move(0,-1);\n"

        self.m_moverBoxRot = (self.m_moverBoxRot + 1) % 36
        ss += "boxTurn=%d;\n" % (self.m_moverBoxRot * 10)
        ss += "DispList:setDirty('pusher.box.rotation');\n"
        self.m_display.setLuaGlObject("v11n.python.Pusher", "Pusher", ss)

    def makeSvgGraph(self):
        ss = ""
        ss += "<svg viewbox='0 0 242 162'>"
        ss += "<rect x='0' y='0' width='242' height='162' fill='white' stroke='blue' stroke-width='1' />"
        for y in [40, 80, 120]:
            ss += "<polyline fill='none' stroke='#a0a0ff' stroke-width='1' points='1,%d 241,%d' />" % (y, y)
        ss += "</svg>"
        self.m_display.setObject("v11.python.Graph", "000_background", ss)

        ss = ""
        ss += "<svg viewbox='0 0 242 162'>";
        ss += "<text x='2' y='160' font-size='12' fill='blue'>%d</text>" % 10
        ss += "<text x='2' y='16' font-size='12' fill='blue'>%d</text>" % 88
        ss += "</svg>";
        self.m_display.setObject("v11.python.Graph", "999_labels", ss);
        self.m_GraphData = [ random.randint(10, 80) for i in xrange(32) ]

    def updateSvgGraph(self):
        self.m_GraphData = self.m_GraphData[1:] + [random.randint(10, 80)]
        ss = ""
        text = ""
        ss += "<svg viewbox='0 0 242 162'>"
        ss += "<polyline fill='none' stroke='none' stroke-width='0' points='0,0 242,162' />"
        ss += "<polyline fill='none' stroke='red' stroke-width='1' points='"
        for i,v in enumerate(self.m_GraphData):
           p = 1.0 - float(v - 10) / (88.0 - 10.0);
           x, y = int(240.0*i/32+0.5), int(160.0*p+0.5)
           ss += "%d,%d " % (x, y)
           if v % 10 == 0:
               text += "<text x='%d' y='%d' font-size='10' fill='green'>%d</text>" % (x, y, v)

        ss += "' />\n"
        self.m_display.setObject("v11.python.Graph", "500_lines", ss + text + "</svg>")

