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

import Ice
import cast
from Visualization import *

# Get servant category as given by the C++ implementation (toServantCategory)
# (The right place for this function would be CAST.)
def castServantCategory(IceInterface):
    category = IceInterface.ice_staticId().lstrip(":").replace("::", ".")
    return category

class _EventReceiverImpl (EventReceiver):
    def __init__(self, displayClient):
        self.m_Client = displayClient

    def handleEvent(self, event, ctx):
        if self.m_Client != None: self.m_Client.handleEvent(event)

    def getControlState(self, ctrlId, ctx):
        if self.m_Client != None: return self.m_Client.getControlState(ctrlId)
        return "";

    def handleForm(self, id, partId, fields, ctx):
        if self.m_Client != None: self.m_Client.handleForm(id, partId, fields)

    def getFormData(self, id, partId, ctx):
        if self.m_Client != None:
            # make the interface consistent with C++ and java
            fields = {}
            rv = self.m_Client.getFormData(id, partId, fields)
            if rv: return (True, fields)
        return (False, None)

    def onDialogValueChanged(self, dialogId, name, value, ctx):
        if self.m_Client != None: self.m_Client.onDialogValueChanged(dialogId, name, value)

    def handleDialogCommand(self, dialogId, name, value, ctx):
        if self.m_Client != None: self.m_Client.handleDialogCommand(dialogId, name, value)

class CDisplayClient:
    def __init__(self):
        self.m_ServerName = "display.srv"
        self.m_ServerHost = ""
        self.m_Server = None
        self.m_Owner = None
        self.m_EventReceiver = None
        self._category = None
        pass

    def configureDisplayClient(self, config):
        if config.has_key("--displayserver"):
            self.m_ServerName = config["--displayserver"].strip()
            print self.m_ServerName
        if config.has_key("--standalone-display-host"):
           self.m_ServerHost = config["--standalone-display-host"].strip()
           if self.m_ServerHost == "/no": self.m_ServerHost = ""
           print self.m_ServerHost

    def connectToStandaloneHost(self, owner):
        try:
            dispCategory = castServantCategory(DisplayInterface)
            prx = owner.getIceServerManual(
                V11NSTANDALONENAME, dispCategory, self.m_ServerHost, V11NSTANDALONEPORT)

            self.m_Server = DisplayInterfacePrx.checkedCast(prx)
            owner.println("DisplayClient(python) connected to standalone server on '%s'" % (self.m_ServerHost));
        except Exception as e:
            owner.println(
                "*** DisplayClient(python): standalone server not found on host '%s'" % (self.m_ServerHost))
            owner.println("%s" % e)
            self.m_Server = None

    def connectIceClient(self, owner):
        self.m_Owner = owner
        if len(self.m_ServerHost) > 0:
            self.connectToStandaloneHost(owner)
            return

        try:
            self.m_Server = owner.getIceServer(self.m_ServerName, DisplayInterface, DisplayInterfacePrx)
        except Exception as e:
            owner.println("*** DisplayClient(python): DisplayServer component not found")
            self.m_Server = None

        if self.m_Server != None:
            try:
                self.m_ServerHost = self.m_Server.getStandaloneHost()
                if len(self.m_ServerHost) > 0:
                    self.m_Server = None
                    owner.debug("DisplayClient(python) Redirecting connection to standalone display server.")
                    self.connectToStandaloneHost(owner)
            except Exception as e:
                pass

    def getComponentId(self):
        if self.m_Owner == None: return "[None]"
        return self.m_Owner.getComponentID()

    def getEventClientId(self):
        id = Ice.Identity()
        id.name = self.getComponentId()
        if self._category == None:
            self._category = castServantCategory(EventReceiver)
        id.category = self._category # "Visualization.EventReceiver"
        return id

    def installEventReceiver(self):
        if self.m_Owner == None:
            raise Exception("CDisplayClient(python): connectIceClient() must be called before installEventReciever().")
        if self.m_Server == None:
            self.m_Owner.println(" *** CDisplayClient(python): Server not connected.")
            return
        if self.m_EventReceiver != None:
            self.m_Owner.println(" *** CDisplayClient(python): The client already has an EventReceiver.")
            return

        iceid = self.getEventClientId()
        self.m_EventReceiver = _EventReceiverImpl(self)
        self.m_Owner.registerIceServer(EventReceiver, self.m_EventReceiver)

        #self.m_Server.addClient(iceid)
        myHost = self.m_Owner.getComponentManager().getComponentDescription(self.getComponentId()).hostName
        self.m_Server.addClient(iceid, myHost, cast.cdl.PYTHONSERVERPORT)

    def setImage(self, id, videoImage):
        if self.m_Server == None: return
        self.m_Server.setRawImage(id, videoImage.width, videoImage.height, 3, videoImage.data)

    def setRawImage(self, id, width, height, channels, data):
        if self.m_Server == None: return
        self.m_Server.setRawImage(id, width, height, channels, data)

    def setCompressedImage(self, id, data):
        if self.m_Server == None: return
        self.m_Server.setCompressedImage(id, data)

    def setHtml(self, id, partId, htmlData):
        if self.m_Server == None: return
        self.m_Server.setHtml(id, partId, htmlData)

    def setHtmlHead(self, id, partId, htmlData):
        if self.m_Server == None: return
        self.m_Server.setHtmlHead(id, partId, htmlData)

    def setActiveHtml(self, id, partId, htmlData):
        if self.m_Server == None: return
        iceid = self.getEventClientId()
        self.m_Server.setActiveHtml(iceid, id, partId, htmlData)

    def setHtmlForm(self, id, partId, htmlData):
        if self.m_Server == None: return
        iceid = self.getEventClientId()
        self.m_Server.setHtmlForm(iceid, id, partId, htmlData)

    def setHtmlFormData(self, id, partId, fieldDict):
        if self.m_Server == None: return
        self.m_Server.setHtmlFormData(id, partId, fieldDict)

    def removeObject(self, id):
        if self.m_Server == None: return
        self.m_Server.removeObject(id)

    def removePart(self, id, partId):
        if self.m_Server == None: return
        self.m_Server.removePart(id, partId)

    def setObject(self, id, partId, svgObject):
        if self.m_Server == None: return
        self.m_Server.setObject(id, partId, svgObject)

    def setLuaGlObject(self, id, partId, script):
        if self.m_Server == None: return
        self.m_Server.setLuaGlObject(id, partId, script)

    #----------------------------------------------------------------- 
    # GUI elements
    #----------------------------------------------------------------- 
    def addButton(self, viewId, ctrlId, label):
        if self.m_Server == None: return
        iceid = self.getEventClientId()
        self.m_Server.addButton(iceid, viewId, ctrlId, label)

    def addCheckBox(self, viewId, ctrlId, label):
        if self.m_Server == None: return
        iceid = self.getEventClientId()
        self.m_Server.addCheckBox(iceid, viewId, ctrlId, label)

    def addAction(self, viewId, actionInfo):
        if self.m_Server == None: return
        iceid = self.getEventClientId()
        self.m_Server.addAction(iceid, viewId, actionInfo)

    #----------------------------------------------------------------- 
    # Active client callbacks - to be reimplemented
    #----------------------------------------------------------------- 
    def handleEvent(self, event):
        pass

    def getControlState(self, ctrlId):
        return ""

    def handleForm(self, id, partId, fieldDict):
        pass

    def getFormData(self, id, partId, fieldDict):
        return False

    def onDialogValueChanged(self, dialogId, name, value):
        pass

    def handleDialogCommand(self, dialogId, name, value):
        pass

