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
from Visualization import *

class CDisplayClient:
    def __init__(self):
        self.m_ServerName = "display.srv"
        self.m_Server = None
        self.m_Owner = None
        pass

    def configureDisplayClient(self, config):
        if config.has_key("--displayserver"):
            self.m_ServerName = config["--displayserver"]
            print self.m_ServerName

    def connectIceClient(self, owner):
        self.m_Owner = owner
        try:
            self.m_Server = owner.getIceServer(self.m_ServerName, DisplayInterface, DisplayInterfacePrx)
        except Exception as e:
            print e
            self.m_Server = None

    def getComponentId(self):
        if self.m_Owner == None: return "[None]"
        return self.m_Owner.getComponentID()

    def getEventClientId(self):
        id = Ice.Identity()
        id.name = self.getComponentId()
        id.category = "Visualization.EventReceiver"
        return id

    def installEventReceiver(self):
        pass

    def setImage(self, id, videoImage)
        if self.m_Server == None: return
        self.m_Server.setImage(id, videoImage)

    def setRawImage(self, id, width, height, channels, data)
        if self.m_Server == None: return
        self.m_Server.setRawImage(id, width, height, channels, data)

    def setCompressedImage(self, id, data)
        if self.m_Server == None: return
        self.m_Server.setCompressedImage(id, data)

    def setHtml(self, id, partId, htmlData):
        if self.m_Server == None: return
        self.m_Server.setHtml(id, partId, htmlData)

    def setHtmlHead(self, id, partId, htmlData):
        if self.m_Server == None: return
        self.m_Server.setHtmlHead(id, partId, htmlData)

    def setHtmlForm(self, id, partId, htmlData):
        if self.m_Server == None: return
        iceid = self.getEventClientId()
        self.m_Server.setHtmlHead(iceid, id, partId, htmlData)

    def setObject(self, id, partId, svgObject):
        if self.m_Server == None: return
        self.m_Server.setObject(id, partId, svgObject)

    def setLuaGlObject(self, id, partId, script):
        if self.m_Server == None: return
        self.m_Server.setLuaGlObject(id, partId, script)

