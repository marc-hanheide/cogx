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
import time
import cogxv11n.DisplayClient

class V11nTestDisplayClient(cast.core.CASTComponent):
    def __init__(self):
        cast.core.CASTComponent.__init__(self)
        self.m_display = cogxv11n.DisplayClient.CDisplayClient()

    def configureComponent(self, config):
       self.m_display.configureDisplayClient(config)

    def startComponent(self):
        self.m_display.connectIceClient(self)

    def runComponent(self):
        self.m_display.setHtml("v11n.python.setHtml", "001", "This is a message from V11nTestDisplayClient")
        while self.isRunning():
            sleepComponent(100)
            pass
