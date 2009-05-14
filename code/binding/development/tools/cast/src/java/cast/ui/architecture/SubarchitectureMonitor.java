/*
 * CAST - The CoSy Architecture Schema Toolkit
 *
 * Copyright (C) 2006-2007 Nick Hawes
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation; either version 2.1 of
 * the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

/**
 * 
 */
package cast.ui.architecture;

import java.io.PrintStream;
import java.util.*;

import org.eclipse.swt.SWT;
import org.eclipse.swt.widgets.Tree;
import org.eclipse.swt.widgets.TreeItem;

import balt.corba.autogen.FrameworkBasics.ProcessDescription;
import cast.configuration.SubarchitectureConfiguration;

/**
 * A class that monitors a subarchitecture, and the contained
 * components, for the ui.
 * 
 * @author nah
 */
public class SubarchitectureMonitor {

    private String m_id;

    private Hashtable<String, ComponentMonitor> m_dataDrivenComponents;
    private Hashtable<String, ComponentMonitor> m_goalDrivenComponents;
    private ComponentMonitor m_workingMemory;
    private ComponentMonitor m_taskManager;
    private String m_workingMemoryID;
    private String m_taskManagerID;
    
    public Collection<ComponentMonitor> getComponents() {
        ArrayList<ComponentMonitor> components = new ArrayList<ComponentMonitor>(m_dataDrivenComponents.values());
        components.addAll(m_goalDrivenComponents.values());
        components.add(m_workingMemory);
        components.add(m_taskManager);
        return components;
    }
    
    
    /**
     * @return the taskManager
     */
    public ComponentMonitor getTaskManager() {
        return m_taskManager;
    }
    
    
    /**
     * @return the workingMemory
     */
    public ComponentMonitor getWorkingMemory() {
        return m_workingMemory;
    }
    
    /**
     * @return the dataDrivenComponents
     */
    public Collection<ComponentMonitor> getDataDrivenComponents() {
        return m_dataDrivenComponents.values();
    }

    /**
     * @return the goalDrivenComponents
     */
    public Collection<ComponentMonitor> getGoalDrivenComponents() {
        return m_goalDrivenComponents.values();
    }

    /**
     * @param _config
     */
    public SubarchitectureMonitor(SubarchitectureConfiguration _config, PrintStream _outStream) {
        m_id = _config.getID();
        createMonitors(_config, _outStream);
    }

    /**
     * @param _config
     * @param _stream 
     */
    private void createMonitors(SubarchitectureConfiguration _config, PrintStream _stream) {

        ArrayList<ProcessDescription> dataDrivenProcesses = _config
            .getUnmanagedComponents();
        m_dataDrivenComponents = new Hashtable<String, ComponentMonitor>(
            dataDrivenProcesses.size());
        for (ProcessDescription desc : dataDrivenProcesses) {
            ComponentMonitor monitor = new ComponentMonitor(
                desc.m_processName);
            m_dataDrivenComponents.put(monitor.getID(), monitor);
        }

        ArrayList<ProcessDescription> goalDrivenProcesses = _config
            .getManagedComponents();
        m_goalDrivenComponents = new Hashtable<String, ComponentMonitor>(
            goalDrivenProcesses.size());
        for (ProcessDescription desc : goalDrivenProcesses) {
            ComponentMonitor monitor = new ComponentMonitor(
                desc.m_processName);
            m_goalDrivenComponents.put(monitor.getID(), monitor);
        }

        m_workingMemoryID = _config.getWorkingMemoryConfig().m_processName;
        m_workingMemory = new ComponentMonitor(m_workingMemoryID);
        
        m_taskManagerID = _config.getTaskManagerConfig().m_processName;
        m_taskManager = new ComponentMonitor(m_taskManagerID);
        
    }

    /**
     * @return the id
     */
    public String getID() {
        return m_id;
    }

    /**
     * @param _component
     * @return
     * @throws CASTUIException
     */
    public ComponentMonitor getComponent(String _component)
            throws CASTUIException {

        ComponentMonitor monitor = m_dataDrivenComponents
            .get(_component);

        if (monitor == null) {
            monitor = m_goalDrivenComponents.get(_component);
        }

        if (monitor == null & _component.equals(m_taskManagerID)) {
            monitor = m_taskManager;
        }
        
        if (monitor == null & _component.equals(m_workingMemoryID)) {
            monitor = m_workingMemory;
        }
        
        if (monitor == null) {
            throw new CASTUIException(
                "unknown component in subarchitecture " + m_id + ": "
                    + _component);
        }

        return monitor;
    }

    public TreeItem getTreeItem(Tree _tree) {
        TreeItem saitem = new TreeItem(_tree, SWT.NONE);

        saitem.setText(0, m_id);
      
        
        
//        TreeEditor editor = new TreeEditor(_tree);
//        Button button = new Button(_tree, SWT.CHECK);
//        button.pack();
//        editor.minimumWidth = button.getSize().x;
//        editor.horizontalAlignment = SWT.LEFT;
//        editor.setEditor(button, saitem, 1);
//        editor = new TreeEditor(_tree);
//        button = new Button(_tree, SWT.CHECK);
//        button.pack();
//        editor.minimumWidth = button.getSize().x;
//        editor.horizontalAlignment = SWT.LEFT;
//        editor.setEditor(button, saitem, 2);
//        editor = new TreeEditor(_tree);
//        button = new Button(_tree, SWT.CHECK);
//        button.pack();
//        editor.minimumWidth = button.getSize().x;
//        editor.horizontalAlignment = SWT.LEFT;
//        editor.setEditor(button, saitem, 3);

        return saitem;
    }

}
