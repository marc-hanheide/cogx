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
import java.util.ArrayList;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;

import org.eclipse.swt.SWT;
import org.eclipse.swt.custom.TableEditor;
import org.eclipse.swt.events.SelectionAdapter;
import org.eclipse.swt.events.SelectionEvent;
import org.eclipse.swt.layout.RowData;
import org.eclipse.swt.layout.RowLayout;
import org.eclipse.swt.widgets.*;

import balt.core.connectors.FrameworkConnectionException;
import balt.core.data.FrameworkQuery;
import cast.cdl.guitypes.*;
import cast.cdl.ui.*;
import cast.ui.UIUtils;
import cast.ui.architecture.GraphicalUI.AppendRunnable;
import cast.ui.architecture.interfaces.ComponentStatusPullInterface.ComponentStatusPullConnectorOut;

/**
 * Class that connects to a CAATUIComponent and manages data exchange
 * with the UI.
 * 
 * @author nah
 */
public class ComponentMonitor {

    private FrameworkQuery m_emptyQuery;

    private String m_id;
    private ComponentStatusPullConnectorOut m_connection;

    private ReadWriteLock m_readWriteLock;

    private boolean m_printing;
    private boolean m_logging;
    private boolean m_debugging;
    private Button m_logCheck;
    private Button m_printCheck;
    private Button m_debugCheck;

    private PrintStream m_outStream;

    private Composite m_outputComposite;

    private Text m_printTextArea;

    private Text m_logTextArea;

    private Text m_debugTextArea;

    private Display m_display;

    private ArrayList<ComponentMonitorListener> m_listeners;

    private ComponentStatus m_componentStatus;

    private String m_printPrefix;
    private String m_logPrefix;
    private String m_debugPrefix;

    public ComponentMonitor(String _id) {
        m_id = _id;

        m_printPrefix = "[PRINT " + m_id + ": ";
        m_logPrefix = "[LOG " + m_id + ": ";
        m_debugPrefix = "[DEBUG " + m_id + ": ";

        m_emptyQuery = new FrameworkQuery(_id, "");
        m_outStream = System.out;

        m_listeners = new ArrayList<ComponentMonitorListener>(1);

        m_componentStatus = new ComponentStatus();

        m_readWriteLock = new ReentrantReadWriteLock(true);
    }

    private void changeEvent(ComponentEventType _type) {
        ComponentMonitorChangeEvent evt = new ComponentMonitorChangeEvent(
            this, _type);
        for (ComponentMonitorListener listener : m_listeners) {
            listener.componentMonitorChanged(evt);
        }
    }

    /**
     * @param _tree
     * @param _treeItem
     * @return
     */
    private TableItem createGUIItems(Table _table) {

        TableItem row = new TableItem(_table, SWT.NONE);
        row.setText(0, m_id);
        // saitem.setBackground(SWT);

        TableEditor editor = new TableEditor(_table);
        m_printCheck = new Button(_table, SWT.CHECK | SWT.CENTER);

        m_printCheck.addSelectionListener(new SelectionAdapter() {

            public void widgetSelected(SelectionEvent e) {
                if (m_printing) {
                    m_printing = false;
                }
                else {
                    m_printing = true;
                }
            }
        });
        m_printCheck.pack();
        editor.grabHorizontal = true;
        editor.setEditor(m_printCheck, row, 1);

        editor = new TableEditor(_table);
        m_logCheck = new Button(_table, SWT.CHECK | SWT.CENTER);
        m_logCheck.pack();
        m_logCheck.addSelectionListener(new SelectionAdapter() {

            public void widgetSelected(SelectionEvent e) {
                if (m_logging) {
                    m_logging = false;
                }
                else {
                    m_logging = true;
                }
            }
        });
        editor.grabHorizontal = true;
        editor.setEditor(m_logCheck, row, 2);

        editor = new TableEditor(_table);
        m_debugCheck = new Button(_table, SWT.CHECK | SWT.CENTER);
        m_debugCheck.pack();
        m_debugCheck.addSelectionListener(new SelectionAdapter() {

            public void widgetSelected(SelectionEvent e) {
                if (m_debugging) {
                    m_debugging = false;
                }
                else {
                    m_debugging = true;
                }
            }
        });
        editor.grabHorizontal = true;
        editor.setEditor(m_debugCheck, row, 3);
        return row;
    }

    public void addComponentMonitorListener(
            ComponentMonitorListener _listener) {
        m_listeners.add(_listener);
    }

    /**
     * @param _data
     */
    public void componentEvent(ComponentEvent _data) {
    // switch (_data.m_event.value()) {
    // case ComponentEventType._ADD:
    // m_totalAdds++;
    // changeEvent(ComponentEventType.ADD);
    // break;
    // case ComponentEventType._OVERWRITE:
    // m_totalOverwrites++;
    // changeEvent(ComponentEventType.OVERWRITE);
    // break;
    // case ComponentEventType._DELETE:
    // m_totalDeletes++;
    // changeEvent(ComponentEventType.DELETE);
    // break;
    // case ComponentEventType._GET:
    // m_totalReads++;
    // changeEvent(ComponentEventType.GET);
    // break;
    // case ComponentEventType._PROPOSED:
    // m_totalProposals++;
    // changeEvent(ComponentEventType.PROPOSED);
    // break;
    // case ComponentEventType._START:
    // m_totalStarts++;
    // changeEvent(ComponentEventType.START);
    // break;
    // case ComponentEventType._END:
    // m_totalEnds++;
    // changeEvent(ComponentEventType.END);
    // break;
    // default:
    // break;
    // }
    }

    public int getChangeQueueLength() {
        m_readWriteLock.readLock().lock();
        int val = m_componentStatus.m_changeQueue;
        m_readWriteLock.readLock().unlock();
        return val;
    }

    /**
     * @return the id
     */
    public String getID() {
        return m_id;
    }

    /**
     * @return the outputComposite
     */
    public Composite getOutputComposite() {
        return m_outputComposite;
    }

    /**
     * @return
     * @throws CASTUIException
     */
    public ComponentStatus getStatus() {
        return m_componentStatus;
    }

    public TableItem getTableItem(Table _table) {

        // System.out.println("ComponentMonitor.getTreeItem()");
        TableItem saitem = createGUIItems(_table);

        // get some values
        if (m_connection != null) {
            try {
                ComponentStatus logOb = m_connection.pull(m_emptyQuery);
                m_printing = true;
                m_printCheck.setSelection(m_printing);
                m_logging = logOb.m_log;
                m_logCheck.setSelection(m_logging);
                m_debugging = logOb.m_debug;
                m_debugCheck.setSelection(m_debugging);
            }
            catch (FrameworkConnectionException e) {
                e.printStackTrace();
                System.exit(1);
            }
        }

        return saitem;
    }

    /**
     * @return
     */
    public int getTotalAdds() {
        m_readWriteLock.readLock().lock();
        int val = m_componentStatus.m_totalAdds;
        m_readWriteLock.readLock().unlock();
        return val;
    }

    public int getTotalDeletes() {
        m_readWriteLock.readLock().lock();
        int val = m_componentStatus.m_totalDeletes;
        m_readWriteLock.readLock().unlock();
        return val;
    }

    public int getTotalEnds() {
        m_readWriteLock.readLock().lock();
        int val = m_componentStatus.m_totalEnds;
        m_readWriteLock.readLock().unlock();
        return val;
    }

    public int getTotalEventsReceived() {
        m_readWriteLock.readLock().lock();
        int val = m_componentStatus.m_totalChangeEventsReceived;
        m_readWriteLock.readLock().unlock();
        return val;
    }

    public int getTotalFilteredEvents() {
        m_readWriteLock.readLock().lock();
        int val = m_componentStatus.m_totalChangeEventsFiltered;
        m_readWriteLock.readLock().unlock();
        return val;
    }

    public int getTotalOverwrites() {
        m_readWriteLock.readLock().lock();
        int val = m_componentStatus.m_totalOverwrites;
        m_readWriteLock.readLock().unlock();
        return val;
    }

    public int getTotalProposals() {
        m_readWriteLock.readLock().lock();
        int val = m_componentStatus.m_totalProposals;
        m_readWriteLock.readLock().unlock();
        return val;
    }

    /**
     * @return
     */
    public int getTotalReads() {
        m_readWriteLock.readLock().lock();
        int val = m_componentStatus.m_totalReads;
        m_readWriteLock.readLock().unlock();
        return val;
    }

    public int getTotalStarts() {
        m_readWriteLock.readLock().lock();
        int val = m_componentStatus.m_totalStarts;
        m_readWriteLock.readLock().unlock();
        return val;
    }

    /**
     * @return the locked
     */
    public boolean isLocked() {
        m_readWriteLock.readLock().lock();
        boolean val = m_componentStatus.m_locked;
        m_readWriteLock.readLock().unlock();
        return val;
    }

    /**
     * @return the sleeping
     */
    public boolean isSleeping() {
        m_readWriteLock.readLock().lock();
        boolean val = m_componentStatus.m_sleeping;
        m_readWriteLock.readLock().unlock();
        return val;
    }

    
    private ArrayList<TextOutput> m_textOutputBuffer;
    
    
    
    /**
     * @param _data
     */
    public final void outputText(TextOutput _data) {

        //bit of an ugly hack before we work on starting the gui earlier
        
        if (m_display == null) {
            
            if(m_textOutputBuffer == null) {
                m_textOutputBuffer = new ArrayList<TextOutput>();
            }
            m_textOutputBuffer.add(_data);
//            System.out.println(m_printPrefix + " " + _data.m_string);
            return;
        }
        //if display has been set, and we have buffered data
        else if(m_textOutputBuffer != null) {
            //set to a local copy
            ArrayList<TextOutput> buffer = m_textOutputBuffer;
            //get rid of buffer
            m_textOutputBuffer = null;
            for (TextOutput output : buffer) {
                outputText(output);
            }
            buffer.clear();
        }
        
        
        // System.out.println("ComponentMonitor.outputText()");
        // System.out.println(_data.m_string);

        String out = _data.m_string + "]\n";

        switch (_data.m_type.value()) {
            case OutputType._PRINT:
                if (m_printing) {
                    m_outStream.print(m_printPrefix + out);
                }
                m_display.syncExec(new AppendRunnable("[" + out,
                    m_printTextArea));

                break;
            case OutputType._LOG:
                if (m_logging) {
                    m_outStream.print(m_logPrefix + out);
                }
                m_display.syncExec(new AppendRunnable("[" + out,
                    m_logTextArea));

                break;
            case OutputType._DEBUG:
                if (m_debugging) {
                    m_outStream.print(m_debugPrefix + out);
                }
                m_display.syncExec(new AppendRunnable("[" + out,
                    m_debugTextArea));
                break;
        }
    }

    /**
     * @param _conn
     */
    public void setConnection(ComponentStatusPullConnectorOut _conn) {
        m_connection = _conn;
    }

    /**
     * @param _display
     */
    public void setDisplay(Display _display) {
        m_display = _display;
    }

    /**
     * Set the parent of the area where text views can be created.
     * 
     * @param _compOutputComp
     */
    public void setOutputArea(Composite _compOutputComp) {
        m_outputComposite = new Composite(_compOutputComp,
            SWT.NO_REDRAW_RESIZE);
        // m_outputComposite.setVisible(false);

        RowLayout outputCompLayout = new RowLayout();
        outputCompLayout.type = SWT.VERTICAL;
        outputCompLayout.pack = false;
        outputCompLayout.wrap = false;
        outputCompLayout.fill = true;

        m_outputComposite.setLayout(outputCompLayout);

        m_printTextArea = new Text(m_outputComposite, SWT.BORDER
            | SWT.V_SCROLL);
        m_printTextArea.setEditable(false);

        m_logTextArea = new Text(m_outputComposite, SWT.BORDER
            | SWT.V_SCROLL);
        m_logTextArea.setEditable(false);
        m_debugTextArea = new Text(m_outputComposite, SWT.BORDER
            | SWT.V_SCROLL);
        m_debugTextArea.setEditable(false);

        // GridData data = new GridData(SWT.FILL, SWT.FILL, true,
        // false);
        // m_printTextArea.setLayoutData(data);

        // m_printTextArea.

        m_outputComposite.addListener(SWT.Resize, new Listener() {

            public void handleEvent(Event e) {
                RowData data = new RowData();
                data.height = m_outputComposite.getSize().y / 3 - 10;
                data.width = m_outputComposite.getSize().x - 30;
//                System.out.println(m_outputComposite.getSize().x + " "
//                    + m_outputComposite.getSize().y);
                m_printTextArea.setLayoutData(data);
                m_logTextArea.setLayoutData(data);
                m_debugTextArea.setLayoutData(data);
            }
        });

    }

    /**
     * @param _outStream
     *            the outStream to set
     */
    public void setOutStream(PrintStream _outStream) {
        m_outStream = _outStream;
    }

    public void update() throws CASTUIException {
        try {
            ComponentStatus csUpdate = m_connection.pull(m_emptyQuery);
            if (UIUtils.changed(m_componentStatus, csUpdate)) {

                m_readWriteLock.writeLock().lock();
                m_componentStatus = csUpdate;
                m_readWriteLock.writeLock().unlock();

                changeEvent(ComponentEventType.END); // dummy type

            }
        }
        catch (FrameworkConnectionException e) {
            throw new CASTUIException("Connection failure", e);
        }
    }

}
