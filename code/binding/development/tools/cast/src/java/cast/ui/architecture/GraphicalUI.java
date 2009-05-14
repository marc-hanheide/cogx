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

import java.io.*;
import java.util.HashMap;

import org.eclipse.swt.SWT;
import org.eclipse.swt.custom.StackLayout;
import org.eclipse.swt.layout.*;
import org.eclipse.swt.widgets.*;

import cast.configuration.ArchitectureConfiguration;
import cast.cdl.ui.*;
import cast.ui.architecture.interfaces.ComponentStatusPullInterface.ComponentStatusPullConnectorOut;
import cast.ui.architecture.views.ArchitectureView;

/**
 * @author nah
 */
public class GraphicalUI extends UIServer {

    private class MonitorUpdaterThread extends Thread {

        private boolean m_bRun = true;

        /**
         * 
         */
        private void stopUpdating() {
            m_bRun = false;
        }

        /*
         * (non-Javadoc)
         * 
         * @see java.lang.Thread#run()
         */
        @Override
        public void run() {
            try {
//                while (m_bRun) {
//                    synchronized (this) {
//                        wait(100);
//                    }
                    for (ComponentMonitor monitor : m_monitors.values()) {
                        // System.out.println("updating: " +
                        // monitor.getID());
                        monitor.update();
                    }
//                }
            }
//            catch (InterruptedException e) {
//                e.printStackTrace();
//                System.exit(1);
//            }
            catch (CASTUIException e) {
                e.printStackTrace();
                System.exit(1);
            }

        }
    }

    /**
     * 
     */
    private static final String TEXT_TAB = "Output";

    /**
     * 
     */
    private static final String ARCH_TAB = "Arch";

    static class AppendRunnable implements Runnable {

        private String m_text;
        private Text m_output;

        public AppendRunnable(String _text, Text _output) {
            m_text = _text;
            m_output = _output;
        }

        /*
         * (non-Javadoc)
         * 
         * @see java.lang.Runnable#run()
         */
        public void run() {

            // System.out.println("appending: " + m_text);
            m_output.append(m_text);
        }

    }

    private class GUIOutputStream extends OutputStream {

        private StringBuffer m_buffer;

        /**
         * 
         */
        public GUIOutputStream() {
            m_buffer = new StringBuffer();
        }

        /*
         * (non-Javadoc)
         * 
         * @see java.io.OutputStream#flush()
         */
        @Override
        public void flush() throws IOException {
            append(m_buffer.toString());
            m_buffer = new StringBuffer();
        }

        /*
         * (non-Javadoc)
         * 
         * @see java.io.OutputStream#write(int)
         */
        @Override
        public void write(int _b) throws IOException {
            // System.out.println("write: " + _b);

            m_buffer.append((char) _b);
            if ((char) _b == '\n') {
                flush();
            }
        }
    }

    private Display m_display;

    private Shell m_shell;

    private ArchitectureMonitor m_architectureMonitor;

    private TabFolder m_viewTabs;

    private Table m_processTable;

    private Text m_textOutput;

    private PrintStream m_writer;

    private HashMap<String, ComponentMonitor> m_monitors;

    private Composite m_compOutputComp;

    private ComponentMonitor m_currentlySelected = null;

    private StackLayout m_textOutputStack;

    private Composite m_rhs;

    private RowLayout m_rhsLayout;

    private TabItem m_architectureTabItem;

    private MonitorUpdaterThread m_mut;

    private Composite m_lhs;

    /**
     * @param _id
     */
    public GraphicalUI(String _id) {
        super(_id);

        // System.out.println("*********************
        // GraphicalUI.GraphicalUI()");

        m_monitors = new HashMap<String, ComponentMonitor>();
        m_writer = new PrintStream(new GUIOutputStream());
    }

    /**
     * @param _comp
     */
    private void createProcessTree(Composite _comp) {

        // m_processTable = new Table (_comp, SWT.BORDER | SWT.V_SCROLL
        // |
        // SWT.H_SCROLL);
        // 
        // m_processTable.setLinesVisible (true);
        // for (int i=0; i<3; i++) {
        // TableColumn column = new TableColumn(m_processTable,
        // SWT.NONE);
        // column.setWidth (100);
        // }
        //        
        // for (int i=0; i<12; i++) {
        // new TableItem (m_processTable, SWT.NONE);
        // }

        // TableItem [] items = m_processTable.getItems ();
        // for (int i=0; i<items.length; i++) {
        // TableEditor editor = new TableEditor (m_processTable);
        // CCombo combo = new CCombo (m_processTable, SWT.NONE);
        // combo.setText("CCombo");
        // combo.add("item 1");
        // combo.add("item 2");
        // editor.grabHorizontal = true;
        // editor.setEditor(combo, items[i], 0);
        // editor = new TableEditor (m_processTable);
        // Text text = new Text (m_processTable, SWT.NONE);
        // text.setText("Text");
        // editor.grabHorizontal = true;
        // editor.setEditor(text, items[i], 1);
        // editor = new TableEditor (m_processTable);
        // Button button = new Button (m_processTable, SWT.CHECK);
        // button.pack ();
        // editor.minimumWidth = button.getSize ().x;
        // editor.horizontalAlignment = SWT.LEFT;
        // editor.setEditor (button, items[i], 2);
        // }

        m_processTable = new Table(_comp, SWT.BORDER | SWT.SINGLE
            | SWT.V_SCROLL);

        GridData data = new GridData(SWT.CENTER, SWT.FILL, false, true);
        m_processTable.setLayoutData(data);

        m_processTable.setLinesVisible(true);
        m_processTable.setHeaderVisible(true);

        TableColumn column1 = new TableColumn(m_processTable,
            SWT.CENTER);
        TableColumn column2 = new TableColumn(m_processTable,
            SWT.CENTER);
        TableColumn column3 = new TableColumn(m_processTable,
            SWT.CENTER);
        TableColumn column4 = new TableColumn(m_processTable,
            SWT.CENTER);

        // column1.
        // column1.setWidth(200);
        column1.setText("name");
        column2.setText("P");
        column3.setText("L");
        column4.setText("D");

        column1.setResizable(true);
        column2.setResizable(false);
        column3.setResizable(false);
        column4.setResizable(false);

        column2.setWidth(30);
        column3.setWidth(30);
        column4.setWidth(30);

//        if (m_architectureMonitor.getMotiveGenerator() != null) {
//            m_architectureMonitor.getMotiveGenerator().getTableItem(
//                m_processTable);
//            m_architectureMonitor.getMotiveManager().getTableItem(
//                m_processTable);
//        }

        for (SubarchitectureMonitor subarch : m_architectureMonitor) {

            // TableItem saitem = subarch.getTableItem(m_processTable);

            subarch.getWorkingMemory().getTableItem(m_processTable);
            subarch.getTaskManager().getTableItem(m_processTable);

            for (ComponentMonitor monitor : subarch
                .getDataDrivenComponents()) {
                monitor.getTableItem(m_processTable);
            }

            for (ComponentMonitor monitor : subarch
                .getGoalDrivenComponents()) {
                monitor.getTableItem(m_processTable);
            }

            // saitem.setExpanded(true);
            // saitem.get
        }

        column1.pack();
//        column2.pack();
//        column3.pack();
//        column4.pack();

        m_processTable.setSelection(m_processTable.getItem(0));

        m_processTable.addListener(SWT.Selection, new Listener() {

            public void handleEvent(Event e) {
                TableItem[] selection = m_processTable.getSelection();
                ComponentMonitor monitor = m_monitors.get(selection[0]
                    .getText());
                // System.out.println("selected = \"" +
                // selection[0].getText() + "\"");
                if (monitor != null) {
                    m_currentlySelected = monitor;
                    m_textOutputStack.topControl = m_currentlySelected
                        .getOutputComposite();
                    m_compOutputComp.layout();
                    m_compOutputComp.redraw();
                    // m_currentlySelected.textAreaVisible(false);
                    // m_currentlySelected = monitor;
                    // m_currentlySelected.textAreaVisible(true);
                }
            }
        });

        // TreeItem[] items = m_processTree.getItems();
        // TreeEditor editor;
        // for (int i = 0; i < items.length; i++) {
        // editor = new TreeEditor(m_processTree);
        // editor.minimumHeight = 50;
        // editor.setEditor(button, saitem, 3);
        // }

        m_processTable.pack(true);
    }

    /**
     * 
     */
    private void createWindow() {

        // create display and top level window (shell)
        m_display = Display.getDefault();

        // Thread uiThread = m_display.getThread();
        // System.out.println(uiThread.getId());
        // System.out.println(uiThread.getName());
        // System.out.println(uiThread.getClass());

        // if the previous statement didn't create the display, then
        // we're in a lot of trouble!
        if (null == Display.getCurrent()) {
            throw new RuntimeException(
                "Main GUI thread is not current interface thread!");
        }

        m_shell = new Shell(m_display);
        m_shell.setText("CAAT Architecture GUI");

        GridLayout shellLayout = new GridLayout(2, false);
        m_shell.setLayout(shellLayout);

        m_lhs = new Composite(m_shell, SWT.NONE);

        // do the layout details for all the little widgets
        GridData data = new GridData(SWT.CENTER, SWT.FILL, false, true);
        m_lhs.setLayoutData(data);

        setupControls(m_lhs);

        m_rhs = new Composite(m_shell, SWT.NONE);
        m_rhsLayout = new RowLayout();
        m_rhsLayout.type = SWT.VERTICAL;
        m_rhsLayout.pack = false;
        m_rhsLayout.wrap = false;
        m_rhsLayout.fill = false;

        m_rhs.setLayout(m_rhsLayout);
        m_rhs
            .setLayoutData(new GridData(SWT.FILL, SWT.FILL, true, true));

        m_rhs.addListener(SWT.Resize, new Listener() {

            public void handleEvent(Event e) {
                RowData rdata = new RowData();
                rdata.height = m_rhs.getSize().y / 2 - 40;
                rdata.width = m_rhs.getSize().x - 30;
                // System.out.println(m_rhs.getSize().x + " "
                // + m_rhs.getSize().y);
                m_viewTabs.setLayoutData(rdata);
                m_textOutput.setLayoutData(rdata);
            }
        });

        setupViews(m_rhs);

        // final Sash sash = new Sash (rhs, SWT.VERTICAL);

        setupOutputArea(m_rhs);

        m_shell.setSize(1000, 800);
        // m_shell.pack();
        m_shell.open();
    }

    private void linkMonitorsToGUI() {
        boolean first = true;
        for (ComponentMonitor monitor : m_monitors.values()) {
            monitor.setOutStream(m_writer);
            monitor.setOutputArea(m_compOutputComp);
            monitor.setDisplay(m_display);

            if (first) {
                m_currentlySelected = monitor;
                // m_currentlySelected.textAreaVisible(true);
                first = false;
                m_textOutputStack.topControl = m_currentlySelected
                    .getOutputComposite();
                m_compOutputComp.layout();
            }
        }

        // ScrolledComposite comp = new ScrolledComposite(m_viewTabs,
        // SWT.BORDER | SWT.V_SCROLL | SWT.H_SCROLL);

        Composite comp = new Composite(m_viewTabs, SWT.BORDER
            | SWT.V_SCROLL | SWT.H_SCROLL);

        comp.setLayout(new GridLayout(1, true));

        ArchitectureView av = new ArchitectureView(
            m_architectureMonitor, m_display, comp);
        av.setLayoutData(new GridData(GridData.CENTER));

        // comp.pack(true);
        // comp.setExpandVertical(true);
        // comp.setExpandHorizontal(true);
        // comp.setMinWidth(av.getSize().x / 2);
        // comp.setMinHeight(av.getSize().y / 2);

        m_architectureTabItem.setControl(comp);
    }

    private void setupControls(Composite parent) {

        parent.setLayout(new GridLayout(1, true));

        Label label = new Label(parent, SWT.SHADOW_NONE);
        label.setText("Components");

        createProcessTree(parent);
    }

    /**
     * @param _rhs
     */
    private void setupOutputArea(Composite _rhs) {
        m_textOutput = new Text(_rhs, SWT.BORDER | SWT.V_SCROLL);
        m_textOutput.setText("\n\n\n\n\n\n\n"); // hack to determine
        // number of lines
        // initially visible
        m_textOutput.setEditable(false);
        // m_textOutput.re
        // t.set
        // GridData data = new GridData(GridData.FILL, GridData.CENTER,
        // false,
        // false);
        // // GridData data = new GridData(GridData.FILL_HORIZONTAL);
        // m_textOutput.setLayoutData(data);
        // m_textOutput.setSize(600,600);

    }

    private void setupViews(Composite _parent) {
        m_viewTabs = new TabFolder(_parent, SWT.BOTTOM);

        // RowData data = new RowData(SWT.FILL, SWT.FILL, true, true);
        // m_viewTabs.setLayoutData(data);

        m_architectureTabItem = new TabItem(m_viewTabs, SWT.NONE);
        m_architectureTabItem.setText(GraphicalUI.ARCH_TAB);

        TabItem textItem = new TabItem(m_viewTabs, SWT.NONE);
        textItem.setText(GraphicalUI.TEXT_TAB);

        m_compOutputComp = new Composite(m_viewTabs,
            SWT.NO_REDRAW_RESIZE);

        m_textOutputStack = new StackLayout();

        m_compOutputComp.setLayout(m_textOutputStack);

        textItem.setControl(m_compOutputComp);

        // inform the GUI shen selection changed

        // m_viewTabs.addListener(SWT.Selection, new Listener() {
        //
        // public void handleEvent(Event e) {
        // TabItem[] selection = m_viewTabs.getSelection();
        // System.out.println("selected = \""
        // + selection[0].getText() + "\"");
        // if (selection[0].getText().equals(GraphicalUI.ARCH_TAB)) {
        // redrawArchictecture();
        // }
        // else if (selection[0].getText().equals(
        // GraphicalUI.TEXT_TAB)) {
        // redrawComponentTab();
        // }
        // }
        //
        // });
    }

    protected void append(String _s) {
        m_display.syncExec(new AppendRunnable(_s, m_textOutput));
    }

    /*
     * (non-Javadoc)
     * 
     * @see caat.ui.architecture.UIServer#handleTextOutput(java.lang.String,
     *      caat.corba.autogen.CAAT.ui.TextOutput)
     */
    @Override
    protected void handleTextOutput(String _src, TextOutput _data) {
        m_monitors.get(_src).outputText(_data);
    }

    /*
     * (non-Javadoc)
     * 
     * @see caat.ui.architecture.UIServer#handleComponentEvent(java.lang.String,
     *      caat.corba.autogen.CAAT.ui.ComponentEvent)
     */
    @Override
    protected void handleComponentEvent(String _src,
            ComponentEvent _data) {
        m_monitors.get(_src).componentEvent(_data);
    }

    /*
     * (non-Javadoc)
     * 
     * @see caat.ui.architecture.UIServer#setup(caat.configuration.ArchitectureConfiguration)
     */
    @Override
    protected void setArchitectureConfiguration(
            ArchitectureConfiguration _connectionConfiguration) {

        m_architectureMonitor = new ArchitectureMonitor(
            _connectionConfiguration, m_writer);
    }

    /*
     * (non-Javadoc)
     * 
     * @see caat.ui.architecture.UIServer#setComponentConnection(caat.corba.autogen.CAAT.ui.ComponentStatus,
     *      caat.ui.architecture.interfaces.ComponentStatusPullInterface.ComponentStatusPullConnectorOut)
     */
    @Override
    protected void setupComponentConnection(
            ComponentStatus _componentStatus,
            ComponentStatusPullConnectorOut _conn) {

        ComponentMonitor monitor = null;
        try {
            // System.out.println("setupCC: " + _logObject.m_component);
            monitor = m_architectureMonitor.getComponent(
                _componentStatus.m_component,
                _componentStatus.m_subarchitecture);
            // System.out.println("setupCC: " + monitor.getID());
        }
        catch (CASTUIException e) {
            e.printStackTrace();
            System.exit(1);
        }

        monitor.setConnection(_conn);

        m_monitors.put(monitor.getID(), monitor);
        // monitor.setInitialValues(_logObject);
    }

    /*
     * (non-Javadoc)
     * 
     * @see balt.core.processes.FrameworkProcess#run()
     */
    @Override
    public void run() {

        createWindow();

        linkMonitorsToGUI();

        while (!m_shell.isDisposed()) {
            if (!m_display.readAndDispatch()) {
                m_display.sleep();
            }
        }
        m_display.dispose();

    }

    /*
     * (non-Javadoc)
     * 
     * @see balt.core.processes.FrameworkProcess#stop()
     */
    @Override
    public void stop() {
        super.stop();
        m_mut.stopUpdating();
        try {
            m_mut.join();
        }
        catch (InterruptedException e) {
            e.printStackTrace();
            System.exit(1);
        }
    }

    /*
     * (non-Javadoc)
     * 
     * @see caat.ui.architecture.UIServer#start()
     */
    @Override
    public void start() {
        super.start();

        m_mut = new MonitorUpdaterThread();
        m_mut.start();
    }
}
