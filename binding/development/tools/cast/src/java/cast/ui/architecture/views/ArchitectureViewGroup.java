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
package cast.ui.architecture.views;

import org.eclipse.swt.events.PaintListener;
import org.eclipse.swt.widgets.*;

/**
 * @author nah
 */
public abstract class ArchitectureViewGroup {

//    protected static class RedrawRunnable implements Runnable {
//
//        private Control m_control;
//
//        public void run() {
//            // System.out.println("RedrawRunnable.run()");
//            m_control.redraw();
//        }
//
//        public RedrawRunnable(Control _control) {
//            m_control = _control;
//        }
//
//    }

    protected Group m_group;

    protected Display m_display;

//    private RedrawRunnable m_redrawRunnable;

    public ArchitectureViewGroup(String _title, Composite _parent,
            int _style, Display _display) {
        m_group = new Group(_parent, _style);
        m_display = _display;

        m_group.setText(_title);
        m_group.pack(true);

//        m_group.addListener(SWT.Paint, new Listener() {
//
//            public void handleEvent(Event _event) {
////                System.out.println("swt.paint");
//                drawView();
//            }
//        });

        // NOTE: The following goes absolutely mental on a mac, getting
        // called all the time. The ab
        //
        // m_group.addPaintListener(new PaintListener() {
        //
        // public void paintControl(PaintEvent _e) {
        // System.out.println(_e.widget);
        // drawView(_e);
        // }
        //
        // });

//        m_redrawRunnable = new RedrawRunnable(m_group);
    }

    /**
     * @param _id
     * @param _parent
     * @param _no_redraw_resize
     * @param _display
     */
    public ArchitectureViewGroup(String _id,
            ArchitectureViewGroup _parent, int _style, Display _display) {
        this(_id, _parent.m_group, _style, _display);
    }

//    /**
//     * 
//     */
//    public void signalRedraw() {
//        m_display.asyncExec(m_redrawRunnable);
//    }

//    protected abstract void drawView();
    
    public abstract void updateView();

    public void setLayout(Layout _layout) {
        m_group.setLayout(_layout);
    }

    public void setLayoutData(Object _layoutData) {
        m_group.setLayoutData(_layoutData);
    }

    public void addPaintListener(PaintListener _listener) {
        m_group.addPaintListener(_listener);
    }

    public void pack() {
        m_group.pack();
    }

    public void pack(boolean _changed) {
        m_group.pack(_changed);
    }
//
//    /**
//     * @param group
//     *            the group to set
//     */
//    private void setGroup(Group group) {
//        m_group = group;
//    }

    /**
     * @return the group
     */
    protected Group getGroup() {
        return m_group;
    }

}