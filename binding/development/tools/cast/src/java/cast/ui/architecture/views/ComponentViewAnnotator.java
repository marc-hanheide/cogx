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

import org.eclipse.swt.widgets.Display;
import org.eclipse.swt.widgets.Label;

import cast.ui.architecture.ComponentMonitor;

/**
 * @author nah
 */
public abstract class ComponentViewAnnotator {

    protected ComponentMonitor m_monitor;
    protected Display m_display;

    protected static class UpdateRunnable implements Runnable {

        private Label m_control;
        private String m_text;
        private boolean m_drawn;

        public void run() {
            // System.out.println("RedrawRunnable.run()");
            if (!m_drawn) {
                m_control.setText(m_text);
                m_drawn = true;
            }
        }

        public UpdateRunnable(Label _control) {
            m_control = _control;
            m_text = "";
            // start with the assumption we need new
            // text
            m_drawn = true;
        }

        public void setText(String _text) {
            // only redraw text if it's different then the old text
            if (!_text.equals(m_text)) {
                m_text = _text;
                m_drawn = false;
            }
        }
    }

    public ComponentViewAnnotator(ComponentMonitor _monitor,
            Display _display) {
        m_monitor = _monitor;
        m_display = _display;
    }

    protected abstract void update();

}
