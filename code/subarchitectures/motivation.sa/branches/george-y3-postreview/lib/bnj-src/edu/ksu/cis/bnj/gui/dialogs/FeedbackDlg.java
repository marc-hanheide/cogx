/*
 * This file is part of Bayesian Network for Java (BNJ).
 * Version 3.3+
 *
 * BNJ is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * BNJ is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with BNJ in LICENSE.txt file; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * BNJ Version History
 * ---------------------------------------------
 * BN tools Jan 2000-May 2002
 *
 *  prealpha- January 200 - June 2001
 *	Benjamin Perry, Haipeng Guo, Laura Haverkamp
 *  Version 1- June 2001 - May 2002
 * 	Haipeng Guo, Benjamin Perry, Julie A. Thornton BNJ
 *
 * Bayesian Network for Java (BNJ).
 *  Version 1 - May 2002 - July 2003
 *  	release: v1.03a 29 July 2003
 * 	Infrastructure - Roby Joehanes, Haipeng Guo, Benjamin Perry, Julie A. Thornton
 *	Modules - Sonal S. Junnarkar
 *  Version 2 - August 2003 - July 2004
 *  	release: v2.03a 08 July 2004
 * 	Infrastructure - Roby Joehanes, Julie A. Thornton
 *	Modules - Siddharth Chandak, Prashanth Boddhireddy, Chris H. Meyer, Charlie L. Thornton, Bart Peinter
 *  Version 3 - August 2004 - Present
 *     	Infrastructure - Jeffrey M. Barber
 *	Modules - William H. Hsu, Andrew L. King, Chris H. Meyer, Julie A. Thornton
 * ---------------------------------------------
 */package edu.ksu.cis.bnj.gui.dialogs;
import java.io.BufferedInputStream;
import java.io.InputStream;
import java.net.URL;
import java.net.URLEncoder;
import org.eclipse.swt.widgets.Shell;
import org.eclipse.swt.SWT;
import org.eclipse.swt.widgets.Text;
import org.eclipse.swt.widgets.Label;
import org.eclipse.swt.widgets.Button;
import org.eclipse.swt.events.SelectionAdapter;
import org.eclipse.swt.events.SelectionEvent;
import org.eclipse.swt.graphics.Color;
import org.eclipse.swt.events.DisposeListener;
import org.eclipse.swt.events.DisposeEvent;
public class FeedbackDlg
{
	private Text	text;
	protected Shell	shell;
	private Color	backGround;
	private Color	foreGround;
	public void show(Shell parent)
	{
		backGround = new Color(parent.getDisplay(), 255, 255, 255);
		foreGround = new Color(parent.getDisplay(), 0, 0, 128);
		shell = new Shell(parent, SWT.PRIMARY_MODAL | SWT.BORDER | SWT.CLOSE | SWT.DIALOG_TRIM | SWT.PRIMARY_MODAL
				| SWT.TITLE);
		shell.addDisposeListener(new DisposeListener()
		{
			public void widgetDisposed(DisposeEvent e)
			{
				backGround.dispose();
				foreGround.dispose();
			}
		});
		int w = 440;
		int h = 320;
		shell.setBounds(parent.getBounds().x + parent.getBounds().width / 2 - w / 2, parent.getBounds().y
				+ parent.getBounds().height / 2 - h / 2, w, h);
		createContents();
		shell.open();
	}
	public static String getURL(String txt)
	{
		return "http://www.zenerd.com/feedback.php?feedback=" + URLEncoder.encode(txt);
	}
	public static void Send(String txt)
	{
		try
		{
			// Create an URL instance
			URL url = new URL(getURL(txt));
			// Get an input stream for reading
			InputStream in = url.openStream();
			// Create a buffered input stream for efficency
			BufferedInputStream bufIn = new BufferedInputStream(in);
			// Repeat until end of file
			for (;;)
			{
				int data = bufIn.read();
				// Check for EOF
				if (data == -1) break;
				//else
				//					System.out.print ( (char) data);
			}
		}
		catch (Exception e)
		{
			// Error
		}
	}
	public void Send()
	{
		FeedbackDlg.Send(text.getText());
	}
	protected void createContents()
	{
		shell.setText("Feedback for BNJ 3.0");
		{
			final Label label = new Label(shell, SWT.NONE);
			label.setBounds(15, 15, 245, 15);
			label.setText("Presents Bayesian Network tools in Java");
		}
		{
			final Button button = new Button(shell, SWT.NONE);
			button.addSelectionListener(new SelectionAdapter()
			{
				public void widgetSelected(SelectionEvent e)
				{
					Send();
					shell.close();
				}
			});
			button.setBounds(370, 255, 55, 25);
			button.setText("Send!");
		}
		{
			final Button button = new Button(shell, SWT.NONE);
			button.addSelectionListener(new SelectionAdapter()
			{
				public void widgetSelected(SelectionEvent e)
				{
					shell.close();
				}
			});
			button.setBounds(15, 255, 55, 25);
			button.setText("Cancel");
		}
		{
			text = new Text(shell, SWT.BORDER | SWT.V_SCROLL);
			text.setBounds(6, 40, 415, 205);
			text.setText("Enter feed back here");
		}
	}
}