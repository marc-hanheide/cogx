/*
 * Comedian example code to demonstrate CAST functionality.
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
package comedyarch;

import cast.AlreadyExistsOnWMException;
import cast.architecture.UnmanagedComponent;
import cast.core.CASTUtils;

import comedyarch.autogen.TwoLiner;

/**
 * @author nah
 */
public class StraightMan extends UnmanagedComponent {

	private String generateSetup() {
		return "What's brown and sticky?";
	}

	@Override
	public void runComponent() {
		while (isRunning()) {

			try {
				// must check we're still running after sleep!
				if (isRunning()) {

					// lock from external access
					lockComponent();

					println("ahem...");
					println(CASTUtils.toString(getCASTTime()));

					// make up a joke
					TwoLiner jk = new TwoLiner("", generateSetup());

					// and then make it available in the s-a working
					// memory
					addToWorkingMemory(newDataID(), jk);
					// addToWorkingMemory(newDataID(), "audience.sa", jk);
					// let other stuff happen if necessary
					unlockComponent();

					sleepComponent(1000);
				}
			} catch (AlreadyExistsOnWMException e) {
				e.printStackTrace();
			}
			// catch (UnknownSubarchitectureException e) {
			// e.printStackTrace();
			// }

		}

	}

}
