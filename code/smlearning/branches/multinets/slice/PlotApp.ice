/** @file PlotApp.ice
 * 
 * 
 * @author	Sergio Roa (DFKI)
 *
 * @version 1.0
 *
 *           2009      Sergio Roa
 
   This is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This package is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License.
   If not, see <http://www.gnu.org/licenses/>.

 
 */

#ifndef PLOTAPP_ICE
#define PLOTAPP_ICE

module smlearning {

module plotting {

sequence<double> SeqDouble;

interface MainWindow {
	void start ();
	void init (int size, SeqDouble lpData, SeqDouble eData);
	void resize (int width, int height);
	void show ();
};

interface PlottingApp {

	void init (int regionsNr, int size);
	void updateData (int region, SeqDouble lpData, SeqDouble eData);
	void resize (int width, int height);
	void show ();

};

};

};


#endif
