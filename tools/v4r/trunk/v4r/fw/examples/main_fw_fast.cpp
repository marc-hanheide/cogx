/*
 * 1394-Based Digital Camera Control Library
 *
 * Written by Damien Douxchamps <ddouxchamps@users.sf.net>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cstdlib>
#include <cctype>
#include <opencv/highgui.h>
#include <v4r/fw/fw.h>

int main ( int argc,char *argv[] ) {
    int mode = 1; //0=YUV422, 1=MONO8/BAYER
    int fps = 1; //0=3.75Hz, 1=7.5Hz, 2=15Hz, 3=30Hz
    bool printModeSelection = false;
    V4R::FW fw;
    std::vector<uint64_t> cameraGuids = fw.readCameraGuids();
    fw.init( mode, fps, cameraGuids, printModeSelection );
    std::vector<IplImage *> imagesBy8;
		std::vector<IplImage *> imagesBGR;
    for (unsigned int i = 0; i < fw.getNrOfCameras(); i++) {
      
        // We only add image header because of the dequeue/enqueue calls
        imagesBy8.push_back(fw.createImageHeader());
        
        
				imagesBGR.push_back(cvCreateImage(cvGetSize(imagesBy8[i]), IPL_DEPTH_8U, 3));
    }

    for (int key = 0; ( ( char ) key ) != 27;) {
        fw.dequeue(imagesBy8);
        for (unsigned int i = 0; i < fw.getNrOfCameras(); i++) {
            char pWndBGR[20], pWndBy8[20];
            sprintf(pWndBy8, "bayer-%i", i);
            sprintf(pWndBGR, "bgr-%i", i);
            cvNamedWindow ( pWndBy8, 1 );
            cvShowImage ( pWndBy8, imagesBy8[i] );
						fw.bayerTo(imagesBy8[i], imagesBGR[i], 1, 514 );
            //cvCvtColor(imagesBy8[i], imagesBGR[i], CV_BayerGR2RGB);
            cvNamedWindow ( pWndBGR, 1 );
            cvShowImage ( pWndBGR, imagesBGR[i] );
        }
        fw.enqueue();
        key = cvWaitKey(2);
    }

    for (unsigned int i = 0; i < fw.getNrOfCameras(); i++) {
        char pWndBGR[20], pWndBy8[20];
        sprintf(pWndBy8, "bayer-%i", i);
        sprintf(pWndBGR, "bgr-%i", i);
        
        // it is save to realse only the header if you where working with the dequeue/enqueue
        cvReleaseImageHeader ( &imagesBy8[i] );
        
        cvReleaseImage ( &imagesBGR[i] );
        cvDestroyWindow ( pWndBy8 );
        cvDestroyWindow ( pWndBGR );
    }
}
// kate: indent-mode cstyle; space-indent on; indent-width 0; 
