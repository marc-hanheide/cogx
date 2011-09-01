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
#include <iostream>
#include <string.h>
#include <cstdlib>
#include <cctype>
#include <opencv/highgui.h>
#include <v4r/fw/fw.h>

int main ( int argc,char *argv[] ) {
    int mode = 1; //0=YUV422, 1=MONO8/BAYER
    int fps = 0; //0=3.75Hz, 1=7.5Hz, 2=15Hz, 3=30Hz
    bool printModeSelection = false;
    int counter = 0;

//      cv::VideoWriter vidWriter("videoStream", CV_FOURCC('U','2','6','3'), 15, cv::Size(1024, 768), false);
//      if( !vidWriter.isOpened() ) {
//             printf("VideoWriter failed to open!\n");
//             return -1;
//      }

    V4R::FW fw;
    std::vector<uint64_t> cameraGuids = fw.readCameraGuids();
    fw.init( mode, fps, cameraGuids, printModeSelection );
    std::vector<IplImage *> imagesBy8;
    std::vector<cv::Mat> lastImg;
//     std::vector<cv::VideoWriter> recorder;
    
    for (unsigned int i = 0; i < fw.getNrOfCameras(); i++) {
        // We only add image header because of the dequeue/enqueue calls
        imagesBy8.push_back(fw.createImageHeader());
        
//         recorder.push_back(cv::VideoWriter("videoStream.avi", CV_FOURCC('D','I','V','X'), 7.5, 1024*768, false));

    }
    for (int key = 0; ( ( char ) key ) != 27;) {
        fw.dequeue(imagesBy8);
        
        for (unsigned int i = 0; i < fw.getNrOfCameras(); i++) {
            char filename[200];
            std::vector<int> pngCompressionOptionParam;
            pngCompressionOptionParam.push_back(CV_IMWRITE_PNG_COMPRESSION);
            pngCompressionOptionParam.push_back(0);

            cv::Mat tempImg(imagesBy8[i]);

            sprintf(filename, "camera-%d-recording-%d.png", i, counter);
            cv::imwrite(filename, diffImg,pngCompressionOptionParam);
//             sprintf(filename, "camera-%d-recording-%d.bmp", i, counter);
//             cv::imwrite(filename, tempImg);

//                 vidWriter << tempImg;





        }
        std::cout << "\rrecording: " << counter;
        std::cout.flush();

        counter++;
        fw.enqueue();
        key = cvWaitKey(2);
    }

    for (unsigned int i = 0; i < fw.getNrOfCameras(); i++) {
        cvReleaseImageHeader ( &imagesBy8[i] );
    }
}
