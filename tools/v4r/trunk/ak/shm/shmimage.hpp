/***************************************************************************
 *   Copyright (C) 2010 by Markus Bader and Bernhard Miller                *
 *   markus.bader@tuwien.ac.at                                             *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/


/**
 * @file shmimage
 * @author Markus Bader
 * @date Aug 4 2011
 * @version 0.1
 * @brief
 *
 * @see
 **/



#ifndef AKSHAREDAKIMAGE_HPP
#define AKSHAREDAKIMAGE_HPP

#include <ak/cv/akimage.h>
#include <ak/shm/shmvar.hpp>
#include <stdint.h>
#include <string.h>

namespace ak {


/**
* @brief Image class base on the opencv IplImage
* @author Markus Bader <markus.bader@austrian-kangaroos.com>
*/
class ShmImage : public ShmArray<char, Shm::VAR_IMAGE> {
    static const int HEADER_SIZE = 0x400;
public:
    /**
    * @brief constructor<br>
    * all data pointer will be NULL
    **/
    ShmImage() {
    }

    /**
      * @brief creates a the shared memory if it does not yet exist<br>
      * @param _name name of the shared memory
      * @param _size see OpenCV -> cvInitImageHeader
      * @param _depth see OpenCV -> cvInitImageHeader
      * @param _channels see OpenCV -> cvInitImageHeader
    **/
    Shm::ErrorCode openOrCreate ( const std::string &_name, CvSize _size, int _depth, int _channels) {
        IplImage tmp;
        cvInitImageHeader(&tmp, _size, _depth, _channels);
        unsigned int size2allocate = HEADER_SIZE + tmp.imageSize;
        Shm::ErrorCode err = ShmArray<char, Shm::VAR_IMAGE>::open ( _name, size2allocate);
        if ( err == Shm::NOT_EXISTANT) {
            err = ShmArray<char, Shm::VAR_IMAGE>::openOrCreate ( _name, size2allocate);
            if (err == Shm::SUCCESS) {
                memset(ptr(), 0, size2allocate);
                shmHeader()->clear();
            }
        }
        if (err == Shm::SUCCESS) {
            cvInitImageHeader(shmHeader()->ipl(), _size, _depth, _channels);
        }
        return err;
    }
    void set(const ak::cv::Image &_img) {
        memcpy(shmHeader(), &_img, sizeof(ak::cv::Image));
        clearPointInShmHeader();
    }
    ak::cv::Image &get(ak::cv::Image &_img) {
        memcpy(&_img, shmHeader(), sizeof(ak::cv::Image));
        _img.imageData = _img.imageDataOrigin = shmData();
        return _img;
    }
    void set(const IplImage *_img) {
        memcpy(shmHeader(), _img, sizeof(IplImage));
        clearPointInShmHeader();
    }
    IplImage *get(IplImage *_img) {
        memcpy(_img, shmHeader(), sizeof(IplImage));
        _img->imageData = _img->imageDataOrigin = shmData();
        return _img;
    }

private:
    ak::cv::Image *shmHeader() {
        return (ak::cv::Image *) ptr();
    }
    char *shmData() {
        return ptr() + HEADER_SIZE;
    }
    void clearPointInShmHeader() {
        shmHeader()->imageData = shmHeader()->imageDataOrigin = NULL;
    }

};

class ShmImageView {
public:
    ShmImageView() : quit_(true) {};
    void setName(const std::string &_name) {
        name_ = _name;
    }
    void view(int delay) {
        quit_ = false;
        do {
            int key = 0;
            show();
            key = cvWaitKey(delay);
            if ( (( char ) key ) == 27 ) {
                /// Esc pressed
                quit_ = true;
            }
        } while (quit_ == false);
        cvDestroyWindow ( name_.c_str() );
    }
    void show() {
        ak::ShmHeaderNA* pHeader = (ak::ShmHeaderNA*)  ak::ShmManager::getSingleton()->findName(name_);
        if (!pHeader) {
            std::cout << "NO such variable: " << name_ << std::endl;
        } else {
            ak::ShmImage shmImage;
            ak::Shm::ErrorCode err = shmImage.open(name_);
            if (err == Shm::SUCCESS) {
                shmImage.get(img_);
                cvShowImage ( name_.c_str(), img_.ipl());
            }
        }
    }
    void close() {
        quit_ = true;
    }
    ak::cv::Image &img(){
      return img_;
    }
private:
    bool quit_;
    std::string name_;
    ak::cv::Image img_;
};


}
#endif
// kate: indent-mode cstyle; space-indent on; indent-width 0;

