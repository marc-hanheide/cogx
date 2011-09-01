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
 * @file shmcvmat.hpp
 * @author Markus Bader
 * @date Aug 4 2011
 * @version 0.1
 * @brief
 *
 * @see
 **/



#ifndef AKSHMCVMAT_HPP
#define AKSHMCVMAT_HPP

#include <opencv/cv.h>
#include <ak/shm/shmvar.hpp>

namespace ak {


/**
* @brief Image class base on the opencv IplImage
* @author Markus Bader <markus.bader@austrian-kangaroos.com>
*/
class ShmCvMat : public ShmArray<char, Shm::VAR_IPL_IMAGE>, public CvMat {
public:

    /**
    * @brief constructor<br>
    * all data pointer will be NULL
    **/
    ShmCvMat()  {
        headerSize_ = sizeof ( CvMat );
        headerOffset_ = 0x1FF; // To be comatible with cv::ak::Image
        data_ = NULL;
    }

    /**
      * @brief creates a the shared memory if it does not yet exist<br>
      * @param rName name of the shared memory
      * @param pImgHeader the header defines the size and the data pointers are turend to the shared memory
    **/
    Shm::ErrorCode openOrCreate ( const std::string &rName, int rows, int cols, int type) {
      int typeSize  = CV_ELEM_SIZE(type);
        unsigned int size2allocate = headerSize_ + rows * cols * typeSize;
        Shm::ErrorCode err = ShmArray<char, Shm::VAR_IPL_IMAGE>::openOrCreate ( rName, size2allocate );
        data_ = ptr() + headerOffset_;
        cvInitMatHeader (cvMat(), rows, cols, type, data_);
        memcpy(ptr(), cvMat(), headerSize_);
        while ( tryLock() == false ) {
            unlock();
        };
        return err;
    }

    /**
    * @brief opens a the shared memory image<br>
    * @ToDo return message
    * @param rName name of the shared memory
    **/
    Shm::ErrorCode open ( const std::string &rName) {
        Shm::ErrorCode err = ShmArray<char, Shm::VAR_IPL_IMAGE>::open ( rName );
        data_ = ptr() + headerOffset_;
        CvMat *pShm = ( CvMat * ) ptr();
        cvInitMatHeader (cvMat(), pShm->rows, pShm->cols, pShm->type, data_);
        return err;
    }
    void updateLocal() {
        CvMat *pShm = ( CvMat * ) ptr();
        cvInitMatHeader (cvMat(), pShm->rows, pShm->cols, pShm->type, data_);
    }
    void updateShm() {
        memcpy(ptr(), cvMat(), headerSize_);
    }
    CvMat *cvMat() {
        return this;
    }
    cv::Mat cv(){
      return cv::cvarrToMat(cvMat());
    }
private:
    unsigned int headerSize_;
    unsigned int headerOffset_;
    char *data_;
};


}
#endif
// kate: indent-mode cstyle; space-indent on; indent-width 0;
