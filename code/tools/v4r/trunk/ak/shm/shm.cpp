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

#include "shm.hpp"
#include "shmheader.hpp"
#include <boost/date_time/c_local_time_adjustor.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/interprocess/containers/vector.hpp>
#include <boost/interprocess/allocators/allocator.hpp>
#include <iostream>


namespace ak {

uint32_t Shm::SEGMENT_SIZE = 0;
std::string Shm::SEGMENT_NAME = "NA";

void Shm::init(std::string name, unsigned int size) {
    SEGMENT_SIZE = size;
    SEGMENT_NAME = name;
}

unsigned int Shm::segment_size() {
    if(SEGMENT_SIZE == 0) init();
    return SEGMENT_SIZE;
}

const char* Shm::segment_name() {
    if(SEGMENT_SIZE == 0)  init();
    
    return SEGMENT_NAME.c_str();
}

const char* Shm::getErrorString(ErrorCode code) {
    switch (code) {
    case Shm::SUCCESS:
        return "Success";
    case Shm::WRONG_SIZE:
        return "The shared memory already holds a value named like this, but with a different size";
    case Shm::UNSPECIFIED:
        return "Unspecified error (Exception?)";
    case Shm::NOT_EXISTANT:
        return "The given shared memory variable name does not exist";
    case Shm::ALREADY_OPENED:
        return "This instance of ShmVarPtr is already opened";
    case Shm::INVALID_PARAM:
        return "Invalid parameter";
    }
    return "Unknown error code";
}
const char* Shm::getShmTypeString(uint8_t typeShm) {
    switch (typeShm) {
    case Shm::TYPE_NA:
        return "NA";
    case Shm::TYPE_ARRAY:
        return "Array";
    case Shm::TYPE_VARIABLE:
        return "Variable";
    case Shm::TYPE_VECTOR:
        return "Vector";
    case Shm::TYPE_LIST:
        return "List";
    case Shm::TYPE_MAP:
        return "Map";
    }
    return "Unknown";
}
const char* Shm::getVarTypeString(uint16_t typeVar) {
    switch (typeVar) {
    case Shm::VAR_NA:
        return "NA";
    case Shm::VAR_INT8:
        return "int8";
    case Shm::VAR_INT16:
        return "int16";
    case Shm::VAR_INT32:
        return "int32";
    case Shm::VAR_INT64:
        return "int64";
    case Shm::VAR_UINT8:
        return "uint8";
    case Shm::VAR_UINT16:
        return "uint16";
    case Shm::VAR_UINT32:
        return "uint32";
    case Shm::VAR_UINT64:
        return "uint64";
    case Shm::VAR_F32:
        return "float";
    case Shm::VAR_F64:
        return "double";
    case Shm::VAR_INT8C2:
        return "int8 C2";
    case Shm::VAR_INT16C2:
        return "int16 C2";
    case Shm::VAR_INT32C2:
        return "int32 C2";
    case Shm::VAR_INT64C2:
        return "int64 C2";
    case Shm::VAR_UINT8C2:
        return "uint8 C2";
    case Shm::VAR_UINT16C2:
        return "uint16 C2";
    case Shm::VAR_UINT32C2:
        return "uint32 C2";
    case Shm::VAR_UINT64C2:
        return "uint64 C2";
    case Shm::VAR_F32C2:
        return "float C2";
    case Shm::VAR_F64C2:
        return "double C2";
    case Shm::VAR_INT8C3:
        return "int8 C3";
    case Shm::VAR_INT16C3:
        return "int16 C3";
    case Shm::VAR_INT32C3:
        return "int32 C3";
    case Shm::VAR_INT64C3:
        return "int64 C3";
    case Shm::VAR_UINT8C3:
        return "uint8 C3";
    case Shm::VAR_UINT16C3:
        return "uint16 C3";
    case Shm::VAR_UINT32C3:
        return "uint32 C3";
    case Shm::VAR_UINT64C3:
        return "uint64 C3";
    case Shm::VAR_F32C3:
        return "float C3";
    case Shm::VAR_F64C3:
        return "double C3";
    case Shm::VAR_INT8C4:
        return "int8 C4";
    case Shm::VAR_INT16C4:
        return "int16 C4";
    case Shm::VAR_INT32C4:
        return "int32 C4";
    case Shm::VAR_INT64C4:
        return "int64 C4";
    case Shm::VAR_UINT8C4:
        return "uint8 C4";
    case Shm::VAR_UINT16C4:
        return "uint16 C4";
    case Shm::VAR_UINT32C4:
        return "uint32 C4";
    case Shm::VAR_UINT64C4:
        return "uint64 C4";
    case Shm::VAR_F32C4:
        return "float C4";
    case Shm::VAR_F64C4:
        return "double C4";
    case Shm::VAR_INT8C5:
        return "int8 C5";
    case Shm::VAR_INT16C5:
        return "int16 C5";
    case Shm::VAR_INT32C5:
        return "int32 C5";
    case Shm::VAR_INT64C5:
        return "int64 C5";
    case Shm::VAR_UINT8C5:
        return "uint8 C5";
    case Shm::VAR_UINT16C5:
        return "uint16 C5";
    case Shm::VAR_UINT32C5:
        return "uint32 C5";
    case Shm::VAR_UINT64C5:
        return "uint64 C5";
    case Shm::VAR_F32C5:
        return "float C5";
    case Shm::VAR_F64C5:
        return "double C5";
    case Shm::VAR_INT8C6:
        return "int8 C6";
    case Shm::VAR_INT16C6:
        return "int16 C6";
    case Shm::VAR_INT32C6:
        return "int32 C6";
    case Shm::VAR_INT64C6:
        return "int64 C6";
    case Shm::VAR_UINT8C6:
        return "uint8 C6";
    case Shm::VAR_UINT16C6:
        return "uint16 C6";
    case Shm::VAR_UINT32C6:
        return "uint32 C6";
    case Shm::VAR_UINT64C6:
        return "uint64 C6";
    case Shm::VAR_F32C6:
        return "float C6";
    case Shm::VAR_F64C6:
        return "double C6";
    case Shm::VAR_IPL_IMAGE:
        return "IplImage";
    }
    return "Unknown";
}

uint16_t Shm::getEntryType(uint16_t typeVar, unsigned int *pChannels) {
    switch (typeVar) {
    case Shm::VAR_NA:
        if (pChannels) *pChannels = 1;
        return Shm::VAR_NA;
    case Shm::VAR_INT8:
        if (pChannels) *pChannels = 1;
        return Shm::VAR_INT8;
    case Shm::VAR_INT16:
        if (pChannels) *pChannels = 1;
        return Shm::VAR_INT16;
    case Shm::VAR_INT32:
        if (pChannels) *pChannels = 1;
        return Shm::VAR_INT32;
    case Shm::VAR_INT64:
        if (pChannels) *pChannels = 1;
        return Shm::VAR_INT64;
    case Shm::VAR_UINT8:
        if (pChannels) *pChannels = 1;
        return Shm::VAR_UINT8;
    case Shm::VAR_UINT16:
        if (pChannels) *pChannels = 1;
        return Shm::VAR_UINT16;
    case Shm::VAR_UINT32:
        if (pChannels) *pChannels = 1;
        return Shm::VAR_UINT32;
    case Shm::VAR_UINT64:
        if (pChannels) *pChannels = 1;
        return Shm::VAR_UINT64;
    case Shm::VAR_F32:
        if (pChannels) *pChannels = 1;
        return Shm::VAR_F32;
    case Shm::VAR_F64:
        if (pChannels) *pChannels = 1;
        return Shm::VAR_F64;
    case Shm::VAR_INT8C2:
        if (pChannels) *pChannels = 2;
        return Shm::VAR_INT8;
    case Shm::VAR_INT16C2:
        if (pChannels) *pChannels = 2;
        return Shm::VAR_INT16;
    case Shm::VAR_INT32C2:
        if (pChannels) *pChannels = 2;
        return Shm::VAR_INT32;
    case Shm::VAR_INT64C2:
        if (pChannels) *pChannels = 2;
        return Shm::VAR_INT64;
    case Shm::VAR_UINT8C2:
        if (pChannels) *pChannels = 2;
        return Shm::VAR_UINT8;
    case Shm::VAR_UINT16C2:
        if (pChannels) *pChannels = 2;
        return Shm::VAR_UINT16;
    case Shm::VAR_UINT32C2:
        if (pChannels) *pChannels = 2;
        return Shm::VAR_UINT32;
    case Shm::VAR_UINT64C2:
        if (pChannels) *pChannels = 2;
        return Shm::VAR_UINT64;
    case Shm::VAR_F32C2:
        if (pChannels) *pChannels = 2;
        return Shm::VAR_F32;
    case Shm::VAR_F64C2:
        if (pChannels) *pChannels = 2;
        return Shm::VAR_F64;
    case Shm::VAR_INT8C3:
        if (pChannels) *pChannels = 3;
        return Shm::VAR_INT8;
    case Shm::VAR_INT16C3:
        if (pChannels) *pChannels = 3;
        return Shm::VAR_INT16;
    case Shm::VAR_INT32C3:
        if (pChannels) *pChannels = 3;
        return Shm::VAR_INT32;
    case Shm::VAR_INT64C3:
        if (pChannels) *pChannels = 3;
        return Shm::VAR_INT64;
    case Shm::VAR_UINT8C3:
        if (pChannels) *pChannels = 3;
        return Shm::VAR_UINT8;
    case Shm::VAR_UINT16C3:
        if (pChannels) *pChannels = 3;
        return Shm::VAR_UINT16;
    case Shm::VAR_UINT32C3:
        if (pChannels) *pChannels = 3;
        return Shm::VAR_UINT32;
    case Shm::VAR_UINT64C3:
        if (pChannels) *pChannels = 3;
        return Shm::VAR_UINT64;
    case Shm::VAR_F32C3:
        if (pChannels) *pChannels = 3;
        return Shm::VAR_F32;
    case Shm::VAR_F64C3:
        if (pChannels) *pChannels = 3;
        return Shm::VAR_F64;
    case Shm::VAR_INT8C4:
        if (pChannels) *pChannels = 4;
        return Shm::VAR_INT8;
    case Shm::VAR_INT16C4:
        if (pChannels) *pChannels = 4;
        return Shm::VAR_INT16;
    case Shm::VAR_INT32C4:
        if (pChannels) *pChannels = 4;
        return Shm::VAR_INT32;
    case Shm::VAR_INT64C4:
        if (pChannels) *pChannels = 4;
        return Shm::VAR_INT64;
    case Shm::VAR_UINT8C4:
        if (pChannels) *pChannels = 4;
        return Shm::VAR_UINT8;
    case Shm::VAR_UINT16C4:
        if (pChannels) *pChannels = 4;
        return Shm::VAR_UINT16;
    case Shm::VAR_UINT32C4:
        if (pChannels) *pChannels = 4;
        return Shm::VAR_UINT32;
    case Shm::VAR_UINT64C4:
        if (pChannels) *pChannels = 4;
        return Shm::VAR_UINT64;
    case Shm::VAR_F32C4:
        if (pChannels) *pChannels = 4;
        return Shm::VAR_F32;
    case Shm::VAR_F64C4:
        if (pChannels) *pChannels = 4;
        return Shm::VAR_F64;
    case Shm::VAR_INT8C5:
        if (pChannels) *pChannels = 5;
        return Shm::VAR_INT8;
    case Shm::VAR_INT16C5:
        if (pChannels) *pChannels = 5;
        return Shm::VAR_INT16;
    case Shm::VAR_INT32C5:
        if (pChannels) *pChannels = 5;
        return Shm::VAR_INT32;
    case Shm::VAR_INT64C5:
        if (pChannels) *pChannels = 5;
        return Shm::VAR_INT64;
    case Shm::VAR_UINT8C5:
        if (pChannels) *pChannels = 5;
        return Shm::VAR_UINT8;
    case Shm::VAR_UINT16C5:
        if (pChannels) *pChannels = 5;
        return Shm::VAR_UINT16;
    case Shm::VAR_UINT32C5:
        if (pChannels) *pChannels = 5;
        return Shm::VAR_UINT32;
    case Shm::VAR_UINT64C5:
        if (pChannels) *pChannels = 5;
        return Shm::VAR_UINT64;
    case Shm::VAR_F32C5:
        if (pChannels) *pChannels = 5;
        return Shm::VAR_F32;
    case Shm::VAR_F64C5:
        if (pChannels) *pChannels = 5;
        return Shm::VAR_F64;
    case Shm::VAR_INT8C6:
        if (pChannels) *pChannels = 6;
        return Shm::VAR_INT8;
    case Shm::VAR_INT16C6:
        if (pChannels) *pChannels = 6;
        return Shm::VAR_INT16;
    case Shm::VAR_INT32C6:
        if (pChannels) *pChannels = 6;
        return Shm::VAR_INT32;
    case Shm::VAR_INT64C6:
        if (pChannels) *pChannels = 6;
        return Shm::VAR_INT64;
    case Shm::VAR_UINT8C6:
        if (pChannels) *pChannels = 6;
        return Shm::VAR_UINT8;
    case Shm::VAR_UINT16C6:
        if (pChannels) *pChannels = 6;
        return Shm::VAR_UINT16;
    case Shm::VAR_UINT32C6:
        if (pChannels) *pChannels = 6;
        return Shm::VAR_UINT32;
    case Shm::VAR_UINT64C6:
        if (pChannels) *pChannels = 6;
        return Shm::VAR_UINT64;
    case Shm::VAR_F32C6:
        if (pChannels) *pChannels = 6;
        return Shm::VAR_F32;
    case Shm::VAR_F64C6:
        if (pChannels) *pChannels = 6;
        return Shm::VAR_F64;
    default:
        if (pChannels) *pChannels = 1;
        return Shm::VAR_NA;
    }
}
const std::string Shm::getPtrValueAsString(void *ptr, uint16_t typeVar, uint32_t array_idx) {
    std::stringstream ss;
    switch (typeVar) {
    case Shm::VAR_INT8:
        ss << ((int8_t*) ptr)[array_idx];
        break;
    case Shm::VAR_INT16:
        ss << ((int16_t*) ptr)[array_idx];
        break;
    case Shm::VAR_INT32:
        ss << ((int32_t*) ptr)[array_idx];
        break;
    case Shm::VAR_INT64:
        ss << ((int64_t*) ptr)[array_idx];
        break;
    case Shm::VAR_UINT8:
        ss << (int) ((uint8_t*) ptr)[array_idx];
        break;
    case Shm::VAR_UINT16:
        ss << ((uint16_t*) ptr)[array_idx];
        break;
    case Shm::VAR_UINT32:
        ss << ((uint32_t*) ptr)[array_idx];
        break;
    case Shm::VAR_UINT64:
        ss << ((uint64_t*) ptr)[array_idx];
        break;
    case Shm::VAR_F32:
        ss << ((float*) ptr)[array_idx];
        break;
    case Shm::VAR_F64:
        ss << ((double*) ptr)[array_idx];
        break;
    case Shm::VAR_INT32C2:
    {
        int *p = &((int*) ptr)[array_idx*6];
        for (int i = 0; i < 2; i++) {
            ss << std::setw(10) << p[i];
        }
    }
    break;
    case Shm::VAR_F32C2:
    {
        float *p = &((float*) ptr)[array_idx*6];
        for (int i = 0; i < 2; i++) {
            ss << std::setprecision (5) << std::fixed << std::setw(10) << p[i];
        }
    }
    break;
    case Shm::VAR_F64C2:
    {
        double *p = &((double*) ptr)[array_idx*6];
        for (int i = 0; i < 2; i++) {
            ss << std::setprecision (5) << std::fixed << std::setw(10) << p[i];
        }
    }
    break;
    case Shm::VAR_INT32C3:
    {
        int *p = &((int*) ptr)[array_idx*6];
        for (int i = 0; i < 3; i++) {
            ss << std::setw(10) << p[i];
        }
    }
    break;
    case Shm::VAR_F32C3:
    {
        float *p = &((float*) ptr)[array_idx*6];
        for (int i = 0; i < 3; i++) {
            ss << std::setprecision (5) << std::fixed << std::setw(10) << p[i];
        }
    }
    break;
    case Shm::VAR_F64C3:
    {
        double *p = &((double*) ptr)[array_idx*6];
        for (int i = 0; i < 3; i++) {
            ss << std::setprecision (5) << std::fixed << std::setw(10) << p[i];
        }
    }
    break;
    case Shm::VAR_INT32C4:
    {
        int *p = &((int*) ptr)[array_idx*6];
        for (int i = 0; i < 4; i++) {
            ss << std::setw(10) << p[i];
        }
    }
    break;
    case Shm::VAR_F32C4:
    {
        float *p = &((float*) ptr)[array_idx*6];
        for (int i = 0; i < 4; i++) {
            ss << std::setprecision (5) << std::fixed << std::setw(10) << p[i];
        }
    }
    break;
    case Shm::VAR_F64C4:
    {
        double *p = &((double*) ptr)[array_idx*6];
        for (int i = 0; i < 4; i++) {
            ss << std::setprecision (5) << std::fixed << std::setw(10) << p[i];
        }
    }
    break;
    case Shm::VAR_INT32C5:
    {
        int *p = &((int*) ptr)[array_idx*6];
        for (int i = 0; i < 5; i++) {
            ss << std::setw(10) << p[i];
        }
    }
    break;
    case Shm::VAR_F32C5:
    {
        float *p = &((float*) ptr)[array_idx*6];
        for (int i = 0; i < 5; i++) {
            ss << std::setprecision (5) << std::fixed << std::setw(10) << p[i];
        }
    }
    break;
    case Shm::VAR_F64C5:
    {
        double *p = &((double*) ptr)[array_idx*6];
        for (int i = 0; i < 5; i++) {
            ss << std::setprecision (5) << std::fixed << std::setw(10) << p[i];
        }
    }
    break;
    case Shm::VAR_INT32C6:
    {
        int *p = &((int*) ptr)[array_idx*6];
        for (int i = 0; i < 6; i++) {
            ss << std::setw(10) << p[i];
        }
    }
    break;
    case Shm::VAR_F32C6:
    {
        float *p = &((float*) ptr)[array_idx*6];
        for (int i = 0; i < 6; i++) {
            ss << std::setprecision (5) << std::fixed << std::setw(10) << p[i];
        }
    }
    break;
    case Shm::VAR_F64C6:
    {
        double *p = &((double*) ptr)[array_idx*6];
        for (int i = 0; i < 6; i++) {
            ss << std::setprecision (5) << std::fixed << std::setw(10) << p[i];
        }
    }
    break;
    default:
        ss << "na";
    }
    return ss.str();
}
const std::string Shm::getAgeString(timeval &t) {
    boost::posix_time::ptime refTime(boost::posix_time::microsec_clock::local_time());
    boost::gregorian::date d(1970, boost::gregorian::Jan, 1);
    boost::posix_time::ptime  t_utc(d, boost::posix_time::seconds(t.tv_sec) + boost::posix_time::microseconds(t.tv_usec));
    boost::posix_time::ptime t_local = boost::date_time::c_local_adjustor<boost::posix_time::ptime>::utc_to_local(t_utc);
    boost::posix_time::time_duration  td = refTime - t_local;
    return  to_simple_string(td);
}
const std::string Shm::getTimevalString(timeval &t) {
    boost::gregorian::date d(1970, boost::gregorian::Jan, 1);
    boost::posix_time::ptime  t_utc(d, boost::posix_time::seconds(t.tv_sec) + boost::posix_time::microseconds(t.tv_usec));
    boost::posix_time::ptime t_local = boost::date_time::c_local_adjustor<boost::posix_time::ptime>::utc_to_local(t_utc);
    return  to_simple_string(t_local);
}

const void* Shm::getFirst(ShmHeader<int8_t, 0> *pHeader) {
    void *ptr = NULL;
    if ( pHeader->getShmType() == Shm::TYPE_ARRAY) {
        ptr = pHeader->pData.get();
    }
    if ( pHeader->getShmType() == Shm::TYPE_VARIABLE) {
        ptr = pHeader->pData.get();
    }
    if ( pHeader->getShmType() == Shm::TYPE_VECTOR) {
        typedef boost::interprocess::allocator<char, boost::interprocess::managed_shared_memory::segment_manager> AllocatorType;
        typedef boost::interprocess::vector<char, AllocatorType> VectorType;
        ShmHeader<VectorType, 0> *pVector = (ShmHeader<VectorType, 0> *) pHeader;
        if (pVector->pData->size() > 0) {
            ptr = &pVector->pData->at(0);
        }
    }
    return ptr;
}

unsigned int Shm::getVectorOrArraySize(ShmHeader<int8_t, 0> *pHeader) {
    int size = 0;
    if ( pHeader->getShmType() == Shm::TYPE_ARRAY) {
        size = pHeader->getSize();
    }
    if ( pHeader->getShmType() == Shm::TYPE_VARIABLE) {
        size = 1;
    }
    if ( pHeader->getShmType() == Shm::TYPE_VECTOR) {
        typedef boost::interprocess::allocator<char, boost::interprocess::managed_shared_memory::segment_manager> AllocatorType;
        typedef boost::interprocess::vector<char, AllocatorType> VectorType;
        ShmHeader<VectorType, 0> *pVector = (ShmHeader<VectorType, 0> *) pHeader;
        size = pVector->pData->size();
    }
    return size;
}

const std::string Shm::getValueAsString(ShmHeader<int8_t, 0> *pHeader, int idx) {
    unsigned int columns;
    uint16_t typeEntyVar = ak::Shm::getEntryType(pHeader->getVarType(), &columns);
    unsigned int rows = getVectorOrArraySize(pHeader);
    void *ptr = (void* ) getFirst(pHeader);
    std::stringstream ss;
    for (unsigned int r = ((idx < 0)?0:idx); r < rows; ((idx < 0)?r++:r=rows)) {
        for (unsigned int c = 0; c < columns; c++) {
            switch (typeEntyVar) {
            case Shm::VAR_NA:
                ss << std::setw(10) << "na";
                break;
            case Shm::VAR_INT8:
                ss << std::setw(10) << (int) ((int8_t *)ptr)[r*columns+c];
                break;
            case Shm::VAR_INT16:
                ss << std::setw(10) << ((int16_t *)ptr)[r*columns+c];
                break;
            case Shm::VAR_INT32:
                ss << std::setw(10) << ((int32_t *)ptr)[r*columns+c];
                break;
            case Shm::VAR_INT64:
                ss << std::setw(10) << ((int64_t *)ptr)[r*columns+c];
                break;
            case Shm::VAR_UINT8:
                ss << std::setw(10) << (int) ((uint8_t *)ptr)[r*columns+c];
                break;
            case Shm::VAR_UINT16:
                ss << std::setw(10) << ((uint16_t *)ptr)[r*columns+c];
                break;
            case Shm::VAR_UINT32:
                ss << std::setw(10) << ((uint32_t *)ptr)[r*columns+c];
                break;
            case Shm::VAR_UINT64:
                ss << std::setw(10) << ((uint64_t *)ptr)[r*columns+c];
                break;
            case Shm::VAR_F32:
                ss<< std::setprecision (5) << std::fixed << std::setw(10) << ((float *)ptr)[r*columns+c];
                break;
            case Shm::VAR_F64:
                ss << std::setprecision (5) << std::fixed << std::setw(10) << ((double *)ptr)[r*columns+c];
                break;
            default:
                ss << std::setw(10) << "?";
            }
            if (columns != 1) {
                if (c != columns-1) ss << ",";
                else  ss << ";";
            }
            if (pHeader->getShmType() != ak::Shm::TYPE_VARIABLE) {
                if (c == columns-1) ss << std::endl;
            }
        }
    }
    return ss.str();
}
void Shm::setValueFromString(ShmHeader<int8_t, 0> *pHeader, const std::string &text, unsigned int idx) {
    unsigned int columns;
    uint16_t typeEntyVar = ak::Shm::getEntryType(pHeader->getVarType(), &columns);
    unsigned int rows = getVectorOrArraySize(pHeader);
    void *ptr = (void* ) getFirst(pHeader);
    std::string valueString(text);
    for (unsigned int i = 0; i < valueString.length(); i++) {
        if ((valueString.at(i) == ',') || (valueString.at(i) == ';')  || (valueString.at(i) == '[')  || (valueString.at(i) == ']')) {
            valueString.erase(i,1);
        }
    }
    unsigned int idxSym = valueString.find (":");
    if (idxSym != (unsigned int) std::string::npos) {
        std::string idxString = valueString.substr(0,idxSym);
        valueString = valueString.substr(idxSym+1);
        std::stringstream ss(idxString);
        ss >> idx;
    }
    std::stringstream ss(valueString);
    unsigned int r = idx;
    int tmp;
    for (unsigned int c = 0; (c < columns) && (r < rows); c++) {
        switch (typeEntyVar) {
        case Shm::VAR_INT8:
            ss >> tmp;
            ((int8_t *)ptr)[r*columns+c] = tmp;
            break;
        case Shm::VAR_INT16:
            ss >> ((int16_t *)ptr)[r*columns+c];
            break;
        case Shm::VAR_INT32:
            ss >> ((int32_t *)ptr)[r*columns+c];
            break;
        case Shm::VAR_INT64:
            ss >> ((int64_t *)ptr)[r*columns+c];
            break;
        case Shm::VAR_UINT8:
            ss >> tmp;
            ((uint8_t *)ptr)[r*columns+c] = tmp;
            break;
        case Shm::VAR_UINT16:
            ss >> ((uint16_t *)ptr)[r*columns+c];
            break;
        case Shm::VAR_UINT32:
            ss >> ((uint32_t *)ptr)[r*columns+c];
            break;
        case Shm::VAR_UINT64:
            ss >> ((uint64_t *)ptr)[r*columns+c];
            break;
        case Shm::VAR_F32:
            ss >> ((float *)ptr)[r*columns+c];
            break;
        case Shm::VAR_F64:
            ss >> ((double *)ptr)[r*columns+c];
            break;
        }
    }
}
bool Shm::verifyNameOrMsg(const std::string &shmName, const std::string &msg, std::ostream &os) {
    if (ak::ShmManager::getSingleton()->findName(shmName) == NULL) {
        os << msg  << shmName << std::endl;
        return false;
    } else {
        return true;
    }
}

};

