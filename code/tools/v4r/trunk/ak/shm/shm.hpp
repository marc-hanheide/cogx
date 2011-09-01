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


#ifndef SHARED_MEM_HPP
#define SHARED_MEM_HPP

#include <iostream>
#include <stdint.h>

#include <sys/time.h>

namespace ak {

template<typename, uint16_t> class ShmHeader;
template<typename, uint16_t> class ShmVar;
template<typename, uint16_t> class ShmArray;
template<typename, uint16_t> class ShmVector;
template<typename, uint16_t> class ShmList;
template<typename, typename, uint16_t> class ShmMap;
/**
 * Class to hold needed constants
 * @author Markus Bader
 **/
class Shm {
    static uint32_t SEGMENT_SIZE;
    static std::string SEGMENT_NAME;
public:
    
    static void init(std::string name = "AKShm", unsigned int size = 0x6000000);
    
    static unsigned int segment_size();
    
    static const char* segment_name();
        
    /// Type of the shared memory
    enum Type {
        TYPE_CUSTOMIZED = 1000,
        TYPE_NA = 0,
        TYPE_ARRAY = 1,
        TYPE_VARIABLE = 2,
        TYPE_VECTOR = 3,
        TYPE_LIST = 4,
        TYPE_MAP = 5
    };
    /// Variable types
    enum Var {
        VAR_CHANNEL_OFFSET = 15,
        VAR_NA = 0,
        VAR_INT8 = 1,
        VAR_INT16 = 2,
        VAR_INT32 = 3,
        VAR_INT64 = 4,
        VAR_UINT8 = 5,
        VAR_UINT16 = 6,
        VAR_UINT32 = 7,
        VAR_UINT64 = 8,
        VAR_F32 = 9,
        VAR_F64 = 10,
        VAR_C2 = VAR_CHANNEL_OFFSET*2,
        VAR_INT8C2 = VAR_C2+1,
        VAR_INT16C2 = VAR_C2+2,
        VAR_INT32C2 = VAR_C2+3,
        VAR_INT64C2 = VAR_C2+4,
        VAR_UINT8C2 = VAR_C2+5,
        VAR_UINT16C2 = VAR_C2+6,
        VAR_UINT32C2 = VAR_C2+7,
        VAR_UINT64C2 = VAR_C2+8,
        VAR_F32C2 = VAR_C2+9,
        VAR_F64C2 = VAR_C2+10,
        VAR_C3 = VAR_CHANNEL_OFFSET*3,
        VAR_INT8C3 = VAR_C3+1,
        VAR_INT16C3 = VAR_C3+2,
        VAR_INT32C3 = VAR_C3+3,
        VAR_INT64C3 = VAR_C3+4,
        VAR_UINT8C3 = VAR_C3+5,
        VAR_UINT16C3 = VAR_C3+6,
        VAR_UINT32C3 = VAR_C3+7,
        VAR_UINT64C3 = VAR_C3+8,
        VAR_F32C3 = VAR_C3+9,
        VAR_F64C3 = VAR_C3+10,
        VAR_C4 = VAR_CHANNEL_OFFSET*4,
        VAR_INT8C4 = VAR_C4+1,
        VAR_INT16C4 = VAR_C4+2,
        VAR_INT32C4 = VAR_C4+3,
        VAR_INT64C4 = VAR_C4+4,
        VAR_UINT8C4 = VAR_C4+5,
        VAR_UINT16C4 = VAR_C4+6,
        VAR_UINT32C4 = VAR_C4+7,
        VAR_UINT64C4 = VAR_C4+8,
        VAR_F32C4 = VAR_C4+9,
        VAR_F64C4 = VAR_C4+10,
        VAR_C5 = VAR_CHANNEL_OFFSET*5,
        VAR_INT8C5 = VAR_C5+1,
        VAR_INT16C5 = VAR_C5+2,
        VAR_INT32C5 = VAR_C5+3,
        VAR_INT64C5 = VAR_C5+4,
        VAR_UINT8C5 = VAR_C5+5,
        VAR_UINT16C5 = VAR_C5+6,
        VAR_UINT32C5 = VAR_C5+7,
        VAR_UINT64C5 = VAR_C5+8,
        VAR_F32C5 = VAR_C5+9,
        VAR_F64C5 = VAR_C5+10,
        VAR_C6 = VAR_CHANNEL_OFFSET*6,
        VAR_INT8C6 = VAR_C6+1,
        VAR_INT16C6 = VAR_C6+2,
        VAR_INT32C6 = VAR_C6+3,
        VAR_INT64C6 = VAR_C6+4,
        VAR_UINT8C6 = VAR_C6+5,
        VAR_UINT16C6 = VAR_C6+6,
        VAR_UINT32C6 = VAR_C6+7,
        VAR_UINT64C6 = VAR_C6+8,
        VAR_F32C6 = VAR_C6+9,
        VAR_F64C6 = VAR_C6+10,
        VAR_OPENCV = 2000,
        VAR_IMAGE = VAR_OPENCV + 50,
        VAR_AK_IMAGE = VAR_OPENCV + 51,
        VAR_IPL_IMAGE = VAR_OPENCV + 52
    };
    /// Error codes
    enum ErrorCode {
        SUCCESS = 0,
        WRONG_SIZE = -1,
        UNSPECIFIED = -2,
        NOT_EXISTANT = -3,
        ALREADY_OPENED = -4,
        INVALID_PARAM = -5
    };
    enum Signals {
        SIGNAL_ALWAYS = 0,
        SIGNAL_CHANGED = 1,
        SIGNAL_NEVER = 2
    };
    static uint16_t getEntryType(uint16_t typeVar, unsigned int *pChannels = NULL);
    static const char* getErrorString(ErrorCode code);
    static const char* getShmTypeString(uint8_t typeShm);
    static const char* getVarTypeString(uint16_t typeVar);
    static const std::string getAgeString(timeval &r);
    static const std::string getTimevalString(timeval &t);
    static const std::string getPtrValueAsString(void *ptr, uint16_t typeVar, uint32_t array_idx = 0);

    /**
     * verifies a shared memory name or prints a ms
     * @param shmName
     * @param msg to print if the name does not exist
     * @param os stream to print on
     * @return true if the name exists
     **/
    static bool verifyNameOrMsg(const std::string &shmName, const std::string &msg, std::ostream &os);
    static const void* getFirst(ShmHeader<int8_t, 0> *pHeader);
    static unsigned int getVectorOrArraySize(ShmHeader<int8_t, 0> *pHeader);
    static const std::string getValueAsString(ShmHeader<int8_t, 0> *pHeader, int idx = -1);
    static void setValueFromString(ShmHeader<int8_t, 0> *pHeader, const std::string &text, unsigned int idx = 0);

};

class ShmIndexException : public std::exception {
public:
    virtual const char* what() const throw() {
        return "Bad index for shared memory array";
    }
};

};


#endif /* SHARED_MEM_HPP */
