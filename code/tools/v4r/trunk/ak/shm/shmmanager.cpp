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

#include "shmmanager.hpp"

#include <stdio.h>
#include <string.h>

#include <ak/shm/shmarray.hpp>
#include <ak/shm/shmvar.hpp>
#include <ak/shm/shmvector.hpp>
#include <boost/regex.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>

namespace ak {

ShmManager* ShmManager::pSingleton = NULL;

ShmManager* ShmManager::getSingleton() {
    if ( !pSingleton ) {
        pSingleton = new ShmManager();
    }
    return pSingleton;
}


ShmManager::ShmManager() : pShm(NULL) {
    createShm();
}


ShmManager::~ShmManager() {
}

void ShmManager::removeShm() {
    boost::interprocess::shared_memory_object::remove(Shm::segment_name());
    if (pShm) {
        delete pShm;
        pShm = NULL;
    }
}

void ShmManager::clearShm() {
    removeShm();
    createShm();
}

boost::interprocess::managed_shared_memory* ShmManager::getShm() {
    if (!pShm) {
        createShm();
    }

    return pShm;
}

void ShmManager::createShm() {
    pShm = new boost::interprocess::managed_shared_memory(boost::interprocess::open_or_create, Shm::segment_name(), Shm::segment_size());
}

void ShmManager::listNames(std::vector< std::string> &rNames, std::vector<int> *pTypes, std::string regx) {
    try {
        typedef boost::interprocess::managed_shared_memory::const_named_iterator const_named_it;
        const_named_it named_beg = pShm->named_begin();
        const_named_it named_end = pShm->named_end();
        for (; named_beg != named_end; ++named_beg) {
            const boost::interprocess::managed_shared_memory::char_type *name = named_beg->name();
            ShmHeaderNA * pHeader = (ShmHeaderNA*)  named_beg->value();
            if (regx.empty()) {
                rNames.push_back(name);
                if (pTypes)  pTypes->push_back(pHeader->getVarType());
            } else {
                /*
                static const boost::regex expression ( regx.c_str() );
                if (boost::regex_match ( name, expression ) ) {
                rNames.push_back(name);
                if (pTypes)  pTypes->push_back(pHeader->getVarType());
                }
                */
                std::string entryName(name);
                if (entryName.find(regx) != entryName.npos)  {
                    rNames.push_back(name);
                    if (pTypes)  pTypes->push_back(pHeader->getVarType());
                }
            }
        }
    }    catch (...) {
        //AK_LOG_ERROR << "exception ShmManager::listNames()";
        std::cerr << "exception ShmManager::listNames()";
    }
}

const void *ShmManager::findName(const std::string &rNames, std::string prefix) {
    const void *value = NULL;
    std::string full_name = "";

    if (prefix.compare("") != 0)
        full_name = prefix + ":" + rNames;
    else
        full_name = rNames;

    try {
        typedef boost::interprocess::managed_shared_memory::const_named_iterator const_named_it;
        const_named_it named_beg = pShm->named_begin();
        const_named_it named_end = pShm->named_end();
        for (; named_beg != named_end; ++named_beg) {
            const boost::interprocess::managed_shared_memory::char_type *name = named_beg->name();
            if (full_name.compare(name) == 0) {
                value = named_beg->value();
            }
        }
    }    catch (...) {
        //AK_LOG_ERROR << "exception ShmManager::findName(): " << rNames;
        std::cerr << "exception ShmManager::findName(): " << rNames;
    }
    return value;
}

int ShmManager::delName(const std::string &rName) {
    return 0;
}


}



