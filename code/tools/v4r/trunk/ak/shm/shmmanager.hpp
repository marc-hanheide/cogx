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


#ifndef AKSHAREDMEMMANAGER_HPP
#define AKSHAREDMEMMANAGER_HPP

#include <vector>
#include <string>

#include <boost/interprocess/managed_shared_memory.hpp>


namespace ak {

/**
    @author Bernhard Miller <bernhard.miller@austrian-kangaroos.com>
*/
class ShmManager {
public:
    /** \brief Returns the pointer to the created singleton sharedmemmanager */
    static ShmManager* getSingleton();

    /** \brief Destructor
    **/
    ~ShmManager();

    /** \brief Recreates the shared memory */
    void clearShm();

    /** \brief Removes the shared memory */
    void removeShm();

    /** \brief Makes sure the shared memory exists, i.e. creates, if it does not exist yet */
    void createShm();

    /** \brief get a pointer to the shared mem manager */
    boost::interprocess::managed_shared_memory* getShm();

    /** \brief Returns list with the names of the shared variables
    * @param rNames vector which will be filled with the varibles
    * @param pTypes type of the variable
    * @param regExpressions examples "(.*)bmp",  "(.*)$"
    * @author Markus Bader */
    void listNames(std::vector< std::string> &rNames, std::vector<int> *pTypes = NULL, std::string regExpressions = "" );

    /** \brief Returns list with the names of the shared variables
    * @param rName search name
    * @return pointer ot the shared variable or NULL if it was not found
    * @author Markus Bader
    */
    const void *findName(const std::string &rName, std::string prefix = "");

    /** \brief deletes a shared variable
    * @param rName search name
    * @return zero on success -> variable found and deleted
    * @author Markus Bader
    */
    int delName(const std::string &rName);

    /** \brief Returns the underlying boost shared memory manager
     * @return segment manager
     * @author Markus Bader
     * @ToDo: implementieren
    */
    boost::interprocess::managed_shared_memory* getShmManager() {
        return pShm;
    };
		

private:
    ShmManager();
    ShmManager( const ShmManager& ) {};

    static ShmManager* pSingleton;
    boost::interprocess::managed_shared_memory* pShm;
};

}


#endif
