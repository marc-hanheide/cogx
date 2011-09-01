//
// Author: Bernhard Miller <bernhard.miller@austrian-kangaroos.com>, (C) 2009
//
#include <ak/shm/shmvar.hpp>
#include <ak/shm/shmarray.hpp>
#include <ak/shm/shmvector.hpp>
#include <ak/shm/shmmap.hpp>
#include <ak/shm/shmmanager.hpp>
#include <cstdio>
#include <stdlib.h>
#include <unistd.h>
#include <string>
#include <vector>
#include <boost/program_options.hpp>
#include "boost/date_time/c_local_time_adjustor.hpp"


std::string shmVarDetails(const std::string &shmName, ak::ShmHeaderNA *pHeader, const boost::posix_time::ptime &refTime) {
    std::stringstream ss;
    timeval t = pHeader->getTimeStamp();
    boost::gregorian::date d(1970, boost::gregorian::Jan, 1);
    boost::posix_time::ptime  t_utc(d, boost::posix_time::seconds(t.tv_sec) + boost::posix_time::microseconds(t.tv_usec));
    boost::posix_time::ptime t_local = boost::date_time::c_local_adjustor<boost::posix_time::ptime>::utc_to_local(t_utc);
    boost::posix_time::time_duration  td = refTime - t_local;
    if (pHeader->getShmType() == ak::Shm::TYPE_VARIABLE) {
        ss << std::setw(30 - shmName.length()) <<  ak::Shm::getValueAsString(pHeader);
    } else {
        ss << std::setw(30 - shmName.length()) <<  ak::Shm::getShmTypeString(pHeader->getShmType());
    }
    bool tryLock = pHeader->try_lock();
    ss << std::setw(5) << " locked: " << (tryLock ? "NO " : "YES");
    if (tryLock)  pHeader->unlock();
    ss << " age: " << to_simple_string(td);
    return ss.str();
}

namespace bi = boost::interprocess;
namespace po = boost::program_options;
int main(int argc, char** argv) {
    ak::Shm::init();
    // declare which options we want to read
    bool force = false;
    std::vector<std::string> nameShmList, nameShmTestLock, nameShmUnLock, nameShmLock, nameShmSignal, nameShmRead, nameShmWrite, nameShmImage;
    po::options_description desc("Allowed Parameters");
    desc.add_options()
    ("help", "get this help message")
    ("all,a", "lists all shared variables")
    ("clear,c", "clears shared memory")
    ("force,f", "Forces action write and ...")
    ("list,l", po::value<std::vector <std::string> >(&nameShmList)->multitoken(), "Lists named variables")
    ("testlock,t", po::value<std::vector <std::string> >(&nameShmTestLock)->multitoken(), "Test locks")
    ("unlock,u", po::value<std::vector <std::string> >(&nameShmUnLock)->multitoken(), "Unlocks variables")
    ("lock,k", po::value<std::vector <std::string> >(&nameShmLock)->multitoken(), "Locks variables")
    ("signal,s", po::value<std::vector <std::string> >(&nameShmSignal)->multitoken(), "Signals a of a named variables change")
    ("read,r", po::value<std::vector<std::string> >(&nameShmRead)->multitoken(), "Reads content of a named variables")
    ("write,w", po::value<std::vector<std::string> >(&nameShmWrite)->multitoken(), std::string(std::string("[value] Writes a value to shared variables") +
            std::string("\nExampe:  -w Var \"3:0 1.03  4 0 4, 4\" writes a six channels vector/array six values at index 3")).c_str());
    po::variables_map vm;
    try {
        po::store(po::parse_command_line(argc, argv, desc), vm);
    } catch (const std::exception& ex) {
        std::cout << desc << std::endl;;
        return 1;
    }
    po::notify(vm);

    if (vm.count("help"))  {
        std::cout << desc << std::endl;;
        return 1;
    }

    if (vm.count("clear"))  {
        std::cout << "Shared memory cleard" << std::endl;
        ak::ShmManager::getSingleton()->clearShm();
    }
    if (vm.count("force"))  {
        force = true;
    }

    if (vm.count("all"))  {
        std::cout << "A list of all shared variables" << std::endl;
        std::vector< std::string > shmNames;
        ak::ShmManager::getSingleton()->listNames(shmNames);
        std::sort( shmNames.begin(), shmNames.end());
        boost::posix_time::ptime refTime(boost::posix_time::microsec_clock::local_time());;
        for (unsigned int i = 0; i < shmNames.size(); i++) {
            std::cout << "  - " << shmNames[i];
            ak::ShmHeaderNA* pHeader = (ak::ShmHeaderNA*)  ak::ShmManager::getSingleton()->findName(shmNames[i]);
            std::cout << shmVarDetails(shmNames[i], pHeader, refTime)  << std::endl;
        }
    }

    if (vm.count("list"))  {
        std::cout << "List named shared variables" << std::endl;
        boost::posix_time::ptime refTime(boost::posix_time::microsec_clock::local_time());;
        for (unsigned int i = 0; i < nameShmList.size(); i++) {
            std::string shmName = nameShmList[i];
            ak::ShmHeaderNA* pHeader = (ak::ShmHeaderNA*)  ak::ShmManager::getSingleton()->findName(shmName);
            if (!pHeader) {
                std::cout << "  - " << shmName << std::setw(20) << "NO such variable" << std::endl;
            } else {
                std::cout << "  - " << shmName;
                std::cout << shmVarDetails(shmName, pHeader, refTime)  << std::endl;
                if (pHeader->getShmType() != ak::Shm::TYPE_VARIABLE) {
                    std::cout << ak::Shm::getValueAsString(pHeader)   << std::endl;;
                }
            }
        }
    }
    if (vm.count("locktest"))  {
        std::cout << "Test locks" << std::endl;
        for (unsigned int i = 0; i < nameShmTestLock.size(); i++) {
            std::string shmName = nameShmTestLock[i];
            ak::ShmHeaderNA* pHeader = (ak::ShmHeaderNA*)  ak::ShmManager::getSingleton()->findName(shmName);
            if (!pHeader) {
                std::cout << "  - " << shmName << std::setw(20) << "NO such variable" << std::endl;
            } else {
                bool tryLock = pHeader->try_lock();
                std::cout << "  - " << shmName << std::setw(20) << " locked: " << (tryLock ? "NO" : "YES") << std::endl;
                if (tryLock) {
                    pHeader->unlock();
                }
            }
        }
    }

    if (vm.count("signal"))  {
        std::cout << "Signals a change" << std::endl;
        for (unsigned int i = 0; i < nameShmSignal.size(); i++) {
            std::string shmName = nameShmSignal[i];
            ak::ShmHeaderNA* pHeader = (ak::ShmHeaderNA*)  ak::ShmManager::getSingleton()->findName(shmName);
            if (!pHeader) {
                std::cout << "  - " << shmName << std::setw(20) << "NO such variable" << std::endl;
            } else {
                ak::ShmInt32 shmVar;
                shmVar.open(shmName);
                shmVar.itHasChanged();
                std::cout << "  - " << shmName << std::setw(20) << "signaled" << std::endl;
            }
        }
    }
    if (vm.count("unlock"))  {
        std::cout << "Unlock variables" << std::endl;
        for (unsigned int i = 0; i < nameShmUnLock.size(); i++) {
            std::string shmName = nameShmUnLock[i];
            ak::ShmHeaderNA* pHeader = (ak::ShmHeaderNA*)  ak::ShmManager::getSingleton()->findName(shmName);
            if (!pHeader) {
                std::cout << "  - " << shmName << std::setw(20) << "NO such variable" << std::endl;
            } else {
                pHeader->unlock();
                std::cout << "  - " << shmName << std::setw(20) << "Unlocked" << std::endl;
            }
        }
    }

    if (vm.count("lock"))  {
        std::cout << "Lock variables" << std::endl;
        for (unsigned int i = 0; i < nameShmLock.size(); i++) {
            std::string shmName = nameShmLock[i];
            ak::ShmHeaderNA* pHeader = (ak::ShmHeaderNA*)  ak::ShmManager::getSingleton()->findName(shmName);
            if (!pHeader) {
                std::cout << "  - " << shmName << std::setw(20) << "NO such variable" << std::endl;
            } else {
                pHeader->lock();
                std::cout << "  - " << shmName << std::setw(20) << "Locked" << std::endl;
            }
        }
    }

    if (vm.count("read"))  {
        std::cout << "Reads variables" << std::endl;
        boost::posix_time::ptime refTime(boost::posix_time::microsec_clock::local_time());;
        for (unsigned int i = 0; i < nameShmRead.size(); i++) {
            std::string shmName = nameShmRead[i];
            ak::ShmHeaderNA* pHeader = (ak::ShmHeaderNA*)  ak::ShmManager::getSingleton()->findName(shmName);
            if (!pHeader) {
                std::cout << "  - " << shmName << std::setw(20) << "NO such variable" << std::endl;
            } else {
                std::cout << "  - " << shmName;
                std::cout << shmVarDetails(shmName, pHeader, refTime) << std::endl;
            }
        }
    }
    if (vm.count("write"))  {
        std::cout << "write variables" << std::endl;
        boost::posix_time::ptime refTime(boost::posix_time::microsec_clock::local_time());;
        for (unsigned int i = 0; i < nameShmWrite.size(); i++) {
            std::string shmName = nameShmWrite[i++];
            if (i >= nameShmWrite.size()) {
                std::cout << "  " << shmName << std::setw(20) << "NO value argument" << std::endl;
                return 1;
            }
            std::string shmValue = nameShmWrite[i];
            ak::ShmHeaderNA* pHeader = (ak::ShmHeaderNA*)  ak::ShmManager::getSingleton()->findName(shmName);
            if (!pHeader) {
                std::cout << "  - " << shmName << std::setw(20) << "NO such variable" << std::endl;
            } else {
                std::cout << std::setw(10) << shmName << " = " << shmValue << std::endl;
                ak::Shm::setValueFromString(pHeader, shmValue);
                if (pHeader->getShmType() != ak::Shm::TYPE_VARIABLE) {
                    std::cout << ak::Shm::getValueAsString(pHeader) << std::endl;
                } else {
                    std::cout << shmVarDetails(shmName, pHeader, refTime)  << std::endl;
                }
            }
        }
    }
    return 0;
}
// kate: indent-mode cstyle; space-indent on; indent-width 0;
