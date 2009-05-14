#ifndef  _OBJECT_TYPE_MANAGE_HPP_
#define  _OBJECT_TYPE_MANAGE_HPP_

#include <map>
#include <string>
#include <vision/idl/Vision.hh>

class ObjectTypeManage
{
protected:
    static std::map<Vision::ObjType, std::string> m_mapObjectTypes;
    
public:
    ObjectTypeManage();
    virtual ~ObjectTypeManage();

    // Initialize (hard-coded) should be called only once.     
    void Initialize();

    Vision::ObjType StringToObjtype(std::string _str);
    std::string ObjtypeToString(Vision::ObjType _type);
    
};

#endif
