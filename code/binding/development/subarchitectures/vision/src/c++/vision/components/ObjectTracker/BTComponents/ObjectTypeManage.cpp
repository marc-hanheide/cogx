#include "ObjectTypeManage.hpp"
#include "vision/components/common/SystemUtils/Common.h"

using namespace std; 
using namespace Common;

map<Vision::ObjType, string> ObjectTypeManage::m_mapObjectTypes;

ObjectTypeManage::ObjectTypeManage()
{
    
}

ObjectTypeManage::~ObjectTypeManage()
{
}


void ObjectTypeManage::Initialize()
{
    m_mapObjectTypes[Vision::UNDEF] = Vision::TYPE_UNDEF;
    m_mapObjectTypes[Vision::HAND] = Vision::TYPE_HAND;
    m_mapObjectTypes[Vision::SHAPE] = Vision::TYPE_SHAPE;
    m_mapObjectTypes[Vision::FLAG] = Vision::TYPE_FLAG;
    m_mapObjectTypes[Vision::CAR] = Vision::TYPE_CAR;
    m_mapObjectTypes[Vision::BOX] = Vision::TYPE_BOX;
    m_mapObjectTypes[Vision::BALL] = Vision::TYPE_BALL;    
}

Vision::ObjType ObjectTypeManage::StringToObjtype(std::string _str)
{
    map<Vision::ObjType, string>::iterator it;
    for (it=m_mapObjectTypes.begin(); it!=m_mapObjectTypes.end(); it++) {
	if (it->second == _str)
	    return it->first;
    }
    throw user_error(__HERE__, "ObjType %s does not match a string in Vision.idl.", _str.c_str());
}

string ObjectTypeManage::ObjtypeToString(Vision::ObjType _type)
{
    map<Vision::ObjType, string>::iterator it = m_mapObjectTypes.find(_type);
    if (it != m_mapObjectTypes.end())	
	return it->second;
    else
	throw user_error(__HERE__, "ObjType error");
}
