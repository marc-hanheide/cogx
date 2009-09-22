#! /usr/bin/env python
# -*- coding: latin-1 -*-

import unittest
import tempfile
import os

import parser, mapltypes
from mapltypes import *
from parser import Parser, ParseError

typetest = \
"""
(:types  truck airplane - vehicle
         package vehicle - thing
         airport - location
         city location thing - object
)

"""

objecttest = \
"""
(:objects  apn1 - airplane
           tru1 tru2 - truck
           obj11 obj12 obj13 obj21 obj22 obj23 - package
           apt1 apt2 - airport
           pos1 pos2 - location
           cit1 cit2 - city
)
"""

class ConditionsTest(unittest.TestCase):
    def parseTypes(self, it):
        it.get(":types")
        typeDict = {objectType.name : objectType, booleanType.name : booleanType}
        
        tlist = mapltypes.parse_typelist(it)
        for key, value in tlist.iteritems():
            if value.string not in typeDict:
                typeDict[value.string] = Type(value.string, [objectType])
            if key.string not in typeDict:
                typeDict[key.string] = Type(key.string, [objectType])

            typeDict[key.string].supertypes.add(typeDict[value.string])

        return typeDict

    def parseObjects(self, it, typeDict):
        it.get(":objects")
        objects = set()
        
        olist = mapltypes.parse_typelist(it)
        for key, value in olist.iteritems():
            if value.string not in typeDict:
                raise ParseError(value, "undeclared type")

            objects.add(TypedObject(key.string, typeDict[value.string]))
        return objects
    
    def testTypeParsing(self):
        """Testing type parsing and type checking"""
        
        p = Parser(typetest.split("\n"))
        types = self.parseTypes(iter(p.root))

        expectedTypes = ["truck", "airplane", "vehicle", "package", "vehicle", "thing", "airport", "location", "object", "boolean"]
        for t in expectedTypes:
            self.assert_(t in types)
            self.assert_(isinstance(types[t], Type))
            self.assert_(types[t].equalOrSubtypeOf(types["object"]))
            self.assert_(types[t].equalOrSubtypeOf(types[t]))
            self.assert_(types[t].equalOrSupertypeOf(types[t]))
            self.assertFalse(types[t].isSubtypeOf(types[t]))
            self.assertFalse(types[t].isSupertypeOf(types[t]))
            self.assertEqual(types[t], types[t])

        self.assert_(types["truck"].isSubtypeOf(types["vehicle"]))
        self.assert_(types["truck"].isSubtypeOf(types["thing"]))
        self.assert_(types["location"].isSupertypeOf(types["airport"]))

    def testObjectParsing(self):
        """Testing object parsing and type checking"""
        
        p = Parser(typetest.split("\n"))
        types = self.parseTypes(iter(p.root))
        
        p = Parser(objecttest.split("\n"))
        objects = dict([(o.name, o) for o in self.parseObjects(iter(p.root), types)])

        expectedObjects = ["apn1", "tru1", "tru2", "obj11", "obj12", "obj13", "obj21", "obj22", "obj23",
                         "apt1", "apt2", "pos1", "pos2", "cit1", "cit2"]
        for o in expectedObjects:
            self.assert_(o in objects)
            self.assert_(isinstance(objects[o], TypedObject))
            self.assert_(objects[o].isInstanceOf(types["object"]))
            self.assertEqual(objects[o], objects[o])

        self.assert_(objects["tru1"].isInstanceOf(types["truck"]))
        self.assert_(objects["tru1"].isInstanceOf(types["vehicle"]))
        self.assert_(objects["tru1"].isInstanceOf(types["thing"]))

        self.assert_(objects["apn1"].isInstanceOf(types["airplane"]))
        self.assert_(objects["apn1"].isInstanceOf(types["vehicle"]))
        self.assert_(objects["apn1"].isInstanceOf(types["thing"]))

        self.assert_(objects["obj13"].isInstanceOf(types["package"]))
        self.assert_(objects["obj13"].isInstanceOf(types["thing"]))

        self.assert_(objects["apt1"].isInstanceOf(types["airport"]))
        self.assert_(objects["apt2"].isInstanceOf(types["location"]))

        self.assert_(objects["pos1"].isInstanceOf(types["location"]))
        self.assert_(objects["pos2"].isInstanceOf(types["location"]))

        self.assert_(objects["cit1"].isInstanceOf(types["city"]))
        self.assert_(objects["cit2"].isInstanceOf(types["city"]))

    def testObjects(self):
        """Testing object typing/type checking"""
        
        p = Parser(typetest.split("\n"))
        types = self.parseTypes(iter(p.root))

        o1 = TypedObject("o1", types["thing"])
        o2 = TypedObject("o1", types["thing"])
        o3 = TypedObject("o1", types["vehicle"])
        
        self.assertEqual(o1, o2)
        self.assertNotEqual(o1, o3)

        self.assert_(o1.isInstanceOf(types["thing"]))
        self.assertFalse(o1.isInstanceOf(types["truck"]))
        self.assertFalse(o1.isInstanceOf(types["vehicle"]))
        self.assertFalse(o1.isInstanceOf(types["location"]))

    def testCompositeTypes(self):            
        """Testing composite types"""
        
        p = Parser(typetest.split("\n"))
        types = self.parseTypes(iter(p.root))
        
        ctype = CompositeType([types["package"], types["truck"]])

        self.assert_(types["package"].isSubtypeOf(ctype))
        self.assertFalse(types["package"].isSupertypeOf(ctype))
        self.assert_(types["truck"].isSubtypeOf(ctype))
        self.assertFalse(types["truck"].isSupertypeOf(ctype))

        self.assert_(ctype.equalOrSupertypeOf(ctype))
        self.assert_(ctype.equalOrSubtypeOf(ctype))
        self.assertFalse(ctype.isSupertypeOf(ctype))
        self.assertFalse(ctype.isSubtypeOf(ctype))
        
        self.assert_(types["thing"].isSupertypeOf(ctype))
        self.assert_(types["object"].isSupertypeOf(ctype))
        self.assertFalse(types["vehicle"].isSupertypeOf(ctype))

        o1 = TypedObject("o1", types["package"])
        o2 = TypedObject("o2", types["truck"])
        o3 = TypedObject("o3", types["airplane"])
        o4 = TypedObject("o3", ctype)
        
        self.assert_(o1.isInstanceOf(ctype))
        self.assert_(o2.isInstanceOf(ctype))
        self.assertFalse(o3.isInstanceOf(ctype))
        self.assert_(o4.isInstanceOf(ctype))
        self.assertFalse(o4.isInstanceOf(types["package"]))
        self.assertFalse(o4.isInstanceOf(types["truck"]))
        self.assertFalse(o4.isInstanceOf(types["vehicle"]))
        self.assert_(o4.isInstanceOf(types["thing"]))
        self.assert_(o4.isInstanceOf(types["object"]))

        ctype2 = CompositeType([types["package"], types["vehicle"]])
        self.assert_(ctype.isSubtypeOf(ctype2))
        self.assert_(ctype2.isSupertypeOf(ctype))

        ctype3 = CompositeType([types["truck"], types["airplane"]])
        self.assert_(ctype3.equalOrSubtypeOf(types["vehicle"]))
        self.assert_(ctype3.isSubtypeOf(types["vehicle"]))
        self.assertFalse(ctype3.isSupertypeOf(types["vehicle"]))
        
    def testFunctionTypes(self):            
        """Testing function types"""
        
        p = Parser(typetest.split("\n"))
        types = self.parseTypes(iter(p.root))

        fpackage = FunctionType(types["package"])
        
        self.assert_(fpackage.equalOrSubtypeOf(types["package"]))
        self.assert_(fpackage.isSubtypeOf(types["package"]))
        self.assertFalse(fpackage.isSupertypeOf(types["package"]))

    def testBulitinTypes(self):
        """Testing builtin types"""

        self.assertFalse(numberType.equalOrSubtypeOf(objectType))
        self.assertFalse(numberType.isSubtypeOf(objectType))
        self.assertFalse(numberType.equalOrSupertypeOf(objectType))
        self.assertFalse(numberType.isSupertypeOf(objectType))
        
        
if __name__ == '__main__':
    unittest.main()    
        
