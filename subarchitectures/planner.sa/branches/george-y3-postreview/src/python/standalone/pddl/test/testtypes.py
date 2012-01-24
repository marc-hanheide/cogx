#! /usr/bin/env python
# -*- coding: latin-1 -*-

import unittest
import tempfile
import os

import parser, mapltypes
from mapltypes import *
from parser import Parser, ParseError
from builtin import TRUE, FALSE, t_object, t_boolean

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
        typeDict = {t_object.name : t_object, t_boolean.name : t_boolean}
        
        tlist = mapltypes.parse_typelist(it)
        for key, value in tlist.iteritems():
            if value.string not in typeDict:
                typeDict[value.string] = Type(value.string, [t_object])
            if key.string not in typeDict:
                typeDict[key.string] = Type(key.string, [t_object])

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
            self.assert_(types[t].equal_or_subtype_of(types["object"]))
            self.assert_(types[t].equal_or_subtype_of(types[t]))
            self.assert_(types[t].equal_or_supertype_of(types[t]))
            self.assertFalse(types[t].is_subtype_of(types[t]))
            self.assertFalse(types[t].is_supertype_of(types[t]))
            self.assertEqual(types[t], types[t])

        self.assert_(types["truck"].is_subtype_of(types["vehicle"]))
        self.assert_(types["truck"].is_subtype_of(types["thing"]))
        self.assert_(types["location"].is_supertype_of(types["airport"]))

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
            self.assert_(objects[o].is_instance_of(types["object"]))
            self.assertEqual(objects[o], objects[o])

        self.assert_(objects["tru1"].is_instance_of(types["truck"]))
        self.assert_(objects["tru1"].is_instance_of(types["vehicle"]))
        self.assert_(objects["tru1"].is_instance_of(types["thing"]))

        self.assert_(objects["apn1"].is_instance_of(types["airplane"]))
        self.assert_(objects["apn1"].is_instance_of(types["vehicle"]))
        self.assert_(objects["apn1"].is_instance_of(types["thing"]))

        self.assert_(objects["obj13"].is_instance_of(types["package"]))
        self.assert_(objects["obj13"].is_instance_of(types["thing"]))

        self.assert_(objects["apt1"].is_instance_of(types["airport"]))
        self.assert_(objects["apt2"].is_instance_of(types["location"]))

        self.assert_(objects["pos1"].is_instance_of(types["location"]))
        self.assert_(objects["pos2"].is_instance_of(types["location"]))

        self.assert_(objects["cit1"].is_instance_of(types["city"]))
        self.assert_(objects["cit2"].is_instance_of(types["city"]))

    def testObjects(self):
        """Testing object typing/type checking"""
        
        p = Parser(typetest.split("\n"))
        types = self.parseTypes(iter(p.root))

        o1 = TypedObject("o1", types["thing"])
        o2 = TypedObject("o1", types["thing"])
        o3 = TypedObject("o1", types["vehicle"])
        
        self.assertEqual(o1, o2)
        self.assertNotEqual(o1, o3)

        self.assert_(o1.is_instance_of(types["thing"]))
        self.assertFalse(o1.is_instance_of(types["truck"]))
        self.assertFalse(o1.is_instance_of(types["vehicle"]))
        self.assertFalse(o1.is_instance_of(types["location"]))

    def testCompositeTypes(self):            
        """Testing composite types"""
        
        p = Parser(typetest.split("\n"))
        types = self.parseTypes(iter(p.root))
        
        ctype = CompositeType([types["package"], types["truck"]])

        self.assert_(types["package"].is_subtype_of(ctype))
        self.assertFalse(types["package"].is_supertype_of(ctype))
        self.assert_(types["truck"].is_subtype_of(ctype))
        self.assertFalse(types["truck"].is_supertype_of(ctype))

        self.assert_(ctype.equal_or_supertype_of(ctype))
        self.assert_(ctype.equal_or_subtype_of(ctype))
        self.assertFalse(ctype.is_supertype_of(ctype))
        self.assertFalse(ctype.is_subtype_of(ctype))
        
        self.assert_(types["thing"].is_supertype_of(ctype))
        self.assert_(types["object"].is_supertype_of(ctype))
        self.assertFalse(types["vehicle"].is_supertype_of(ctype))

        o1 = TypedObject("o1", types["package"])
        o2 = TypedObject("o2", types["truck"])
        o3 = TypedObject("o3", types["airplane"])
        o4 = TypedObject("o3", ctype)
        
        self.assert_(o1.is_instance_of(ctype))
        self.assert_(o2.is_instance_of(ctype))
        self.assertFalse(o3.is_instance_of(ctype))
        self.assert_(o4.is_instance_of(ctype))
        self.assertFalse(o4.is_instance_of(types["package"]))
        self.assertFalse(o4.is_instance_of(types["truck"]))
        self.assertFalse(o4.is_instance_of(types["vehicle"]))
        self.assert_(o4.is_instance_of(types["thing"]))
        self.assert_(o4.is_instance_of(types["object"]))

        ctype2 = CompositeType([types["package"], types["vehicle"]])
        self.assert_(ctype.is_subtype_of(ctype2))
        self.assert_(ctype2.is_supertype_of(ctype))

        ctype3 = CompositeType([types["truck"], types["airplane"]])
        self.assert_(ctype3.equal_or_subtype_of(types["vehicle"]))
        self.assert_(ctype3.is_subtype_of(types["vehicle"]))
        self.assertFalse(ctype3.is_supertype_of(types["vehicle"]))
        
    def testFunctionTypes(self):            
        """Testing function types"""
        
        p = Parser(typetest.split("\n"))
        types = self.parseTypes(iter(p.root))

        fpackage = FunctionType(types["package"])
        
        self.assert_(fpackage.equal_or_subtype_of(types["package"]))
        self.assert_(fpackage.is_subtype_of(types["package"]))
        self.assertFalse(fpackage.is_supertype_of(types["package"]))

    def testBulitinTypes(self):
        """Testing builtin types"""

        self.assertFalse(t_number.equal_or_subtype_of(t_object))
        self.assertFalse(t_number.is_subtype_of(t_object))
        self.assertFalse(t_number.equal_or_supertype_of(t_object))
        self.assertFalse(t_number.is_supertype_of(t_object))
        
        
if __name__ == '__main__':
    unittest.main()    
        
