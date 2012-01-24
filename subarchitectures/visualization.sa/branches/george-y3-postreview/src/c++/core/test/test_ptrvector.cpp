/*
 * © Copyright 2010 Marko Mahnič. 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include "../ptrvector.hpp"

#include <vector>
#include <sstream>
#include <cppunit/extensions/HelperMacros.h>

static int deleted = 0;
class CPtrVectorTest: public CppUnit::TestFixture {
private:
   class CInt {
   public:
      int valid;
      int num;
      CInt(int i) { num = i; valid = 937; };
      virtual ~CInt() { 
         valid = 0;
         deleted++;
      };
      virtual bool isvalid() { return valid == 937; }
   };
   std::vector<CInt*> testdata;

   void copyTestData(CPtrVector<CInt> &vect) {
      vect.clear();
      std::vector<CInt*>::iterator it;
      for(it = testdata.begin(); it != testdata.end(); it++) {
         vect.push_back(*it);
      }
   }
public:
   void setUp() {
      testdata.clear();
      for (int i=1; i <= 20; i++) testdata.push_back(new CInt(i));
   }
   void tearDown() {
      // just clear the data, don't delete - the array elements may be invalid!
      // we allow memory to leak.
      testdata.clear();
   }

   void testCopy() {
      CPtrVector<CInt> vect;
      copyTestData(vect);
      CPPUNIT_ASSERT( vect.size() == testdata.size() );
   }

   void testForEach() {
      CPtrVector<CInt> vect;
      copyTestData(vect);
      CInt *pint;
      FOR_EACH(pint, vect) {
         pint->num = 99;
      }

      std::vector<CInt*>::iterator it;
      for(it = testdata.begin(); it != testdata.end(); it++) {
         CPPUNIT_ASSERT( (*it)->num == 99 );
      }
   }

   void testForEachNULL() {
      CPtrVector<CInt> vect;
      copyTestData(vect);
      CInt *pint;
      FOR_EACH(pint, vect) {
         // nothing
      }

      CPPUNIT_ASSERT( pint == NULL );
   }

   void testForEachIterator() {
      CPtrVector<CInt> vect;
      copyTestData(vect);
      CInt *pint;
      FOR_EACH(pint, vect) {
         (*__itpint)->num = 99;
      }

      std::vector<CInt*>::iterator it;
      for(it = testdata.begin(); it != testdata.end(); it++) {
         CPPUNIT_ASSERT( (*it)->num == 99 );
      }
   }

   void testForEach_MapValue() {
      std::map<std::string, CInt*> map;
      map.clear();
      std::vector<CInt*>::iterator it;
      int i = 0;
      for(it = testdata.begin(); it != testdata.end(); it++) {
         std::ostringstream si;
         si.str("");
         si << i;
         map[si.str()]=(*it);
         i++;
      }
      CInt *pint;
      FOR_EACH_V(pint, map) {
         pint->num = 99;
      }

      for(it = testdata.begin(); it != testdata.end(); it++) {
         CPPUNIT_ASSERT( (*it)->num == 99 );
      }
   }

   void testForEach_MapKey() {
      std::map<std::string, CInt*> map;
      map.clear();
      std::vector<CInt*>::iterator it;
      int i = 0;
      for(it = testdata.begin(); it != testdata.end(); it++) {
         std::ostringstream si;
         si.str("");
         si << i;
         map[si.str()]=(*it);
         i++;
      }
      std::string *pstr;
      FOR_EACH_K(pstr, map) {
         map[*pstr]->num = 99;
         // std::cout << *pstr << " ";
      }

      for(it = testdata.begin(); it != testdata.end(); it++) {
         CPPUNIT_ASSERT( (*it)->num == 99 );
      }
   }

   void _do_testPack(bool single, int packmode) {
      char msg[128];
      for (int i = 0; i < 3; i++) {
         CPtrVector<CInt> vect;
         copyTestData(vect);
         CInt *pint;
         int count = vect.size();
         FOR_EACH(pint, vect) {
            if (single) { // isolated nulls
               if (pint->num % 3 == i) {
                  *__itpint = NULL;
                  count -= 1;
               }
            }
            else { // runs of nulls
               if ((pint->num / 3) % 3 == i) {
                  *__itpint = NULL;
                  count -= 1;
               }
            }
         }
         vect.pack(packmode);
         sprintf(msg, "r:%s, m:%s, i:%d, t.size:%ld --> count:%d --> v.size:%ld",
               single ? "isolated": "pack",
               packmode==CPtrVector<CInt>::packquick ? "pquick" : 
               packmode == CPtrVector<CInt>::packsorted ? "psorted" : "none",
               i, testdata.size(), count, vect.size());
         if (packmode == CPtrVector<CInt>::packnone)
            CPPUNIT_ASSERT_MESSAGE( msg, vect.size() == testdata.size() );
         else
            CPPUNIT_ASSERT_MESSAGE(msg, vect.size() == count );

         FOR_EACH(pint, vect) {
            if (! pint) {
               CPPUNIT_ASSERT_MESSAGE( msg, packmode == CPtrVector<CInt>::packnone);
               continue;
            }
            if (single) CPPUNIT_ASSERT_MESSAGE( msg, ! (pint->num % 3 == i) );
            else CPPUNIT_ASSERT_MESSAGE (msg, ! ((pint->num / 3) % 3 == i) );
         }
      }
   }

   void testPack() {
      _do_testPack(true, CPtrVector<CInt>::packnone);
      _do_testPack(true, CPtrVector<CInt>::packquick);
      _do_testPack(true, CPtrVector<CInt>::packsorted);
      _do_testPack(false, CPtrVector<CInt>::packnone);
      _do_testPack(false, CPtrVector<CInt>::packquick);
      _do_testPack(false, CPtrVector<CInt>::packsorted);
   }

   void _do_testRemove(bool single, int packmode, bool groupremove) {
      char msg[128];
      for (int i = 0; i < 3; i++) {
         CPtrVector<CInt> vect, toremove;
         copyTestData(vect);
         CInt *pint;
         int count = vect.size();
         FOR_EACH(pint, vect) {
            if (single) { // isolated nulls
               if (pint->num % 3 == i) {
                  toremove.push_back(pint);
                  count -= 1;
               }
            }
            else { // runs of nulls
               if ((pint->num / 3) % 3 == i) {
                  toremove.push_back(pint);
                  count -= 1;
               }
            }
         }
         deleted = 0;
         if (groupremove) {
           vect.remove(toremove, packmode);
         }
         else {
           FOR_EACH(pint, toremove) {
              CPPUNIT_ASSERT_MESSAGE(msg, ! (pint == NULL));
              vect.remove(pint, packmode);
           }
         }
         CPPUNIT_ASSERT_MESSAGE(msg, deleted == 0 );

         sprintf(msg, "r:%s-%s, m:%s, i:%d, t.size:%ld --> count:%d --> v.size:%ld",
               groupremove ? "group" : "individual",
               single ? "isolated": "pack",
               packmode==CPtrVector<CInt>::packquick ? "pquick" : 
               packmode == CPtrVector<CInt>::packsorted ? "psorted" : "none",
               i, testdata.size(), count, vect.size());
         if (packmode == CPtrVector<CInt>::packnone)
            CPPUNIT_ASSERT_MESSAGE( msg, vect.size() == testdata.size() );
         else
            CPPUNIT_ASSERT_MESSAGE(msg, vect.size() == count );

         FOR_EACH(pint, vect) {
            if (! pint) {
               CPPUNIT_ASSERT_MESSAGE( msg, packmode == CPtrVector<CInt>::packnone);
               continue;
            }
            if (single) CPPUNIT_ASSERT_MESSAGE( msg, ! (pint->num % 3 == i) );
            else CPPUNIT_ASSERT_MESSAGE (msg, ! ((pint->num / 3) % 3 == i) );
         }
      }
   }

   void testRemoveIsolated() {
      _do_testRemove(true, CPtrVector<CInt>::packnone, false);
      _do_testRemove(true, CPtrVector<CInt>::packquick, false);
      _do_testRemove(true, CPtrVector<CInt>::packsorted, false);
   }

   void testRemovePacked() {
      _do_testRemove(false, CPtrVector<CInt>::packnone, false);
      _do_testRemove(false, CPtrVector<CInt>::packquick, false);
      _do_testRemove(false, CPtrVector<CInt>::packsorted, false);
   }

   void testGroupRemoveIsolated() {
      _do_testRemove(true, CPtrVector<CInt>::packnone, true);
      _do_testRemove(true, CPtrVector<CInt>::packquick, true);
      _do_testRemove(true, CPtrVector<CInt>::packsorted, true);
   }

   void testGroupRemovePacked() {
      _do_testRemove(false, CPtrVector<CInt>::packnone, true);
      _do_testRemove(false, CPtrVector<CInt>::packquick, true);
      _do_testRemove(false, CPtrVector<CInt>::packsorted, true);
   }

   void testDelete() {
      for (int i = 0; i < 3; i++) {
         tearDown();
         setUp();
         CPtrVector<CInt> vect, toremove;
         copyTestData(vect);
         CInt *pint;
         int count = vect.size();
         FOR_EACH(pint, vect) {
            if (pint->num % 3 == i) {
               toremove.push_back(pint);
               count -= 1;
            }
         }
         int packmode = CPtrVector<CInt>::packquick;
         deleted = 0;
         vect.delete_items(toremove, packmode);
         CPPUNIT_ASSERT(vect.size() == count );
         CPPUNIT_ASSERT(deleted == toremove.size() );

         FOR_EACH(pint, vect) {
            if (! pint) {
               CPPUNIT_ASSERT(packmode == CPtrVector<CInt>::packnone);
               continue;
            }
            CPPUNIT_ASSERT(! (pint->num % 3 == i) );
         }
      }
   }

   void testDeleteItem() {
      for (int i = 0; i < 3; i++) {
         tearDown();
         setUp();
         CPtrVector<CInt> vect, toremove;
         copyTestData(vect);
         CInt *pint;
         int count = vect.size();
         FOR_EACH(pint, vect) {
            if (pint->num % 3 == i) {
               toremove.push_back(pint);
               count -= 1;
            }
         }
         int packmode = CPtrVector<CInt>::packnone;
         deleted = 0;
         FOR_EACH(pint, toremove) {
            vect.delete_item(pint, packmode);
         }
         vect.pack(CPtrVector<CInt>::packquick);

         CPPUNIT_ASSERT(vect.size() == count );
         CPPUNIT_ASSERT(deleted == toremove.size() );

         FOR_EACH(pint, vect) {
            if (! pint) {
               CPPUNIT_ASSERT(packmode == CPtrVector<CInt>::packnone);
               continue;
            }
            CPPUNIT_ASSERT(! (pint->num % 3 == i) );
         }
      }
   }

public:
   CPPUNIT_TEST_SUITE( CPtrVectorTest );

   CPPUNIT_TEST( testCopy );
   CPPUNIT_TEST( testForEach );
   CPPUNIT_TEST( testForEachNULL );
   CPPUNIT_TEST( testForEachIterator );
   CPPUNIT_TEST( testForEach_MapValue );
   CPPUNIT_TEST( testForEach_MapKey );
   CPPUNIT_TEST( testPack );

   CPPUNIT_TEST( testRemoveIsolated );
   CPPUNIT_TEST( testRemovePacked );
   CPPUNIT_TEST( testGroupRemoveIsolated );
   CPPUNIT_TEST( testGroupRemovePacked );

   CPPUNIT_TEST( testDelete );
   CPPUNIT_TEST( testDeleteItem );

   CPPUNIT_TEST_SUITE_END();
   //static CppUnit::Test* createSuite()
   //{
   //   CppUnit::TestSuite suite;
   //   suite.addTest(new CppUnit::TestCaller<CPtrVectorTest>("testForEach", &CPtrVectorTest::testForEach));
   //   return suite;
   //}
};
CPPUNIT_TEST_SUITE_REGISTRATION( CPtrVectorTest );


#include <cppunit/ui/text/TestRunner.h>
#include <cppunit/extensions/TestFactoryRegistry.h>
int main( int argc, char **argv)
{
   CppUnit::TestFactoryRegistry &registry = CppUnit::TestFactoryRegistry::getRegistry();
   CppUnit::TextUi::TestRunner runner;
   runner.addTest( registry.makeTest() );

   // runner.addTest( CPtrVectorTest::suite() );
   // runner.addTest( ComplexNumberTest::suite() );

   bool wasSuccessful = runner.run( "", false );
   return !wasSuccessful;
}

