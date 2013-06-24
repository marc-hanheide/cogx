# -*- coding: utf-8 -*-
from Polygon import *
from Polygon.Utils import pointList
from itertools import izip
import os
import sys
import re

class PolygonAreaTest():

      def __init__(self, d):
            self.datadir = d 
            self.test_polygons=dict()

      def getData(self):
            for f in os.listdir(self.datadir):
                  #add a key value pair to the dictionary for each file
                  k = f.rstrip('.csv')
                  self.test_polygons[k]=[]
                  #open the file and read it line by line into an array
                  try:
                        fh = open(self.datadir + '/' + f, 'rU')
                        lines = [line.strip() for line in fh]
                        fh.close()
                  except IOError as e:
                        print "I/O Error({0}): {1}".format(e.errno,e.strerror)
                  #convert from a string of bracketted pairs into
                  #an array of int tuples and create polygon objects
                  #for each array
                  for i in range(0, len(lines)):
                        lines[i] = re.sub('\)', '', lines[i])
                        lines[i] = re.sub('\(', '', lines[i])
                        lines[i] = lines[i].split(',')
                        lines[i] = [int(j) for j in lines[i]]
                        tmp_array = []
                        for j in range(0,len(lines[i]),2):
                              tmp_array.append(tuple([lines[i][j],lines[i][j+1]]))
                        tmp_tuple = tuple(tmp_array)
                        self.test_polygons[k].append(Polygon(tmp_tuple))

      def outputAreas(self):
            for k in self.test_polygons.keys():
                  count = 0
                  total = 0
                  plist = self.test_polygons[k]
                  for p in plist:
                        total += p.area()
                        count +=1
                  print "Condition: ", k , " Average Polygon Area: ", str(total/count)


if __name__ == '__main__':
      Test = PolygonAreaTest('./polygondata')
      Test.getData()
      Test.outputAreas()
