# -*- coding: utf-8 -*-
from Polygon import *
from Polygon.Utils import pointList
from itertools import izip
import os
import sys
import re
import csv


def rowToPolygon(row):
      result = row[7]
      result = re.sub('\)', '', result)
      result = re.sub('\(', '', result)
      tmp_array = map(lambda pt:tuple(map(int,pt)),map(lambda x:x.split(':'),result[:-1].split(';')))
      return Polygon(tuple(tmp_array))

#returns the average area of the polygons in poly
def averageArea(polys):
      return reduce(lambda x,y: x+y,[x.area() for x in polys]) / len(polys)

#pt is a str
def pointToTuple(pt):
      result = re.sub('\(', '', pt)
      result = re.sub('\)', '', result)
      return tuple(map(int,result.split(':')))

class PolygonAreaTest():

      #trials is a list of tuples of the form stimtype,stimnum
      def __init__(self,file):
            self.trials = [tuple([1,i+1]) for i in range(2)]
            self.trials.extend([tuple([2,i+1]) for i in range(7)])
            self.trials.extend([tuple([3,i+1]) for i in range(4)])
            self.getDataFromFile(file)

      def getDataFromFile(self,file):
            self.csv_results = []
            with open(file) as csvfile:
                  reader = csv.reader(csvfile)
                  for row in reader:
                        self.csv_results.append(row)
            print 'rows: ' +  str(len(self.csv_results))

      def getPolygonsForTrial(self,stimtype,stimnum):
            return map(rowToPolygon ,
                       filter(lambda row: row[2]=="1" and int(row[4])==stimtype and int(row[5])==stimnum and row[7]!='-5',
                              self.csv_results))
      
      def outputAreas(self):
            for stimtype,stimnum in self.trials:
                  polys = self.getPolygonsForTrial(stimtype,stimnum)
                  avg = averageArea(polys)
                  print "stimtype: ", stimtype , " stimnum : ", stimnum , " Average Polygon Area: " , avg , " Number of Polygons : " , len(polys)

      # returns a list of pairs where the first is the point tuple and the second is rating
      def getLikertValuesForTrial(self,stimtype,stimnum):
            return map(lambda row:[pointToTuple(row[6]),int(row[7])],
                       filter(lambda row: row[2]=="3" and int(row[4])==stimtype and int(row[5])==stimnum,self.csv_results))

      # Not doing any check for not applicable.  So we will need to check that out later.
      def outputLikertScores(self):
            for stimtype,stimnum in self.trials:
                  vals = self.getLikertValuesForTrial(stimtype,stimnum)
                  pts = set(map(lambda pair:pair[0],vals))
                  for pt in pts:
                        scores = filter(lambda val: val[0] == pt, vals)
                        scores = map(lambda pair : pair[1] , scores) 
                        print "stimtype: ", stimtype , " stimnum: ", stimnum , " point: " ,
                        print pt , " average score: ", reduce(lambda a,b:a+b ,scores)/float(len(scores)) ,
                        print " Number of pts : " , len(scores)

                              
# pass in the results file you want to analyze.  
if __name__ == '__main__':
      Test = PolygonAreaTest(sys.argv[1])
      Test.outputAreas()
      Test.outputLikertScores()
