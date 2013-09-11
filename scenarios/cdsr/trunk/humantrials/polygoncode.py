# -*- coding: utf-8 -*-
from Polygon import *
from Polygon.Utils import pointList
from itertools import izip
import os
import sys
import re
import csv
from scipy import stats

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

def rowToLikertTuple(row):
      return [pointToTuple(row[6]),int(row[7])]
      
def printPolyStats(t,n,polys):
      print "Stimulus("+str(t)+','+str(n)+') average area = ' + str(averageArea(polys))
      return True

def extentValues(polys):
      xmins = []
      xmaxs = []
      ymins = []
      ymaxs = []
      for poly in polys:
            bb = poly.boundingBox()
            xmins.append(bb[0])
            xmaxs.append(bb[1])
            ymins.append(bb[2])
            ymaxs.append(bb[3])
      return xmins , xmaxs , ymins , ymaxs

class PolygonAreaTest():

      # Computes the t-test between the areas of different types
      # trials is a list of the form [type1,num1],[type2,num2].  The first trial should be smaller
      def outputAreaTTests(self):
            trials = [[tuple([1,2]),tuple([1,1])],
                      [tuple([2,2]),tuple([2,4])],
                      [tuple([2,3]),tuple([2,5])],
                      [tuple([3,2]),tuple([3,1])],
                      [tuple([3,3]),tuple([3,2])],
                      [tuple([3,3]),tuple([3,4])]]
            for trial in trials:
                  type1, num1 = trial[0]
                  type2, num2 = trial[1]
                  self.outputAreaDifferences(type1,num1,type2,num2)
                  self.outputExtentDifferences(type1,num1,type2,num2)

      def outputExtentDifferences(self,type1,num1,type2,num2):
            polys1 = self.getPolygonsForTrial(type1,num1)
            polys2 = self.getPolygonsForTrial(type2,num2)
            xmin1,xmax1,ymin1,ymax1 = extentValues(polys1)
            xmin2,xmax2,ymin2,ymax2 = extentValues(polys2)
            print "Comparing extents of (" + str(type1) + ',' + str(num1) + ') to (' + str(type2) + ',' + str(num2) + ')'
            print "x-min: " + str(sum(xmin1)/len(xmin1)) + ' to ' + str(sum(xmin2)/len(xmin2)) + ' p-value = ' + str(stats.ttest_ind(xmin1,xmin2)[1])
            print "x-max: " + str(sum(xmax1)/len(xmax1)) + ' to ' + str(sum(xmax2)/len(xmax2)) + ' p-value = ' + str(stats.ttest_ind(xmax1,xmax2)[1])
            print "y-min: " + str(sum(ymin1)/len(ymin1)) + ' to ' + str(sum(ymin2)/len(ymin2)) + ' p-value = ' + str(stats.ttest_ind(ymin1,ymin2)[1])
            print "y-max: " + str(sum(ymax1)/len(ymax1)) + ' to ' + str(sum(ymax2)/len(ymax2)) + ' p-value = ' + str(stats.ttest_ind(ymax1,ymax2)[1])
                

      # computes a t-test between the areas as determined by stimuli type1 num1 and type2 num2
      def outputAreaDifferences(self, type1, num1, type2, num2):
            polys1 = self.getPolygonsForTrial(type1,num1)
            polys2 = self.getPolygonsForTrial(type2,num2)
            printPolyStats(type1,num1,polys1)
            printPolyStats(type2,num2,polys2)
            tscore,pvalue = stats.ttest_ind(map(lambda x:x.area(),polys1),
                                            map(lambda x:x.area(),polys2))
            print 'p value = ' + str(pvalue)

      def outputNearFarDifferences(self):
            polys1 = self.getPolygonsForTrial(2,2)
            polys2 = self.getPolygonsForTrial(2,4)
            polys1.extend(self.getPolygonsForTrial(2,3))
            polys2.extend(self.getPolygonsForTrial(2,5))
            print "Polys1 average area = " + str(averageArea(polys1))
            print "Polys2 average area = " + str(averageArea(polys2))
            tscore,pvalue = stats.ttest_ind(map(lambda x:x.area(),polys1),
                                            map(lambda x:x.area(),polys2))
            print 'p value = ' + str(pvalue)
            


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
            return map(rowToLikertTuple,
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
      Test.outputAreaTTests()
      Test.outputNearFarDifferences()

