# -*- coding: utf-8 -*-
from Polygon import *
from Polygon.IO import *
from Polygon.Utils import pointList
from itertools import izip
import os
import sys
import re

class KappaScoreAnalysis():

      def __init__(self, track, tid, granularity, width, height, filepath):
            self.test_points_granularity = granularity
            self.stimulus_width = width
            self.stimulus_height = height
            self.resultsfilepath = filepath
            self.testtrack = track
            self.testid = tid
            self.test_points = self.generateTestPoints()# vector of datapoints
            self.subject_ids = []
            self.subject_polygons = dict()
            #key = user id, value = list of 1:test_point_in_poly 0:test_point_out_poly
            self.subject_test_point_classification = dict()
            self.subject_test_point_yes = dict()
            self.between_subject_kappa = dict()
            
            
      def generateTestPoints(self):
            test_point_list = []
            for i in range(10,500,self.test_points_granularity):
                 for j in range(10,375,10):
                       test_point_list.append((i,j))
            return test_point_list

      #get the user ids and create the polygons for the trial ids we are interested in
      def getData(self):
            #open the file and read it line by line into an array
            try:
                  fh = open(self.resultsfilepath, 'rU')
                  lines = [line.strip() for line in fh]
                  fh.close()
            except IOError as e:
                  print "I/O Error({0}): {1}".format(e.errno,e.strerror)

            for i in range(0, len(lines)):
                  lines[i] = lines[i].split(',')
                  #check if it is the type of trial we are interested in
                  #i.e., polygon (not likert or sweetspot)
                  #by checking the testtrack value
                  if lines[i][2] == self.testtrack:
                        #check if it is the stimulus type we are interested in
                        #by checking the testid
                        if lines[i][3] == self.testid:
                              #get the user data
                              if lines[i][0] in self.subject_ids:
                                    print "Error: user is has duplicate entry for testid"
                              else:
                                    self.subject_ids.append(lines[i][0])
                                    self.subject_polygons[lines[i][0]] = self.convertResultsStringToPolygon(lines[i][7])
                  


      #Example input string: (345:211);(276:237);(267:204);(345:201);
      def convertResultsStringToPolygon(self, res_string):
            #if the res_string ends in a semi-colon remove it
            if res_string.endswith(";"):
                  res_string = res_string[:-1]
            #split the string up into points using semicolons
            res_string = res_string.split(";")

            vertex_array=[]
            for i in range(0,len(res_string)):
                  #remove the opening and closing brackets from the points
                  res_string[i] = re.sub('\(', '', res_string[i])
                  res_string[i] = re.sub('\)', '', res_string[i])
                  # split the point values using the : delimiter
                  vertex_coords = res_string[i].split(":")
                  vertex_array.append(tuple((int(vertex_coords[0]),int(vertex_coords[1]))))
            return Polygon(tuple(vertex_array))

      def writePolygonsToSVG(self):
            plist = []
            for i in range(0,len(self.subject_ids)):
                  plist.append(self.subject_polygons[self.subject_ids[i]])
            writeSVG("simulated_polygons_data_" + str(self.testtrack) + "_" + str(self.testid) + ".svg",plist,width=500,height=375)

      def classifyTestPoints(self):
            num_test_points = len(self.test_points)
            for i in range(0,len(self.subject_ids)):
                  curr_id = self.subject_ids[i]
                  curr_p = self.subject_polygons[curr_id]
                  curr_yes = 0 #count how many test points inside region - used in cohen kappa
                  classification_list = [0] * num_test_points
                  for j in range(0,num_test_points):
                        if curr_p.isInside(self.test_points[j][0],self.test_points[j][1]):
                              classification_list[j] = 1
                              curr_yes += 1
                  self.subject_test_point_classification[curr_id] = classification_list
                  self.subject_test_point_yes[curr_id] = curr_yes
                  

      #for each user calculate their cohen kappa score with all the other users for this trial type
      def calculateBetweenSubjectKappa(self):
            #calculate Cohen Kappa see: en.wikipedia.org/wiki/Cohen's_kappa
            for i in range(0,len(self.subject_ids)):
                  curr_id = self.subject_ids[i]
                  curr_classification = self.subject_test_point_classification[curr_id]
                  curr_cohens = []
                  curr_subj_dict = dict()
                  for j in range(0, len(self.subject_ids)):
                        if j != i:
                              comparision_id = self.subject_ids[j]
                              comparision_classification = self.subject_test_point_classification[comparision_id]
                              agree = 0 #count how often they both agreed (both 0 or both 1)
                              curr_yes = 0
                              comp_yes = 0
                              for k in range(0,len(self.test_points)):
                                    if curr_classification[k] == comparision_classification[k]:
                                          agree += 1
                              #observed percentage agreement
                              #print " agree " + str(float(agree))
                              #print " denom " + str(float(len(self.test_points)))
                              Pa = float(agree)/float(len(self.test_points))
                              #print "Pa " + str(Pa)
                              #prob of random agreement
                              #random yes
                              PCurrYes = (float(self.subject_test_point_yes[curr_id])/float(len(self.test_points)))
                              PCompYes = (float(self.subject_test_point_yes[comparision_id])/float(len(self.test_points)))
                              PYes = PCurrYes * PCompYes
                              PNo = (1-PCurrYes) * (1-PCompYes)
                              Pe = PYes + PNo
                              #print str(curr_id) + " " + str(Pa) + " " + str(Pe)
                              coh_kappa = (Pa-Pe)/(1-Pe)
                              curr_subj_dict[comparision_id] = coh_kappa
                        else:
                              curr_subj_dict[curr_id] = 1.0 #every subject will have a kappa of 1 with themselves
                  #compute the average cohens kapps for the current user
                  self.between_subject_kappa[curr_id] = curr_subj_dict

      def outputBetweenSubjectKappa(self):
            COLUMN_WIDTH = 14
            STRING_WHEN_MISSING = '""'
            PADDINGSTRING = "."
            columns = ["subject ids"]
            columns = columns + [subject for (subject,kappa_dict) in sorted(self.between_subject_kappa.items())]
            print "".join([col.ljust(COLUMN_WIDTH) for col in columns])
            for subj_id in sorted(self.between_subject_kappa.keys()):
                  row = []
                  row.append(subj_id)
                  for (subject,kappa_score) in sorted(self.between_subject_kappa[subj_id].items()):
                        row.append(str(round(kappa_score,4)))
                  print "".join([cell_val.ljust(COLUMN_WIDTH) for cell_val in row])
                       

##      def outputAreas(self):
##            for k in self.test_polygons.keys():
##                  count = 0
##                  total = 0
##                  plist = self.test_polygons[k]
##                  for p in plist:
##                        total += p.area()
##                        count +=1
##                  print "Condition: ", k , " Average Polygon Area: ", str(total/count)



if __name__ == '__main__':
      Test = KappaScoreAnalysis("2","2_5",5,500,375,'./simulated_polygon_data.csv')
      Test.getData()
      print "got data!"
      Test.writePolygonsToSVG()
      print "svg created"
      Test.classifyTestPoints()
      print "test points classified"
      Test.calculateBetweenSubjectKappa()
      Test.outputBetweenSubjectKappa()
      print "finished"
