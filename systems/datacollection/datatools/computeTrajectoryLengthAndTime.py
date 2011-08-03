import getopt, sys
import os

def main():
  try:
      opts, args = getopt.getopt(sys.argv[1:], "h:f", ["help","filename="])
  except getopt.GetoptError, err:
      # print help information and exit:
      print str(err) # will print something like "option -a not recognized"
      usage()
      sys.exit(2)
  filename = None
  for o, a in opts:
      if o in ("-h", "--help"):
        usage()
        sys.exit()
      elif o in ("-f", "--filename"):
        filename = a 
      else:
        assert False, "unhandled option"

  if filename == None or not os.path.isfile(filename):
    print "need to give a filename!"
    sys.exit()
  calculateLength(filename)




# RobotPose format <x,y,theta,seconds>

def calculateLength(filename):
  f = open(filename, 'r')
  posesList = [] 
  for line in f:
#    print line
    linelist = line.rsplit()
    posesList.append((float(linelist[0]), float(linelist[1]), float(linelist[2]), float(linelist[3])))

  oldpose = (posesList[0][0], posesList[0][1])
  oldtime = float(posesList[0][3])
  totallenght = 0
  totaltime = 0
  for p in posesList:
    length_thisstep = ((oldpose[0] - p[0])**2 + (oldpose[1] - p[1])**2)**0.5
    totallenght = totallenght + length_thisstep 
#  print float(posesList[len(posesList)-1][3]) - oldtime 
  basename, extention = os.path.splitext(filename) 
  outfile = basename + "_trajlen_time.txt"
  f1 = open(outfile,'w')
  f1.write ("Total length (in meters): " + str(totallenght) + "\n")
  f1.write("Total time: " + str(totaltime) + "\n")
  print "Total length (in meters): " + str(totallenght)
  print  "Total time: " + str(totaltime)

def usage():
    print """
     USAGE:
       --place for your university,
       --file to give the file to send. 
     eg. uploaddata.py --place birmingham --filename data.tar.gz"""
if __name__ == "__main__":
    main()
