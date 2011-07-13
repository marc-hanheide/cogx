import getopt, sys
import ftplib
import os

class progressBar:
	def __init__(self, minValue = 0, maxValue = 10, totalWidth=12):
		self.progBar = "[]"   # This holds the progress bar string
		self.min = minValue
		self.max = maxValue
		self.span = maxValue - minValue
		self.width = totalWidth
		self.amount = 0       # When amount == max, we are 100% done 
		self.updateAmount(0)  # Build progress bar string

	def updateAmount(self, newAmount = 0):
		if newAmount < self.min: newAmount = self.min
		if newAmount > self.max: newAmount = self.max
		self.amount = newAmount

		# Figure out the new percent done, round to an integer
		diffFromMin = float(self.amount - self.min)
		percentDone = (diffFromMin / float(self.span)) * 100.0
		percentDone = round(percentDone)
		percentDone = int(percentDone)
		
    # Figure out how many hash bars the percentage should be
		allFull = self.width - 2
		numHashes = (percentDone / 100.0) * allFull
		numHashes = int(round(numHashes))

		# build a progress bar with hashes and spaces
		self.progBar = "[" + '-'*numHashes + ' '*(allFull-numHashes) + "]"

		# figure out where to put the percentage, roughly centered
		percentPlace = (len(self.progBar) / 2) - len(str(percentDone)) 
		percentString = str(percentDone) + "%"

		# slice the percentage into the bar
		self.progBar = self.progBar[0:percentPlace] + percentString + self.progBar[percentPlace+len(percentString):]

	def __str__(self):
		return str(self.progBar)
def main():
  try:
      opts, args = getopt.getopt(sys.argv[1:], "h:f:p", ["help","forcedir", "filename=", "place="])
  except getopt.GetoptError, err:
      # print help information and exit:
      print str(err) # will print something like "option -a not recognized"
      usage()
      sys.exit(2)
  output = None
  forceDirUpload = False
  filename = None
  dirname = None
  for o, a in opts:
      if o in ("-h", "--help"):
        usage()
        sys.exit()
      elif o in ("-p", "--place"):
        dirname = a
      elif o in ("-f", "--filename"):
        filename = a 
      elif o in ("--forcedir"):
        forceDirUpload = True 
      else:
        assert False, "unhandled option"
  if (dirname == None):
    print "Specify your university with -p, e.g. -p vienna"
    sys.exit()  
  
  sftp = ftplib.FTP('130.237.218.125','alperaydemir','alper') # Connect
  sftp.cwd('/home/alper')
  try:
   sftp.mkd(dirname)
  except ftplib.error_perm:
   print "Directory: " + dirname  + " already exists, not overwriting directory but overwriting any previously uploaded files. Directory contents:"    
   sftp.dir()
   print 
  
  sftp.cwd(dirname)    
  #print "Contents of " + dirname + " directory:"
  #sftp.dir()
  upload(sftp, filename, forceDirUpload)

def upload(sftp, filename, forceDirUpload):
  # check if this is a folder
  if os.path.isdir(filename):
    print "This is a folder, it's recommended that you compress the directory and send it that way!"
 #   if (forceDirUpload):
 #     allfiles = [] 
 #    subfiles = []
 #     for root, dirs, files in os.walk(filename): 
 #       for f in files: 
 #         allfiles.append(os.path.join(root, f)) 
 #         if root != filename: # I'm in a subdirectory 
 #           subfiles.append(os.path.join(root, f)) 
 #     print "allfiles=", allfiles 
 #     print "NOTE: No two files can have the same name even in different subdirectories!"
 #   else:
 #     print "If you really have to upload a folder, use --forcedir option as well"  
  else:
    upload_file(sftp, filename)
 
bytessofar = 0 
prog = progressBar(0, 2)
currentfile = ''
oldprog = ''
currentfilesize = 0
def upload_file(sftp, filename):
  global prog
  global bytessofar
  global currentfile
  global currentfilesize

  currentfilesize = os.path.getsize(os.path.abspath(filename))
  print "Uploading file " + os.path.abspath(filename) + " of size " + str(currentfilesize)
  prog = progressBar(0, currentfilesize, 50)
  currentfile = os.path.basename(filename)
  try:
   fp = open(os.path.abspath(filename),'r') # file to send
   sftp.storbinary('STOR ' + os.path.basename(filename), fp, 1024, callback=handle) # Send the file
   fp.close() # Close file and FTP
   print "File uploaded, current directory contents: "
   sftp.dir()
   sftp.quit()
  except ftplib.all_errors, e:
   print e
   sftp.quit()

def handle(p):
  global bytessofar
  global prog
  global currentfile
  global oldprog
  global currentfilesize

  bytessofar = bytessofar + 1024
  prog.updateAmount(bytessofar)
  if oldprog != str(prog) or bytessofar % (1024*1000) == 0:
    sys.stdout.write('\b\b\b\b')
    sys.stdout.flush()
    sys.stdout.write('\r' + currentfile + ": " + prog.progBar + "(" + str(bytessofar) + "/" + str(currentfilesize) + ")")
    sys.stdout.flush()
    oldprog = prog.progBar 

def usage():
    print """-p for your university,
     -f or --file to give the data to send. 
     eg. uploaddata.py -p birmingham -f data.tar.gz"""
if __name__ == "__main__":
    main()
