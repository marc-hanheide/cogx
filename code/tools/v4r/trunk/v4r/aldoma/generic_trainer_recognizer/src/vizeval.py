#!/usr/bin/python
## walter wohlkinger
import sys
import os
from xml.dom.minidom import parse
import string
#import numpy
#import cv
import shutil
import ftplib
import os
import sys
import traceback

def result_list_callback(msg):

   str =  ''.join(msg.filelist)
   param = ' -multiview 1 -bc 1,1,1  '
   print "pcd_viewer " + param + str
   subprocess.Popen("pcd_viewer " + param + str, shell=True,stdout=None,stderr=None)


def convert_pcds():
    directory = '/home/walter/3DNET/Cat10_TestDatabase/pcd/'
    for fn in os.path(directory):
        if os.path.isfile(directory + fn):
            subprocess.Popen("convert_pcd_ascii_binary " + directory + fn + " " + directory + fn + " 1", shell=True,stdout=None,stderr=None).wait()
    


def upload_eval(filename):
    print "uploading File:", filename
    print "Logging in..."
    ftp = ftplib.FTP()
    ftp.connect('melanie.webhoster.ag')
    print ftp.getwelcome()
    try:
        try:
            ftp.login('web1355f2','public')
            ftp.cwd('eval')
            # move to the desired upload directory
#            print "Currently in:", ftp.pwd()
            print "Uploading...",
            fullname = filename
            name = os.path.split(fullname)[1]
            f = open(fullname, "rb")
            ftp.storbinary('STOR ' + name, f)
            f.close()
            print "OK"
        finally:
            print "Quitting..."
            ftp.quit()
    except:
        traceback.print_exc()

def main(fn):
    
    rf = open(fn)
    modeldir = rf.readline().split()[1] 
    traindir = rf.readline().split()[1] 
    testdir  = rf.readline().split()[1] 
    descr    = rf.readline().split()[1] 
    numbernn = int(rf.readline().split()[1]) 
  
    if not os.path.isdir(testdir+'/eval'):
        os.mkdir(testdir+'/eval')
    evaldir  = testdir+'/eval'
          
    NNd = {}  # count, correct_ones
    NNf = {}
    scenes = []
    for line in rf:
        eva = line.strip()
        #get annotation for this testcase
        print eva
        annofn = testdir + '/annotation/' + eva.split()[0].replace('.pcd','.anno')
        annof = open(annofn)
        tmpstr = annof.readline().strip()
        print tmpstr
        anno = tmpstr[tmpstr.rfind(tmpstr.split()[4]):]
        annof.close()
        
        if not NNd.has_key(anno):
            NNd[anno] = [1,0.0,0.0]
            NNf[anno] = {}
        else:
            NNd[anno][0] += 1    
        
        snn = 0
        mnn = 0
        #NN check 1st returned directory+filename for classname in xml ( classname != directory name
        # if there are no results in the result file, for whatever reason
        if len(eva.split()) < numbernn:
            continue
        
        mdir = eva.split()[1].split('/')[0]
        dom = parse(modeldir + "/" + mdir + "/metainfo.xml") 
        node = dom.getElementsByTagName('classname')[0]
        classname = string.strip(node.firstChild.data)   
        
        if classname == anno:
            NNd[anno][1] += 1
            snn = 1
        
        #NN with numbernn.. check if one of the NN is the correct class
        for i in range(2,numbernn):
            mdir = eva.split()[i].split('/')[0]
            dom = parse(modeldir + "/" + mdir + "/metainfo.xml") 
            node = dom.getElementsByTagName('classname')[0]
            classname = string.strip(node.firstChild.data)   
        
            if classname == anno:
                NNd[anno][2] += 1
                mnn = 1
                break

        # find the most distracting category
        for i in range(2,numbernn):
            mdir = eva.split()[i].split('/')[0]
            dom = parse(modeldir + "/" + mdir + "/metainfo.xml") 
            node = dom.getElementsByTagName('classname')[0]
            classname = string.strip(node.firstChild.data)   
            
            if not classname == anno:
                if NNf[anno].has_key(classname):
                    NNf[anno][classname] += 1
                else:
                    NNf[anno][classname] = 1

        #test scene name, image name+path, correct 1/0 for NN1, correct 1/0 for NNx, best matching view/model
        scenes.append([eva.split()[0], testdir + '/image_color/' + eva.split()[0].replace('.pcd','.png'), snn,mnn, eva.split()[1]])
        #if mnn == 0:
        #    cv.SaveImage(evaldir + '/' + eva.split()[0].replace('.pcd','.jpg'), cv.LoadImage(testdir + '/image_color/' + eva.split()[0].replace('.pcd','.png')))

    #print NNd
    #print numpy.asarray(scenes)
    nndf = open(evaldir + '/' + descr + '_NN1+'+str(numbernn)+'.result_overview','w')
    overallnn1 = 0.0
    overallnnx = 0.0
    overallcnt = 0
    for a in NNd.keys():
        overallcnt +=NNd[a][0]
        overallnn1 += NNd[a][1]
        overallnnx += NNd[a][2]
    
    print overallcnt,overallnn1,overallnnx
    nndf.write('classname | % correct class in first NN | % correct class in top '+str(numbernn)+'-NN | most distracting class \n\n')
    if overallcnt > 0:
        nndf.write('            #OVERALL | {:.2%}.'.format(overallnn1/overallcnt) + ' | {:.2%}.'.format(overallnnx/overallcnt) + '\n')
    else:
        nndf.write('            #OVERALL | 0.0 | 0.0 \n')
        
    for a in sorted(NNd.keys()):
        
        #max(stats.iteritems(), key=operator.itemgetter(1))[0]
        nndf.write(a.rjust(20) + ' | {:0.2%}.'.format(NNd[a][1]/NNd[a][0]) + ' | {:3.2%}.'.format(NNd[a][2]/NNd[a][0]) + ' ' + max(NNf[a], key=NNf[a].get)  + '\n' )
        
    nndf.close()
    #scenesf = open(evaldir + '/' + descr + '_NN1+'+str(numbernn)+'.result_detail','w')
    #scenesf.write(scenes)
    #scenesf.close()
    
    #upload_eval(evaldir + '/' + descr + '_NN1+'+str(numbernn)+'.result_overview')
    
   
if __name__ == "__main__":   
    
    if len(sys.argv) < 2:
        print "usage: ./vizeval path_to_results_file"
    else: 
        main(sys.argv[1])
