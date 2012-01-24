import os, sys

import pymodulepaths
import sqlite3 as sqlite
import numpy

import ObjectRecognizer.objectmodel as objectmodel
mdlpath = "/home/mmarko/Documents/doc/Devel/CogX/code/systems/ul/xdata/models/800"
mdlpath = "/home/mmarko/Documents/luvss/Devel/CogX/code/systems/ul/xdata/models/800"

def loadModel(mdldir, name):
    m = objectmodel.CObjectModel(name)
    m.FM.setModelStorePath(mdldir)
    m.loadViews(delayedLoad=False)
    m.loadPairMatches(mdldir, name)
    return m

def makedb(dboutput="xdata/test.db"):
    db = sqlite.connect(dboutput)

    try: db.execute(
        """
        create table models (
            id          integer primary key,
            name        text
        );
        """)
    except: pass

    try: db.execute(
        """
        create table views (
            id          integer primary key,
            modelid     integer,
            imagefile   text,
            hasPose     integer,
            phi         float,
            lambda      float,
            rotation    float,
            foreign key (modelid) references models(id)
        );
        """)
    except: pass

    try: db.execute(
        """
        create table sifts (
            id          integer primary key,
            viewid      integer,
            cx          float,
            cy          float,
            scale       float,
            orientation float,
            descriptor  blob,
            foreign key (viewid) references views(id)
        );
        """)
    except: pass

    db.close()

def convert(mdlpath, name, dboutput="xdata/test.db"):
    m = loadModel(mdlpath, name) # "CvetMetaTea")
    db = sqlite.connect(dboutput)
    cur = db.cursor()
    cur.execute("""insert into models values (NULL,?);""", [name])
    modelid = cur.lastrowid
    for i, vp in enumerate(m.viewPoints):
        cur.execute(
            """insert into views values (NULL,?,?,?,?,?);""",
            [modelid, "%s_VP%03d_L%03d" % (name, vp.vpPhi, vp.vpLambda), vp.vpPhi, vp.vpLambda, 0]
        )
        viewid = cur.lastrowid
        fp = vp.featurePacks[0]
        for k in xrange(len(fp)):
            feature = fp[k]
            # SiftGPU returns floats in range 0.0-0.5; convert to 0-255
            descriptor = (feature.descriptor() * 512+0.5).astype(numpy.uint8)
            descriptor = descriptor.data
            cur.execute(
                """insert into sifts values (NULL,?,?,?,?,?,?);""",
                [viewid, float(feature.x), float(feature.y),
                 float(feature.scale), float(feature.orientation),
                 descriptor]
            )

        print "View", viewid, "saved"
        db.commit()

def loadNew(name):
    db = sqlite.connect("xdata/test.db")
    views = []
    cur = db.cursor()
    cur.execute("select id from models where models.name=?", [name])
    modelid = cur.fetchone()
    if modelid == None:
        print "No such model", name
        return
    modelid = modelid[0]
    cur.execute("select * from views where modelid=? order by views.id", [modelid])
    for row in cur:
        view = [row[1], row[2], row[3], row[4]]
        csift = db.cursor()
        csift.execute("select * from sifts where viewid=? order by sifts.id", [row[0]])
        sifts = []
        for s in csift:
            feature = [s[1], s[2], s[3], s[4], s[5]]
            descr = numpy.frombuffer(s[6], dtype=numpy.uint8).astype(numpy.float32) / 512
            feature.append(descr)
            sifts.append(feature)
        print "view", row[0], "at", row[2], row[3], "has", len(sifts), "features"
        view.append(sifts)
        views.append(view)
    db.close()


def tmoldmodel():
    loadModel(mdlpath, "CvetMetaTea")

def tmnewmodel():
    loadNew("CvetMetaTea")

def test():
    if 0: makedb()
    if 0: convert(mdlpath, "CvetMetaTea")
    if 1: loadNew("CvetMetaTea")
    if 0:
        from timeit import Timer
        print "Old format"
        t = Timer("tmoldmodel()", "from __main__ import tmoldmodel")
        print t.timeit(number=10)
    if 1:
        from timeit import Timer
        print "New format"
        t = Timer("tmnewmodel()", "from __main__ import tmnewmodel")
        print t.timeit(number=10)


if __name__ == "__main__":
    if len(sys.argv) > 1:
        db = sys.argv[1]
        fn = sys.argv[2]
        makedb(db)
        convert(os.path.dirname(fn), os.path.basename(fn), db)
    else:
        test()
