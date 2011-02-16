#!/usr/bin/env python
# vim:set fileencoding=utf-8 sw=4 ts=8 et:vim

# Quick and dirty: .
#   - input: path to directory with images of the object
#     - model name is the name of the directory
#     - all images in the directory are are processed
#     - no clipping
#   - output: updated sqlilte database
#     - existing models are replaced

import pymodulepaths
import os, sys
import mksqlite
import sqlite3 as sqlite
import ObjectRecognizer.featuresetup as fsetup
import numpy as np
from PIL import Image

Features = fsetup.CSiftSetup(fsetup.CSiftSetup.GPU, fsetup.CSiftSetup.NUMPY)

###########################################################################
def PIL2NumPy(input):
    modes_map = {
        "RGB" : np.uint8,
        "L"   : np.uint8,
        "F"   : np.float32
        }

    if not modes_map.has_key(input.mode):
        raise ValueError, 'Unknown or unsupported input mode!. Supported modes are RGB, L and F.'

    result_ro = np.asarray(input, dtype=modes_map[input.mode])  # Read-only array
    return result_ro.copy()  # Return a writeable array
###########################################################################


class CConfig:
    def __init__(self):
        self.imageDir = os.path.join(os.path.abspath("."), "xdata")
        self.modelDb = "xdata/models.db"
        self.modelName = os.path.basename(self.imageDir)
        print self.modelName

    def setImagePath(self, path):
        self.imageDir = path
        self.modelName = os.path.basename(self.imageDir)

def extract(filename):
    im = Image.open(filename)
    im = im.convert("L")
    arr = PIL2NumPy(im)
    return Features.extractor.extractFeatures(arr)

def removeModel(db, name):
    cur = db.cursor()
    cur.execute(
        """
        delete from sifts where viewid in
            (select id from views where modelid in
                (select id from models where name = ?)
            );
        """,
        [name])
    cur.execute(
        """
        delete from views where modelid in
            (select id from models where name = ?);
        """,
        [name])
    cur.execute(
        """
        delete from models where name = ?;
        """,
        [name])
    db.commit()

def run(config):
    mksqlite.makedb(config.modelDb)

    db = sqlite.connect(config.modelDb)
    removeModel(db, config.modelName)

    cur = db.cursor()
    cur.execute("""insert into models values (NULL,?);""", [config.modelName])
    modelid = cur.lastrowid
    db.commit()
    print modelid

    fileList = os.listdir(config.imageDir)
    fileList = [ os.path.join(config.imageDir, fn) for fn in fileList if fn.endswith(".png")]

    for fn in fileList:
        # set hasPose to 0 since we usually can't guess from filename
        cur.execute(
            """insert into views (id,modelid,imagefile,hasPose,phi,lambda,rotation)
            values (NULL,?,?,?,?,?,?);""",
            [modelid, os.path.basename(fn), 0, 0, 0, 0]
        )
        viewid = cur.lastrowid

        sifts = extract(fn)
        for k in xrange(len(sifts)):
            feature = sifts[k]
            # SiftGPU returns floats in range 0.0-0.5; convert to 0-255
            descriptor = (feature.descriptor() * 512+0.5).astype(np.uint8)
            descriptor = descriptor.data
            cur.execute(
                """insert into sifts values (NULL,?,?,?,?,?,?);""",
                [viewid, float(feature.x), float(feature.y),
                 float(feature.scale), float(feature.orientation),
                 descriptor]
            )
        db.commit()
    db.close()

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print "Usage:"
        print "   buildmodel database modelDir [modelDir ...]"
        print "     - database is a sqlite3 database with models"
        print "     - modelDir contains png images to be processed"
        print
    else:
        config = CConfig()
        config.modelDb = sys.argv[1]
        for param in sys.argv[2:]:
            config.setImagePath(param)
            run(config)
