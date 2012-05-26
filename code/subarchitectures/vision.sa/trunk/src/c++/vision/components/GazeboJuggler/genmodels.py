#!/usr/bin/python
# vim: set fileencoding=utf-8 sw=4 sts=4 ts=8 et :vim

import sys, os, re
#from PIL import Image, ImageDraw, ImageColor as ico, ImageFont
#import cv
from PyQt4 import QtGui, QtCore

print dir()

def rectPoly(x0, y0, w, h):
    return (( (x0, y0), (x0+w, y0), (x0+w, y0+h), (x0, y0+h), (x0, y0) ), )

Fonts = {
    "arial": "/usr/share/fonts/truetype/msttcorefonts/arial.ttf"
}
#def makeMaterials_cv(self):
#    img = cv.CreateImage((800,600), cv.IPL_DEPTH_8U, 3)
#    print dir(img)
#    cv.FillPoly(img, rectPoly(0, 0, img.width, img.height), cv.CV_RGB(0x20, 0x40, 0xe0))
#    font = cv.InitFont(cv.CV_FONT_HERSHEY_SIMPLEX, 1, 1, 0, 2)
#    cv.PutText(img, "This is the '%s'" % self.name, (10, 30), font, cv.CV_RGB(0xe0, 0x40, 0x20))
#    cv.SaveImage("xdata/x-%s.png" % self.name, img)
#def makeMaterials_pil(self):
#    ppm = 800
#    (w, h) = self.getUnfoldSize(ppm)
#    faces = self.getUnfoldFaces(ppm)
#    img = Image.new("RGB", (w, h), "#2040e0")
#    font = ImageFont.truetype(Fonts["arial"], 32)
#    draw = ImageDraw.Draw(img)
#    draw.text((faces[1][2]/2, faces[1][1]),
#            "This is the '%s'" % self.name, fill=ico.getrgb("#e04020"), font=font)
#    for i in xrange(6):
#        fp = faces[i][:4]
#        fl = faces[i][4]
#        print fp, fl
#        fim = img.copy().crop(fp)
#        fim.save("xdata/x-%s-%s.png" % (self.name, fl[6]))

def readConfig(fname):
    res = {}
    section = None
    content = []
    def addSection(section, content):
        content = "".join(content)
        if section.startswith("python:"):
            content = eval(content)
            section = section[7:]
        res[section] = content

    for line in open(fname).readlines():
        sline = line.strip()
        if sline.startswith("[") and sline.endswith("]"):
            if section != None:
                addSection(section, content)
            content = []
            section = sline.strip(" \t[]")
            continue
        content.append(line)

    if section != None:
        addSection(section, content)

    return res

tmpl_material = """
material %(material_name)s
{
  technique
  {
    pass
    {
      texture_unit
      {
        texture %(image_filename)s
      }
    }
  }
}
"""

tmpl_include_model = """
  <model:physical name="%(model_name)s"> <!-- OBJECT -->
  <static>true</static>
  <xyz>%(x)f %(y)f %(z)f</xyz>
  <rpy>0 0 %(zrot)f</rpy>

  <include embedded="true">
    <xi:include href="%(model_file)s" />
  </include>
  </model:physical>
"""

tmpl_box = readConfig("res/box.tmpl")

tmpl_places="""
[places]
1  0.150  0.176
1 -0.150  0.176
"""

class OgreBox:
    def __init__(self, name, prefix="cogxGeneratedModel", size = (1.0, 1.0, 1.0), color = (0.2, 0.4, 1.0)):
        self.name = name
        self.prefix = prefix
        self.size = size
        self.color = color

    @property
    def material_name(self):
        if self.prefix == "": return self.name

        return "%s/%s" % (self.prefix, self.name)

    def makeModel(self):
        (sx, sy, sz) = self.size

        faces = []
        face_locations = tmpl_box["face-locations"]
        for f in face_locations:
            faces.append(tmpl_box["face"] % {
                    "px": f[0] * sx, "py": f[1] * sy, "pz": f[2] * sz,
                    "sx": f[3] * sx, "sy": f[4] * sy, "sz": f[5] * sz,
                    "material_name": "%s-%s" % (self.material_name, f[6])
                   })

        geometry = tmpl_box["geometry"] % {
                "sx": sx, "sy": sy, "sz": sz,
                "model_name": self.name,
                "faces": "\n".join(faces)
               }

        body = tmpl_box["body"] % {
                "model_name": self.name,
                "geometry": geometry
               }

        model = tmpl_box["model"] % {
                "model_name": self.name,
                "body": body
               }

        return model


    # Unfold Box Layout:
    #       T
    #    L  F  R  BK
    #       B
    # @param ppm = pixels per meter; self.size is in meters
    def getUnfoldSize(self, ppm = 800):
        face_locations = tmpl_box["face-locations"]
        lr_size = [int(ppm * self.size[i] * face_locations[0][i+3]) for i in xrange(3) ]
        fb_size = [int(ppm * self.size[i] * face_locations[2][i+3]) for i in xrange(3) ]
        tb_size = [int(ppm * self.size[i] * face_locations[4][i+3]) for i in xrange(3) ]
        w = fb_size[0] * 2 + lr_size[1] * 2
        h = fb_size[2]     + tb_size[1] * 2
        return (w, h)

    def getUnfoldFaces(self, ppm = 800):
        face_locations = tmpl_box["face-locations"]
        lr_size = [int(ppm * self.size[i] * face_locations[0][i+3]) for i in xrange(3) ]
        fb_size = [int(ppm * self.size[i] * face_locations[2][i+3]) for i in xrange(3) ]
        tb_size = [int(ppm * self.size[i] * face_locations[4][i+3]) for i in xrange(3) ]

        w = fb_size[0] * 2 + lr_size[1] * 2
        h = fb_size[2]     + tb_size[1] * 2
        x1 = lr_size[1]
        x2 = x1 + fb_size[0]
        x3 = x2 + lr_size[1]
        x4 = w + 1
        y1 = tb_size[1]
        y2 = y1 + fb_size[2]
        y3 = h + 1
        faces = [ # same order as in face_locations
            (x1, 0,  x2, y1, face_locations[4]), #T
            (0,  y1, x1, y2, face_locations[0]), #L
            (x1, y1, x2, y2, face_locations[2]), #F 
            (x2, y1, x3, y2, face_locations[1]), #R
            (x3, y1, x4, y2, face_locations[3]), #BK
            (x1, y2, x2, y3, face_locations[5])  #B
            ]
        return faces


    def getTextureString(self, color, niter=200):
        co = [
            "0x%06x" % color.rgb(),
            "0x%06x" % color.lighter(120).rgb(),
            "0x%06x" % color.darker(120).rgb() ]

        text = ""
        s = 0
        for i in xrange(niter):
            count = s % 7 + 3
            sx = ""
            for j in xrange(count):
                s = s + (ord(self.name[(i + j) % len(self.name)]))
                sx += "%c" % (s % 64 + 64)

            text += "%s-%s " % (co[i % 3], sx)
        return text

    # weight: 0-99 or -1; 25 light, 50 normal, 75 bold
    def getBestFont(self, imgDevice, fontname, weight, text, minwidth, maxwidth, maxheight):
        pointsize = 128
        minheight = maxheight / 4
        font = QtGui.QFont(fontname, pointsize, weight)
        fmet = QtGui.QFontMetrics(font, imgDevice)
        tw = fmet.width(text)
        if tw > maxwidth:
            pointsize *= (1.0 * maxwidth / tw)
            font = QtGui.QFont(fontname, pointsize, weight)
            fmet = QtGui.QFontMetrics(font, imgDevice)
        elif tw < minwidth:
            pointsize *= (1.0 * minwidth / tw)
            font = QtGui.QFont(fontname, pointsize, weight)
            fmet = QtGui.QFontMetrics(font, imgDevice)

        if fmet.height() < minheight:
            pointsize *= (1.0 * minheight / fmet.height())
            font = QtGui.QFont(fontname, pointsize, weight)
            fmet = QtGui.QFontMetrics(font, imgDevice)
        elif fmet.height() > maxheight:
            pointsize *= (1.0 * maxheight / fmet.height())
            font = QtGui.QFont(fontname, pointsize, weight)
            fmet = QtGui.QFontMetrics(font, imgDevice)

        print fontname, pointsize, "fw", fmet.width(text), "fh", fmet.height(), "minw", minwidth, "maxh", maxheight
        return font

    def makeMaterials(self):
        ppm = 800
        cobg = QtGui.QColor(self.color[0], self.color[1], self.color[2])
        (w, h) = self.getUnfoldSize(ppm)
        faces = self.getUnfoldFaces(ppm)
        img = QtGui.QImage(QtCore.QSize(w, h), QtGui.QImage.Format_RGB32)
        img.fill(cobg.rgb())
        draw = QtGui.QPainter(img)

        font = QtGui.QFont("courier", 32)
        topt = QtGui.QTextOption()
        draw.setPen(cobg.lighter(115))
        topt.setWrapMode(QtGui.QTextOption.WrapAnywhere)
        stxt = self.getTextureString(cobg)
        draw.setFont(font)
        draw.drawText(QtCore.QRectF(0, 0, w, h), stxt, topt)

        fleft = faces[1]
        szleft = (fleft[2] - fleft[0], fleft[3] - fleft[1])
        ffront = faces[2]
        szfront = (ffront[2] - ffront[0], ffront[3] - ffront[1])
        ftop = faces[0]
        sztop = (ftop[2] - ftop[0], ftop[3] - ftop[1])

        draw.setPen(cobg.darker(120))
        font = self.getBestFont(img, "arial", 75, self.name, szleft[0] * 1.5 + szfront[0], w, szleft[1] * 0.6)
        fmet = QtGui.QFontMetrics(font, img)
        draw.setFont(font)
        draw.drawText(0, fleft[1] + fmet.ascent(), "%s" % self.name)

        draw.setPen(cobg.darker(140))
        font = self.getBestFont(img, "times", 75, self.name, szleft[0] + szfront[0] * 1.5, w, szleft[1] * 0.6)
        fmet = QtGui.QFontMetrics(font, img)
        draw.setFont(font)
        draw.drawText(w - fmet.width(self.name), fleft[3] - fmet.descent(), "%s" % self.name)

        draw.setPen(cobg.darker(130))
        font = self.getBestFont(img, "courier", 75, self.name, sztop[0] * 0.8, sztop[0] * 1.5, sztop[1])
        fmet = QtGui.QFontMetrics(font, img)
        draw.setFont(font)
        draw.drawText(QtCore.QRectF(ftop[0], ftop[1], sztop[0], sztop[1]),
                "%s" % self.name, topt)

        draw.end()

        materials = []
        for i in xrange(6):
            fp = faces[i][:4] # face points
            fl = faces[i][4]  # face_location
            print fp, fl
            fim = img.copy(QtCore.QRect(fp[0], fp[1], fp[2] - fp[0], fp[3] - fp[1]))
            fname = "x-%s-%s.png" % (self.name.lower(), fl[6])
            fname = fname.replace(" ", "-")
            matname = "%s-%s" % (self.material_name, fl[6]) 
            fim.save("xdata/Media/materials/textures/%s"  % fname)
            materials.append(tmpl_material % {
                    "material_name": matname,
                    "image_filename": fname })

        return "\n".join(materials)

class ProjectWriter:
    def __init__(self, prjname):
        self.prjname = prjname
        self.fmat = None
        self.fscn = None
        self.juggler = None
        pass

    def prepareDirs(self):
        def mkdir(path):
            if not os.path.exists(path):
                os.makedirs(path)
        mkdir("xdata/Media/materials/scripts")
        mkdir("xdata/Media/materials/textures")
        mkdir("xdata/models")

    def openFiles(self):
        self.fmat = open("xdata/Media/materials/scripts/%s.material" % self.prjname, "w")
        self.fscn = open("xdata/%s.objects.scn" % self.prjname, "w")
        self.juggler = open("xdata/%s.objects.txt" % self.prjname, "w")
        self.juggler.write("[objects]\n")

    def closeFiles(self):
        self.fmat.close()
        self.fscn.close()
        self.juggler.write(tmpl_places)
        self.juggler.close()

    def label2modelname(self, label):
        name = re.sub("([a-z])\s*([A-Z])", "\\1\\2", label)
        name = re.sub("[^a-zA-Z0-9.]+", "-", name)
        return name

    def label2filename(self, label):
        name = "xdata/models/gen-%s.model" % (self.label2modelname(label))
        return name

    def createOjbect(self, label, ogreObj):
        fname = self.label2filename(label)
        fmod = open(fname, "w")
        fmod.write("""<?xml version="1.0"?>\n""");
        fmod.write(ogreObj.makeModel())
        fmod.close()

        self.fmat.write(ogreObj.makeMaterials())

    def insertObject(self, label, x, y, z, xrot=0, yrot=0, zrot=0):
        name = self.label2modelname(label)
        fname = self.label2filename(label)
        self.fscn.write(tmpl_include_model % {
                "model_name": name,
                "model_file": fname[fname.find('/')+1:],
                "x": x, "y": y, "z": z,
                "xrot": xrot, "yrot": yrot, "zrot": zrot
               })
        self.juggler.write("%s\n" % (name))

    def mergeWorld(self, template, placeholder, outfile):
        wrld = open(template).read()
        objs = open("xdata/%s.objects.scn" % self.prjname).read()
        wrld = wrld.replace(placeholder, objs)
        open(outfile, "w").write(wrld)


def createObjects(prjname):
    labels = ["SanDisk Flash", "Nokia Phone", "Staedtler Textsurfer", "Jaffa Cakes"]
    # x:-, y:/, z:|
    sizes  = [(0.5, 0.3, 0.6), (0.4, 0.2, 0.6), (0.5, 0.3, 0.2)]
    # rgb
    colors = [(0x20, 0x40, 0xe0, "b"), (0xe0, 0x30, 0x10, "r"), (0x10, 0xd0, 0x30, "g")]

    PW = ProjectWriter(prjname)
    PW.prepareDirs()
    PW.openFiles()

    a = QtGui.QApplication(sys.argv) # Qt is used for rendering
    x = 0.1; y = 0.8; z = 1.5; zrot = 0
    for l in labels:
        for si,s in enumerate(sizes):
            for ci,c in enumerate(colors):
                label = "%s-%s-%d" % (c[3], l, si)
                b = OgreBox(label, size = s, color = c[:3], prefix = "cogx")
                PW.createOjbect(label, b)
                PW.insertObject(label, x, y, z, zrot=zrot)
                z += 1.1
                zrot = (zrot + 7) % 360
    PW.closeFiles()
    PW.mergeWorld("res/bigtest.world.in", "#OBJECTLIST#", "xdata/%s.world" % prjname)

createObjects("test1")

