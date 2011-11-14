#!/usr/bin/python
# vim: set fileencoding=utf-8 sw=4 sts=4 ts=8 et :vim

import sys
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

tmpl_material = """
material %(material_name)
{
  technique
  {
    pass
    {
      texture_unit
      {
        texture %(image_filename)
      }
    }
  }
}
"""

tmpl_box = {
        "face": """
<visual>
  <xyz>%(px)g %(py)g %(pz)g</xyz>
  <scale>%(sx)g %(sy)g %(sz)g</scale>
  <mesh>unit_box</mesh>
  <material>%(material_name)s</material>
</visual>
""",

        "geometry": """
<geom:box name="%(model_name)s_geom">
  <xyz>0 0 0</xyz>
  <rpy>0 0 0</rpy>
  <size>%(sx)g %(sy)g %(sz)g</size>
  <mass>0.1</mass>

  %(faces)s

</geom:box>
""",

        "body": """
<body:box name="%(model_name)s_body">
  <xyz>0 0 0</xyz>
  <rpy>0 0 0</rpy>

  <mass>0.05</mass>
  <massMatrix>true</massMatrix>
  <ixx>0.001</ixx>
  <ixy>0.0</ixy>
  <ixz>0.0</ixz>
  <iyy>0.0005</iyy>
  <iyz>0.0</iyz>
  <izz>0.001</izz>
  <cx>0.0</cx>
  <cy>0.0</cy>
  <cz>0.0</cz>

  %(geometry)s

</body:box>
""",

        "model": """
<model:physical name="%(model_name)s_model"
  xmlns:model="http://playerstage.sourceforge.net/gazebo/xmlschema/#model" 
  xmlns:controller="http://playerstage.sourceforge.net/gazebo/xmlschema/#controller" 
  xmlns:interface="http://playerstage.sourceforge.net/gazebo/xmlschema/#interface" 
  xmlns:sensor="http://playerstage.sourceforge.net/gazebo/xmlschema/#sensor" 
  xmlns:body="http://playerstage.sourceforge.net/gazebo/xmlschema/#body" 
  xmlns:geom="http://playerstage.sourceforge.net/gazebo/xmlschema/#geom" 
  xmlns:joint="http://playerstage.sourceforge.net/gazebo/xmlschema/#joint" 
  >

  <xyz>0 0 0</xyz>
  <rpy>0 0 0</rpy>
  <static>false</static>
  <canonicalBody>%(model_name)s_body</canonicalBody>

  %(body)s

</model:physical>
""",

        "face-locations": [ # position, scale, name
                ( 0.5, 0, 0, 0, 1, 1, "left"),
                (-0.5, 0, 0, 0, 1, 1, "right"),
                (0,  0.5, 0, 1, 0, 1, "front"),
                (0, -0.5, 0, 1, 0, 1, "back"),
                (0, 0,  0.5, 1, 1, 0, "top"),
                (0, 0, -0.5, 1, 1, 0, "bottom"),
               ]
}

class OgreBox:
    def __init__(self, name, prefix="cogxGeneratedModel", size = (1.0, 1.0, 1.0), color = (0.2, 0.4, 1.0)):
        self.name = name
        self.prefix = prefix
        self.size = size
        self.color = color

    def makeModel(self):
        (sx, sy, sz) = self.size

        if self.prefix == "": material_name = self.name
        else: material_name = "%s/%s" % (self.prefix, self.name)

        faces = []
        face_locations = tmpl_box["face-locations"]
        for f in face_locations:
            faces.append(tmpl_box["face"] % {
                    "px": f[0] * sx, "py": f[1] * sy, "pz": f[2] * sz,
                    "sx": f[3] * sx, "sy": f[4] * sy, "sz": f[5] * sz,
                    "material_name": "%s-%s" % (material_name, f[6])
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


    def makeMaterials(self):
        ppm = 800
        cobg = QtGui.QColor(0x20, 0x40, 0xe0)
        (w, h) = self.getUnfoldSize(ppm)
        faces = self.getUnfoldFaces(ppm)
        img = QtGui.QImage(QtCore.QSize(w, h), QtGui.QImage.Format_RGB32)
        img.fill(cobg.rgb())

        draw = QtGui.QPainter(img)
        draw.setPen(cobg.lighter(120))
        draw.drawLine(0, 0, w, h)
        font = QtGui.QFont("arial", 24)
        finf = QtGui.QFontInfo(font)
        draw.setFont(font)
        draw.drawText(faces[1][2]/2, faces[1][1] + finf.pixelSize(), "This is the '%s'" % self.name)

        draw.end()
        for i in xrange(6):
            fp = faces[i][:4] # face points
            fl = faces[i][4]  # face_location
            print fp, fl
            fim = img.copy(QtCore.QRect(fp[0], fp[1], fp[2] - fp[0], fp[3] - fp[1]))
            fim.save("xdata/x-%s-%s.png" % (self.name, fl[6]))



a = QtGui.QApplication(sys.argv)
b = OgreBox("example", size = (0.5, 0.3, 0.6))
print b.makeModel()
b.makeMaterials()


