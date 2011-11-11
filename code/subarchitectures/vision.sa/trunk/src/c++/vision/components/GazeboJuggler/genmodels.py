#!/usr/bin/python
# vim: set fileencoding=utf-8 sw=4 sts=4 ts=8 et :vim

from PIL import Image, ImageDraw, ImageColor as ico, ImageFont

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


    def makeMaterials(self):
        img = Image.new("RGB", (800, 600), "#2040e0")
        font = ImageFont.truetype("/usr/share/fonts/truetype/msttcorefonts/arial.ttf", 32)
        draw = ImageDraw.Draw(img)
        draw.text((10, 10), "This is the '%s'" % self.name, fill=ico.getrgb("#e04020"), font=font)
        img.save("xdata/x-%s.png" % self.name)


print OgreBox("example").makeModel()
OgreBox("example").makeMaterials()


