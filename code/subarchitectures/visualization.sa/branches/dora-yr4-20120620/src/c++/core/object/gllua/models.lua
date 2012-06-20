
StdModel = {}

function StdModel:box(sx, sy, sz)
   sx = sx*0.5
   sy = sy*0.5
   sz = sz*0.5

   for i=1,2 do 
      if i == 1 then a = 1.0
      else a = -1.0 end

      -- front/back
      glBegin(GL_QUADS)
      glNormal(0.0, 0.0,     a)
      glVertex(-sx, -sy,  sz*a)
      glVertex( sx, -sy,  sz*a)
      glVertex( sx,  sy,  sz*a)
      glVertex(-sx,  sy,  sz*a)
      glEnd()

      -- top/bottom
      glBegin(GL_QUADS)
      glNormal(0.0,    a, 0.0)
      glVertex(-sx, sy*a,  sz)
      glVertex(-sx, sy*a, -sz)
      glVertex( sx, sy*a, -sz)
      glVertex( sx, sy*a,  sz)
      glEnd()

      -- right/left
      glBegin(GL_QUADS)
      glNormal(   a, 0.0, 0.0)
      glVertex(sx*a, -sy,  sz)
      glVertex(sx*a, -sy, -sz)
      glVertex(sx*a,  sy, -sz)
      glVertex(sx*a,  sy,  sz)
      glEnd()
   end
end

function StdModel:sincos(np)
   local tsin = {}
   local tcos = {}
   local dan = 2*math.pi / np
   local an = 0.5*dan
   for i=1,np do
      tsin[i] = math.sin(an+dan*(i-1))
      tcos[i] = math.cos(an+dan*(i-1))
   end
   return tsin, tcos
end

function StdModel:ellipse(rx, ry, np)
   if np < 3 then np = 3 end
   local dan = 2*math.pi / np
   local an = 0.5*dan
   glBegin(GL_TRIANGLES)
   for i=1,np do
      glVertex(0.0, 0.0, 0.0)
      glVertex(math.cos(an+dan*(i-1)), math.sin(an+dan*(i-1)), 0.0)
      glVertex(math.cos(an+dan*(i)), math.sin(an+dan*(i)), 0.0)
   end
   glEnd()
end

function StdModel:cylinder(rx, ry, sz, np)
   if np < 3 then np = 3 end
   local tsin, tcos
   local k, i, ni
   tsin, tcos = self:sincos(np)
   for i=1,#tcos do tcos[i] = rx*tcos[i] end
   for i=1,#tsin do tsin[i] = ry*tsin[i] end
   sz = sz*0.5
   glNormal(0.0, 0.0, 1.0)
   glBegin(GL_TRIANGLES)
   for i=1,np do
      glVertex(0.0, 0.0, sz)
      glVertex(tcos[i], tsin[i], sz)
      ni = i % np + 1
      glVertex(tcos[ni], tsin[ni], sz)
   end
   glEnd()
   glNormal(0.0, 0.0, -1.0)
   glBegin(GL_TRIANGLES)
   for i=1,np do
      glVertex(0.0, 0.0, -sz)
      glVertex(tcos[i], tsin[i], -sz)
      ni = i % np + 1
      glVertex(tcos[ni], tsin[ni], -sz)
   end
   glEnd()
   glBegin(GL_QUADS)
   for i=1,np do
      ni = i % np + 1
      glNormal(tcos[i], tsin[i], 0.0)
      glVertex(tcos[i], tsin[i], sz)
      glVertex(tcos[i], tsin[i], -sz)
      glVertex(tcos[ni], tsin[ni], -sz)
      glVertex(tcos[ni], tsin[ni], sz)
   end
   glEnd()
end

-- TODO: (origin, size, normal)
function StdModel:xfloor(x)
   size = 5
   sy = size
   sz = size
   gl.Begin(gl.QUADS)
   gl.Color(0.7, 0.7, 0.7)
   gl.Normal(1.0, 0.0, 0.0)
   gl.Vertex(x, -sy,  sz)
   gl.Vertex(x, -sy, -sz)
   gl.Vertex(x,  sy, -sz)
   gl.Vertex(x,  sy,  sz)
   gl.End()
   gl.Begin(gl.LINES)
   gl.LineWidth(4.0)
   gl.Color(1.0, 1.0, 1.0)
   for i=-size,size do
      gl.Vertex(x, i,  size)
      gl.Vertex(x, i, -size)
      gl.Vertex(x,  size, i)
      gl.Vertex(x, -size, i)
   end
   gl.LineWidth(1.0)
   gl.End()
end
function StdModel:zfloor(z)
   size = 5
   sx = size
   sy = size
   gl.Begin(gl.QUADS)
   gl.Color(0.7, 0.7, 0.7)
   gl.Normal(0.0, 0.0, 1.0)
   gl.Vertex(-sx, sy,  z)
   gl.Vertex(-sx, -sy, z)
   gl.Vertex( sx, -sy, z)
   gl.Vertex( sx, sy,  z)
   gl.End()
   gl.Begin(gl.LINES)
   gl.LineWidth(4.0)
   gl.Color(1.0, 1.0, 1.0)
   for i=-size,size do
      gl.Vertex(i,  size, z)
      gl.Vertex(i, -size, z)
      gl.Vertex( size, i, z)
      gl.Vertex(-size, i, z)
   end
   gl.LineWidth(1.0)
   gl.End()
end
