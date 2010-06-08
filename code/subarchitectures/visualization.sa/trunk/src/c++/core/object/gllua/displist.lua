
DispList = { context={} }

function DispList:create(name)
   local ctx = v11nGetOpenGlContext()
   assert(ctx, "No OpenGL context is active")
   local displist = self.context[ctx]
   if displist==nil then
      displist = {}
      self.context[ctx] = displist
   end
   assert(not displist[name], "Compiled display list named '" .. name .. "' exists.")

   local id = glGenLists(1)
   --_glErrorCheck()
   assert(id~=0, "glGenLists failed")
   glNewList(id, GL_COMPILE_AND_EXECUTE)
   --_glErrorCheck()
   assert(glIsList(id)==GL_TRUE, "glNewList failed")
   displist[name] = id
end

function DispList:replace(name)
   local ctx = v11nGetOpenGlContext()
   assert(ctx, "No OpenGL context is active")
   local displist = self.context[ctx]
   if displist==nil then
      displist = {}
      self.context[ctx] = displist
   end
   local id = displist[name] 
   if id then
      displist[name] = nil
      glDeleteLists(id, 1)
   end

   id = glGenLists(1)
   --_glErrorCheck()
   assert(id~=0, "glGenLists failed")
   glNewList(id, GL_COMPILE_AND_EXECUTE)
   --_glErrorCheck()
   assert(glIsList(id)==GL_TRUE, "glNewList failed")
   displist[name] = id
end

function DispList:endList()
  glEndList()
end

function DispList:delete(name)
   local ctx = v11nGetOpenGlContext()
   assert(ctx, "No OpenGL context is active")
   local displist = self.context[ctx]
   if not displist then return end
   local id = displist[name] 
   if not id then return end
   glDeleteLists(id, 1)
end

function DispList:exists(name)
   local ctx = v11nGetOpenGlContext()
   if not ctx then return false end
   local displist = self.context[ctx]
   if not displist then return false end
   local id = displist[name] 
   if not id then return false end
   return true
end

function DispList:draw(name)
   local ctx = v11nGetOpenGlContext()
   assert(ctx, "No OpenGL context is active")
   local displist = self.context[ctx]
   assert(displist, "No lists have been defined in this context")
   local id = displist[name] 
   assert(id, "DispList named '" .. name .. "' not found")
   glCallList(id)
end

