
DispList = { context={}, dirty={} }

function DispList:_getDispListTab(create)
   create = create or false
   local ctx = v11nGetOpenGlContext()
   assert(ctx, "No OpenGL context is active")
   local displist = self.context[ctx]
   if displist==nil and create then
      displist = {}
      self.context[ctx] = displist
   end
   return displist
end

--function DispList:genList(name)
--   local displist = self:_getDispListTab(create=true)
--   assert(not displist[name], "Compiled display list named '" .. name .. "' exists.")

--   local id = glGenLists(1)
--   assert(id~=0, "glGenLists failed")
--   displist[name] = id
--   return id
--end

-- Define or redefine a display list
function DispList:newList(name)
   local displist = self:_getDispListTab(true)
   local id = displist[name] 
   if not id then
      id = glGenLists(1)
      assert(id~=0, "glGenLists failed")
      displist[name] = id
   end

   --glNewList(id, GL_COMPILE_AND_EXECUTE)
   glNewList(id, GL_COMPILE)
   assert(glIsList(id)==GL_TRUE, "glNewList failed")
end

function DispList:endList()
  glEndList()
end

function DispList:delete(name)
   local displist = self:_getDispListTab(false)
   if not displist then return end
   local id = displist[name] 
   if not id then return end
   displist[id] = nil
   if not glIsList(id) then return end
   glDeleteLists(id, 1)
end

function DispList:exists(name)
   local displist = self:_getDispListTab(false)
   if not displist then return false end
   local id = displist[name] 
   if not id then return false end
   if not glIsList(id) then return false end
   return true
end

function DispList:draw(name)
   local displist = self:_getDispListTab(false)
   assert(displist, "No lists have been defined in this context")
   local id = displist[name] 
   assert(id, "DispList named '" .. name .. "' not found")
   glCallList(id)
end

function DispList:setDirty(name)
   self.dirty[name] = true
end

function DispList:getDirty(names)
   local displist = self:_getDispListTab(false)
   local dirt = self.dirty
   if next(dirt) ~= nil then -- not empty, clear for next round
      self.dirty = {}
   end
   if displist == nil then
      for _,n in pairs(names) do
         dirt[n] = true
      end
   else
      for _,n in pairs(names) do
         if not displist[n] then
            dirt[n] = true
         end
      end
   end
   return dirt
end
