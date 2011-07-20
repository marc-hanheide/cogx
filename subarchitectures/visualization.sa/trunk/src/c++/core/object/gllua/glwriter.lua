
-- _v11n_glw_interface_ is set in C++ code and is a pointer to the CLuaGlScript object
-- that evaluates this script.
function showLabel(x, y, z, text, size)
   v11nGlw_RenderText(_v11n_glw_interface_, x, y, z, text, size)
end
