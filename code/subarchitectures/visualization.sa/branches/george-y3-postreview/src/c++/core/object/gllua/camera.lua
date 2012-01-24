
-- Camera objects are managed in C++ code.
-- _v11n_script_object_ is set in C++ code and is a pointer to the CLuaGlScript object
-- that evaluates this script.
function setCamera(name, xEye, yEye, zEye, xView, yView, zView, xUp, yUp, zUp)
   v11nCamera_SetPosition(_v11n_script_object_, name, xEye, yEye, zEye, xView, yView, zView, xUp, yUp, zUp)
end
