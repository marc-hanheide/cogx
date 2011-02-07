
-- Camera objects are managed in C++ code
function setCamera(name, xFrom, yFrom, zFrom, xTo, yTo, zTo, xUp, yUp, zUp)
   v11nCameraLookAt(_v11n_script_object_, name, xFrom, yFrom, zFrom, xTo, yTo, zTo, xUp, yUp, zUp)
end
