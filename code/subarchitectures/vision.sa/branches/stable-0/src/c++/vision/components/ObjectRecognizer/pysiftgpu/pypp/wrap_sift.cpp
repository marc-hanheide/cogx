/*
 * @author:  Marko Mahnič
 * @created: jan 2009 
 */
   
// class SiftGPU_wrapper, extra code
bp::tuple x_GetFeatureVector()
{
   int nf = GetFeatureNum();
   if (nf < 1) return bp::make_tuple(bp::object(), bp::object());
   // if (nf > 1000) nf = 1000;

   npy_intp dims[2] = { nf, 4 };
   bp::object ftrs(bp::handle<> (PyArray_SimpleNew(2, dims, PyArray_FLOAT)));
   void *ftrs_data = PyArray_DATA((PyArrayObject*) ftrs.ptr());

   dims[1] = 128;
   bp::object dscr(bp::handle<> (PyArray_SimpleNew(2, dims, PyArray_FLOAT)));
   void *dscr_data = PyArray_DATA((PyArrayObject*) dscr.ptr());

   SiftGPU::GetFeatureVector((SiftKeypoint*)ftrs_data, (float*)dscr_data);

   return bp::make_tuple(ftrs, dscr);
}

int x_RunSiftOnNdarray(bp::object image)
{
   PyObject* pimg = image.ptr();
   if (!PyArray_Check(pimg)) {
      PyErr_SetString(PyExc_ValueError, "Expecting a PyArray Object");
      bp::throw_error_already_set();
   }

   // TODO: PyArrayObject* pcont = PyArray_ContiguousFromObject(pimg, PyArray_UBYTE, 2, 3);
   //   if (pcont != NULL)
   void *data = PyArray_DATA((PyArrayObject*) pimg);
   int h = PyArray_DIM(pimg, 0);
   int w = PyArray_DIM(pimg, 1); 

   // GLUT_RGB = 0
   // Matches OpenGL's right now.
#define IL_COLOUR_INDEX     0x1900
#define IL_COLOR_INDEX      0x1900
#define IL_RGB              0x1907
#define IL_RGBA             0x1908
#define IL_BGR              0x80E0
#define IL_BGRA             0x80E1
#define IL_LUMINANCE        0x1909
#define IL_LUMINANCE_ALPHA  0x190A
   int rv = SiftGPU::RunSIFT(w, h, (unsigned char*) data, IL_BGR, GL_UNSIGNED_BYTE);
   // Py_DECREF(pcont)

   return rv;
}

void x_ParseParam(bp::str param_str)
{
   bp::list params = param_str.split();
   PyObject* plpar = (PyObject*) params.ptr();
   int narg = PyObject_Length(plpar);
   if (narg < 1) return;
   char** args = new char* [narg];

   for (int i = 0; i < narg; i++) {
      PyStringObject *pstr = (PyStringObject*) PySequence_GetItem(plpar, i);
      args[i] = pstr->ob_sval;
   }
   SiftGPU::ParseParam(narg, args);

   // Cleanup
   delete args;
}

