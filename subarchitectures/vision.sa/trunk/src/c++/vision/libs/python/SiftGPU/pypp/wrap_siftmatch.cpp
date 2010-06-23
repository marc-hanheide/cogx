/*
 * @author:  Marko Mahnič
 * @created: jan 2009 
 */
   
// class SiftMatchGPU_wrapper, extra code

#define PyArray_NDIMS(obj) (((PyArrayObject *)(obj))->nd)
void x_CheckArray(int param_index, bp::object a)
{
   PyObject* pobj = a.ptr();
   if (!PyArray_Check(pobj)) {
      PyErr_SetString(PyExc_ValueError, "Expecting a PyArray Object");
      bp::throw_error_already_set();
   }
   if (PyArray_NDIMS(pobj) != 2 || PyArray_DIM(pobj, 1) != 128) {
      PyErr_SetString(PyExc_ValueError, "Expecting a PyArray Object with Dimensions Nx128");
      bp::throw_error_already_set();
   }
}

int x_CopyDescriptors(int index, bp::object a)
{
   PyObject* pobj = a.ptr();
   int np = PyArray_DIM(pobj, 0);
   // TODO: PyArrayObject* pcont = PyArray_ContiguousFromObject(pobj, PyArray_FLOAT, 2, 2);
   //   if (pcont != NULL)
   SiftMatchGPU::SetDescriptors(index, np, (float*) PyArray_DATA(pobj), -1);
   // Py_DECREF(pcont)

   return np; // count of copied elements
}

bp::object x_GetSiftMatch(bp::object a, bp::object b)
{
   // check a and b: ndarray, and maybe slice, list, tuple ... huh!
   x_CheckArray(1, a);
   x_CheckArray(2, b);

   // call SetDescriptors for a/0 and b/1
   int na = x_CopyDescriptors(0, a);
   int nb = x_CopyDescriptors(1, b);

   std::cout << "Will Match Features: " << na << ":" << nb << std::endl;
   int match_buf[4096][2];
   int nmatch = SiftMatchGPU::GetSiftMatch(4096, match_buf, 1024, 0.8, 0);
   std::cout << "Matches found: " << nmatch << std::endl;

   // call GetSiftMatch
   // return ndarray or maybe a list of tuples
   int type = PyArray_INT;
   if (sizeof(int) == 2) type = PyArray_SHORT;

   npy_intp dims[2] = { nmatch, 2 };
   bp::object mtch(bp::handle<> (PyArray_SimpleNew(2, dims, type)));
   void *ftrs_data = PyArray_DATA((PyArrayObject*) mtch.ptr());
   memcpy(ftrs_data, match_buf, sizeof(match_buf[0]) * nmatch);

   return mtch;
}

