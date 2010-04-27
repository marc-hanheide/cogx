#!/usr/bin/python
# vim: fileencoding=utf-8 sw=4 ts=8 et:vim
# Author:  Marko Mahnič
# Created: jan 2009

import os, sys
import optparse
from string import Template
import pyplusplus as pypp
from pyplusplus import function_transformers as FT
import pygccxml.declarations.matchers as gcmatch
# basedir=r"."

def parseOptions():
    usage = "Usage: %prog [options] args"
    parser = optparse.OptionParser(usage)

    parser.add_option("-v", "--verbose", action="store", type="int", dest="verbose")
    parser.add_option("-q", "--quiet", action="store_const", const=0, dest="verbose")
    parser.add_option("-b", "--basedir", action="store", type="string", default=".", dest="basedir")
    parser.add_option("-s", "--siftgpudir", action="store", type="string", default="libsiftgpu", dest="siftgpudir")
    parser.add_option("-p", "--pyinclude", action="store", type="string", default="", dest="pyinclude")

    (options, args) = parser.parse_args()
    if options.pyinclude == "": options.pyinclude = "/usr/include/python2.6"
    # if options.verbose > 3: print "Options parsed"
    # if len(args) != 1: parser.error("incorrect number of arguments")
    return (options, args)

opts, args = parseOptions()

#Creating an instance of class that will help to expose the declarations
mb = pypp.module_builder.module_builder_t(
    [r"SiftGPU.h"]
    , gccxml_path=r"" 
    , working_directory=opts.basedir
    , include_paths=[
        r'/usr/include/',
        opts.pyinclude,
        opts.basedir,
        r'%s/SiftGPU/src'% (opts.siftgpudir)
        ]
    , define_symbols=[]
    , encoding="utf-8")
# Exclude everything (all was included by default)
mb.decls(gcmatch.regex_matcher_t( '.*' ) ).exclude()

# mb.add_declaration_code(
#     """
#     """)
mb.add_registration_code(
    """
    import_array();
    bp::numeric::array::set_module_and_type("numpy", "ndarray"); 
    """)

def fileContent(fn):
    cnt = "".join(open(fn, "r").readlines())
    return cnt.decode("utf-8")

# Exported Class: SiftGPU
def export_SiftGPU():
    mb.decls(gcmatch.declaration_matcher_t( 'SiftGPU' ) ).include()
    cls = mb.class_('SiftGPU')
    cls.add_wrapper_code(fileContent("wrap_sift.cpp"))
    cls.mem_funs("SetImageList").exclude()
    cls.variables("_timing").exclude()
    # alternative: 
    #    mb.calldefs(gcmatch.declaration_matcher_t( '::SiftGPU::SetImageList' ) ).exclude()
    #    mb.decls(gcmatch.declaration_matcher_t( '::SiftGPU::_timing' ) ).exclude()

    # RunSIFT: Only allow the signature (char * imagepath)
    runs = cls.mem_funs("RunSIFT")
    for f in runs:
        if len(f.required_args) != 1 or f.required_args[0].type.decl_string != "char *": f.exclude()
    # ... And add a new RunSift that takes a NumPy ndarray
    cls.add_registration_code(
        """def("RunSIFT", (int (::SiftGPU_wrapper::*)(bp::object))&::SiftGPU_wrapper::x_RunSiftOnNdarray)"""
        , works_on_instance=True )

    # Dimensions as a tuple
    get_dim = mb.mem_fun("::SiftGPU::GetImageDimension")
    get_dim.add_transformation(FT.output("w"), FT.output("h"))

    # FeatureVector: a tuple of ndarray-s
    mb.calldefs(gcmatch.declaration_matcher_t( '::SiftGPU::GetFeatureVector' ) ).exclude()
    cls.add_registration_code(
        """def("GetFeatureVector", (bp::tuple (::SiftGPU_wrapper::*)())&::SiftGPU_wrapper::x_GetFeatureVector)"""
        , works_on_instance=True )

    # SIFT parameters: python list 2 argv array
    cls.mem_funs("ParseParam").exclude()
    cls.add_registration_code(
        """def("ParseParam", (void (::SiftGPU_wrapper::*)(bp::str))&::SiftGPU_wrapper::x_ParseParam)"""
        , works_on_instance=True )

def export_SiftMatchGPU():
    mb.decls(gcmatch.declaration_matcher_t( 'SiftMatchGPU' ) ).include()
    cls = mb.class_('SiftMatchGPU')
    cls.add_wrapper_code(fileContent("wrap_siftmatch.cpp"))
    cls.mem_funs("SetDescriptors").exclude()
    cls.mem_funs("GetSiftMatch").exclude()
    cls.mem_funs("GetGuidedSiftMatch").exclude()
    cls.add_registration_code(
        """def("GetSiftMatch", (bp::object (::SiftMatchGPU_wrapper::*)(bp::object, bp::object))&::SiftMatchGPU_wrapper::x_GetSiftMatch)"""
        , works_on_instance=True )

export_SiftGPU()
export_SiftMatchGPU()

# Remove private and protected members from all exported classes
mb.decls(gcmatch.access_type_matcher_t( 'private' ) ).exclude()
mb.decls(gcmatch.access_type_matcher_t( 'protected' ) ).exclude()
mb.calldefs(gcmatch.access_type_matcher_t( 'private' ) ).exclude()
mb.calldefs(gcmatch.access_type_matcher_t( 'protected' ) ).exclude()

#Well, don't you want to see what is going on?
# mb.print_declarations()

#Creating code creator. After this step you should not modify/customize declarations.
mb.build_code_creator( module_name='__PySiftGpu' )
mb.code_creator.add_include("numpy/arrayobject.h")
mb.code_creator.add_include("gl.h")
mb.code_creator.add_include("iostream")

#Writing code to file(s).
mb.write_module( './xdata/bindings.cpp' )
