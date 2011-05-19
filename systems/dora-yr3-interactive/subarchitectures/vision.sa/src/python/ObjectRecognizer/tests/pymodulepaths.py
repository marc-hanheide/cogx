#!/usr/bin/env python
# vim:set fileencoding=utf-8 sw=4 ts=8 et:vim
import os, sys
path_to_cogx_root = "../" * 6 
sys.path.insert(0, os.path.abspath(os.path.join(path_to_cogx_root, "output/python")))
path_to_object_recognizer = "../" * 2 # dev-path that contains ObjectRecognizer dir
sys.path.insert(0, os.path.abspath(path_to_object_recognizer))

