#! /usr/bin/env python
# -*- coding: latin-1 -*-


"""MAPSIM: a Multiagent Planning simulation environment.

For usage information call with -h or --help.
"""
    
import config

if __name__ == "__main__":
    import main
    try:
        main.main()    
    except SystemExit, e:
        if config.verbosity > 0:
            raise e
    except:
        if config.verbosity > 0:
            print "A serious error occurred!  To enforce debugging MAPSIM will NOT die gracefully!"
            print
        raise

