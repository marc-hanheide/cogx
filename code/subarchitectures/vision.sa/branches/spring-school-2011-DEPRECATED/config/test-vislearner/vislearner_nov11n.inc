# File: vislearner_nov11n.inc
# vim: ft=conf sw=3

VARDEFAULT visl_clfConfig = subarchitectures/vision.sa/config/test-vislearner/cmLearn.config

CPP MG vis.learner  VisualLearner --clfconfig "%(visl_clfConfig)" --log # --debug
