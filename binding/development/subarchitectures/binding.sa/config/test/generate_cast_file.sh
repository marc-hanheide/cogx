#!/bin/bash

mkdir -p ../../../../binding_test/dotfiles_bindings_$1

echo "HOST localhost"
echo ""
echo "SUBARCHITECTURE binding.sa localhost"
echo "CPP  WM BindingWorkingMemory #--log true --debug true"
echo "#JAVA WM binding.test.SimpleWM --log true"
echo "CPP  TM AlwaysPositiveTaskManager"
echo "CPP  GD binding.scorer BindingScorer -nth 0 -maxN 1 #--log --debug"
echo "CPP  GD binding.judge BindingJudge #--log"
echo "CPP  GD binding.binder Binder #--log true "
echo "CPP  GD binding.groups BindingGroupManager #--log "
echo "CPP  GD binding.state BindingStatusMonitor --log"
echo "CPP  GD binding.dot BindingDotViewer -visfile vis_binding.dot -d binding_test/dotfiles_bindings_$1 --log --proxy short --union short "
echo "CPP  GD binding.garbage GarbageCollector --log --police-mode strict"
echo "CPP  GD binding.ambiguity AmbiguityIdentifier --log"
echo "CPP  GD binding.tester TesterMonitor --log --test $1 #--debug"

