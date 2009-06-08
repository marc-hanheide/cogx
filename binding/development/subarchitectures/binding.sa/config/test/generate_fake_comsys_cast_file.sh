#!/bin/bash

mkdir -p ../../../../binding_test/dotfiles_fake_comsys_$1

echo "HOST localhost"
echo ""
echo "SUBARCHITECTURE binding.sa localhost"
echo "CPP  WM BindingWorkingMemory"
echo "CPP  TM AlwaysPositiveTaskManager"
echo "CPP  GD binding.scorer BindingScorer -nth 0 -maxN 1"
echo "CPP  GD binding.judge BindingJudge "
echo "CPP  GD binding.binder Binder "
echo "CPP  GD binding.groups BindingGroupManager"
echo "CPP  GD binding.state BindingStatusMonitor --log"
echo "CPP  GD binding.dot BindingDotViewer -visfile vis_binding.dot -d binding_test/dotfiles_fake_comsys_$1 --log --proxy short --union short "
echo "CPP  GD binding.garbage GarbageCollector --log --police-mode strict"
echo "CPP  GD binding.ambiguity AmbiguityIdentifier --log"
echo "JAVA GD binding.fake_comsys binding.monitors.test.FakeComSysTest --test $1"
