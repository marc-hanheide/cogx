#!/bin/bash

mkdir -p ../../../../binding_test/dotfiles_many_monitors_$1


echo "HOST localhost"
echo ""
echo "SUBARCHITECTURE binding.sa localhost"
echo "CPP  WM BindingWorkingMemory"
echo "#JAVA WM binding.test.SimpleWM --log false"
echo "CPP  TM AlwaysPositiveTaskManager"
echo "CPP  GD binding.scorer BindingScorer -nth 0 -maxN 1"
echo "CPP  GD binding.judge BindingJudge "
echo "CPP  GD binding.binder Binder "
echo "CPP  GD binding.groups BindingGroupManager"
echo "CPP  GD binding.state BindingStatusMonitor --log true"
echo "CPP  GD binding.dot BindingDotViewer -d binding_test/dotfiles_many_monitors_$1 --log --proxy short --union short "
echo "CPP  GD binding.ambiguity AmbiguityIdentifier --log"
echo "CPP  GD binding.garbage GarbageCollector --log --police-mode strict"
echo "CPP  GD binding.tester0 MultiTesterMonitor --log --test $1 -bsa binding.sa --instance 0 --noOfMultiTesters $2 --postConditionTester"
for ((a=1; a < $2 ; a++))
do
echo ""
echo "SUBARCHITECTURE another$a.sa localhost"
echo "CPP  WM BindingWorkingMemory"
echo "CPP  TM AlwaysPositiveTaskManager"
echo "CPP  GD binding.tester$a MultiTesterMonitor --log --test $1 -bsa binding.sa --instance $a --noOfMultiTesters $2"
done
