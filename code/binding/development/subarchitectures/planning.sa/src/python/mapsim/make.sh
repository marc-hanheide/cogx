# only the two C programs need to be made
make -j2 -C planning/plan_monitor/VAL/
make -j2 -C planning/planner_base/ContinualAxff/
# make sure the latest Planner.idl is used
echo "Updating IDL stubs."
cd planning
omniidl -bpython Planner.idl
cd ..