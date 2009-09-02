#!/bin/bash

make clean
make -j 8

strip ayPlan

echo "1"

time ./ayPlan --max-cost 100 --problem ./DOMAINS/pipesworld/p01.pddl --domain ./DOMAINS/pipesworld/domain.pddl < /dev/null 2> /dev/null | grep "WE FOUND PLAN"

echo "2"

time ./ayPlan  --problem ./DOMAINS/pipesworld/p01.pddl --domain ./DOMAINS/pipesworld/domain.pddl < /dev/null 2> /dev/null | grep "WE FOUND PLAN"

echo "3"

time ./ayPlan --max-cost 100 --problem ./DOMAINS/blocksworld/propositional/problem.pddl --domain ./DOMAINS/blocksworld/propositional/domain.pddl < /dev/null 2> /dev/null | grep "WE FOUND PLAN"

echo "4"

time ./ayPlan  --problem ./DOMAINS/blocksworld/propositional/problem.pddl --domain ./DOMAINS/blocksworld/propositional/domain.pddl < /dev/null 2> /dev/null | grep "WE FOUND PLAN"


echo "5"

time ./ayPlan --max-cost 100 --problem ./DOMAINS/rovers/propositional/problem.pddl --domain ./DOMAINS/rovers/propositional/domain.pddl < /dev/null 2> /dev/null | grep "WE FOUND PLAN"

echo "6"

time ./ayPlan  --problem ./DOMAINS/rovers/propositional/problem.pddl --domain ./DOMAINS/rovers/propositional/domain.pddl < /dev/null 2> /dev/null | grep "WE FOUND PLAN"

echo "7"

time ./ayPlan --max-cost 100 --problem ./DOMAINS/logistics/propositional/problem.pddl --domain ./DOMAINS/logistics/propositional/domain.pddl < /dev/null 2> /dev/null | grep "WE FOUND PLAN"

echo "8"

time ./ayPlan  --problem ./DOMAINS/logistics/propositional/problem.pddl --domain ./DOMAINS/logistics/propositional/domain.pddl < /dev/null 2> /dev/null | grep "WE FOUND PLAN"

echo "9"


time ./ayPlan --max-cost 100 --problem ./DOMAINS/storage/propositional-action-costs/problem.pddl --domain ./DOMAINS/storage/propositional-action-costs/domain.pddl < /dev/null 2> /dev/null | grep "WE FOUND PLAN"


echo "10"

time ./ayPlan  --problem ./DOMAINS/storage/propositional-action-costs/problem.pddl --domain ./DOMAINS/storage/propositional-action-costs/domain.pddl < /dev/null 2> /dev/null | grep "WE FOUND PLAN"

echo "11"

time ./ayPlan --max-cost 100 --problem ./DOMAINS/satellite/propositional/problem.pddl --domain ./DOMAINS/satellite/propositional/domain.pddl < /dev/null 2> /dev/null | grep "WE FOUND PLAN"

echo "12"

time ./ayPlan  --problem ./DOMAINS/satellite/propositional/problem.pddl --domain ./DOMAINS/satellite/propositional/domain.pddl < /dev/null 2> /dev/null | grep "WE FOUND PLAN"

 echo "13"

time ./ayPlan --max-cost 9 --problem ./DOMAINS/pipesworld/p05.pddl --domain ./DOMAINS/pipesworld/domain.pddl < /dev/null 2> /dev/null | grep "WE FOUND PLAN"

echo "14"

time ./ayPlan  --problem ./DOMAINS/pipesworld/p05.pddl --domain ./DOMAINS/pipesworld/domain.pddl < /dev/null 2> /dev/null | grep "WE FOUND PLAN"

echo "15"

time ./ayPlan --max-cost 1000 --problem ./DOMAINS/zenotravel/propositional/problem.pddl --domain ./DOMAINS/zenotravel/propositional/domain.pddl  < /dev/null 2> /dev/null | grep "WE FOUND PLAN"
