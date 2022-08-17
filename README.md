# MO-CBS with cost splitting
This repo contains the Python implementation of algorithms (MO-CBS with cost splitting, disjoint cost splitting) related to a multi-objective multi-agent path finding (MO-MAPF) problem.

We have implemented  MO-CBS with standard splitting, cost splitting and disjoint cost splitting based on this [repo](https://github.com/wonderren/public_pymomapf) and realized several optimizations. 

Script run_single.sh can be served as an entry point to the planners and contains all arguments that have included in this repo

 ./momapf library contains our detailed implementation. See Dom_Checker.ipynb and Cost_Bound.ipynb to get port to our dominance checker and cost bound.

All test instances are constained in ./benchmark. To test more instances, you can go to [this link](https://movingai.com/benchmarks/mapf/index.html) and download to ./benchmark 

Below shows the code structure.

```
│  data_process.py
│  README.md
│  run_example.py
│  run_single.sh
│
├─benchmark
│  ├─empty-16-16
│  ├─empty-16-16-result
│  ├─maze-32-32-2
│  ├─maze-32-32-2-result
│  ├─random-32-32-20
│  ├─random-32-32-20-result
│  ├─room-32-32-4
│  └─room-32-32-4-result
│
├─momapf
│  │  arguments.py
│  │  common.py
│  │  Cost_Bound.ipynb
│  │  Dom_Checker.ipynb
│  │  ll_solver.py
│  │  mocbs_new.py
│  │  utils.py
│  │  __init__.py
```