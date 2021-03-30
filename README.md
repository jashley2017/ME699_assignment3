# Assignment 3: Inverse Dynamics Control of the Panda 7-DoF Manipulator
## Author: Josh Ashley

### How to Run
* This script is designed to be run in a julia environment generated from [RMC-21](https://github.com/hpoonawala/rmc-s21/tree/master/julia/odes).
* Copy this script into the odes folder of that repository and from there run these commands.

```
julia
]activate . 
include("startup.jl")
include("JA_solution.jl")
```

* From there you can now view the trajectory from http://localhost:8700/
* Additionally, the console will display the resulting final joint angles and the error from the goal point.
* To test the 'traj' function, after including the solution you can now run 'traj(Float64)' in the julia prompt and a set of joint angles will result.

### Adjusting the script
* On line 127, you can change between using CTC and PD by adjusting the function call.
* The goal points are set manually in the control_{CTC,PD} and traj functions as variable q_des.
* The timescale is set in move_robot_{CTC,PD} and traj.
