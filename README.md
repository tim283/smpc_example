# smpc_example

**If you just want to quickly run (S)MPC examples, use 'run_examples.m' and select an MPC mode.**


This stochastic Model Predictive Control (SMPC) example consists of 4 matlab files:
- run_mpc.m: runs the (S)MPC example (run script or use command "run_mpc()")
- run_examples.m: simple script with one variable to run different (S)MPC examples
- plot_inputs_states.m: runs the (S)MPC example and plots states and the input
- nmpc.m: nonlinear MPC routine developed by Gruene/Pannek [1] (details: http://numerik.mathematik.uni-bayreuth.de/~lgruene/nmpc-book/matlab_nmpc.html)

SMPC_introduction.pdf [2] serves as a brief introduction to the example and SMPC with probabilistic constraints (chance constraints).


run_mpc.m allows to run predefined MPC modes or to make simple changes to the (SMPC) algorithm (see lines 13 - 68). 

The following predefined options exist:

0) no MPC, no constraint, no uncertainty (MPC input set to 0; only stabilizing feedback matrix K)
1) MPC without constraint;  no uncertainty
2) MPC with constraint;  no uncertainty
3) MPC with constraint;  uncertainty
4) SMPC with (chance) constraint; uncertainty

If desired, specific parameters can be passed to run_mpc.m as arguments (see line 8).

Constraint tightening is computed in nmpc.m (see lines 430 - 452).


If you find mistakes or have suggestions, feel free to contribute!


---
[1] L. Grüne and J. Pannek. Nonlinear Model Predictive Control. Springer-Verlag, London, 2017.

[2] T. Brüdigam. (Stochastic) Model Predictive Control - a Simulation Example. arXiv:2101.12020, 2021.
