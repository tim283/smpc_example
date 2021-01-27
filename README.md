# smpc_example

This (S)MPC example consists of 3 matlab files:
- run_mpc.m: runs the (S)MPC example
- plot_inputs_states.m: runs the MPC example and plots states and the input
- nmpc.m: nonlinear MPC routine developed by Gruene/Pannek (details: http://numerik.mathematik.uni-bayreuth.de/~lgruene/nmpc-book/matlab_nmpc.html)

SMPC_introduction.pdf serves as a brief introduction to the example and (S)MPC with probabilistic constraints (chance constraints).

run_mpc.m allows simple changes to the (SMPC) algorithm (see lines 8 - 52). 
The following options exist:
1) MPC,  no uncertainty, no constraint
2) MPC,  no uncertainty, constraint
3) MPC,  uncertainty,    constraint
4) SMPC, uncertainty,    (chance) constraint, 

Constraint tightening is computed in nmpc.m (see lines 428 - 442)


If you find mistakes or have suggestions, feel free to contribute!
