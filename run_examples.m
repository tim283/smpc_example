% this script offers a simple way to run 5 different (S)MPC examples

%% choose MPC mode

mpc_mode = 4;

% 0) no MPC, no constraint, no uncertainty (MPC input set to 0; only stabilizing feedback matrix K)
% 1) MPC,  no constraint, no uncertainty
% 2) MPC,  x1-constraint, no uncertainty    (x1<=2.8)
% 3) MPC,  x1-constraint, uncertainty       (x1<=2.8; sigma=0.08)
% 4) SMPC, x1-constraint, uncertainty       (x1<=2.8; sigma=0.08; beta=0.9)


%% run MPC code

run_mpc(mpc_mode);
