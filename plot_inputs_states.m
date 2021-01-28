% plot inputs and states

%% run smpc (runs new MPC simulation)
[x,u, x1_limit, sig, beta, s]= run_mpc;


%% plot input and states

% set up figure for input and states
figure(2)
clf

plot_noise = 1;         % only plot noise if seed is selected in run_mpc script

steps = 0:length(x)-1;  % get steps (last input is computed but not applied) 


% state x1
if plot_noise == 0
    subplot(3,1,1)
else
    subplot(3,2,1)
end
hold on
title('state - x1')
grid on
plot(steps,x(:,1), 'b', 'Linewidth',0.8)
yline(x1_limit, 'r', 'Linewidth',0.8)
ylim([-0.5 x1_limit+0.5]);
xlim([steps(1) steps(end)]);
hold off


% state x2
if plot_noise == 0
    subplot(3,1,2)
else
    subplot(3,2,3)
end
hold on
title('state - x2')
plot(steps,x(:,2), 'b', 'Linewidth',0.8)
grid on
ylim([-0.5 5.5]);
xlim([steps(1) steps(end)]);
hold off


% input u
if plot_noise == 0
    subplot(3,1,3)
else
    subplot(3,2,5)
end
K = [0.2858 -0.4910];
u_applied = [];
for i = 1:length(u)
    u_applied(i,1) = u(i,1) - K*[x(i,1); x(i,2)];
end
hold on
title('input - u')
grid on
plot(steps,u_applied(:,1), 'b', 'Linewidth',0.8)
yline(0.2,'r', 'Linewidth',0.8)
yline(-0.2,'r', 'Linewidth',0.8)
ylim([-0.25 0.25]);
xlim([steps(1) steps(end)]);
hold off


% plot noise (given seeding)
% rng(2,'twister'); % hardcoded seeding
rng(s);             % retrieve seeding from run_mpc

w = [];

for i = 1: length(x)
    w(i,1) = normrnd(0,sig);
    w(i,2) = normrnd(0,sig);
end

% plot subplots with noise
if plot_noise == 1
    subplot(3,2,2)
    hold on
    title('noise - x1', 'Linewidth',0.8)
    plot(w(:,1), 'b')
    grid on
    hold off

    subplot(3,2,4)
    hold on
    title('noise - x2', 'Linewidth',0.8)
    plot(w(:,2), 'b')
    grid on
    hold off
end


