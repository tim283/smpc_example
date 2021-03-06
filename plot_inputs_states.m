% plot inputs and states 

%% run smpc (runs new MPC simulation)
[x,u, x1_limit, sig, beta, s, comp_time]= run_mpc;



%% plot input and states

% set up figure for input and states
figure(2)
clf

plot_noise = 1;         % plot noise? 0: no; 1: yes (if yes: computation time will also be plotted)
% if noise was not saved, automatically swich to 0
if exist('s','var') == 0
    plot_noise = 0;
end


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
% plot constraint only if close enough
if x1_limit < 40
    yline(x1_limit, 'r', 'Linewidth',0.8)
    ylim([-0.5 x1_limit+0.5]);
    gamma1 = sqrt(2*[1;0]'*[sig^2 0; 0 sig^2]*[1;0])*erfinv(2*beta-1); % chance constraint addition for first predicted step
    yline(x1_limit-gamma1, 'r--', 'Linewidth',0.8)
end
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
% rng(30,'twister'); % hardcoded seeding
rng(s);             % retrieve seeding from run_mpc

w = [];

for i = 1: length(x)
    w(i,1) = normrnd(0,sig);
    w(i,2) = normrnd(0,sig);
end

% plot subplots with noise
if plot_noise == 1
    
    w_max = max(max(abs(w))); % get largest uncertainty  
        
    
    subplot(3,2,2)
    hold on
    title('noise - w1')
    plot(steps, w(:,1), 'b', 'Linewidth',0.8)
    % plot standard deviation
    yline(sig,'r:', 'Linewidth',0.8)
    yline(-sig,'r:', 'Linewidth',0.8)
    if w_max > 0
        ylim([-1.2*w_max 1.2*w_max])
    end
    grid on
    hold off
    
    subplot(3,2,4)
    hold on
    title('noise - w2')
    plot(steps, w(:,2), 'b', 'Linewidth',0.8)
    % plot standard deviation
    yline(sig,'r:', 'Linewidth',0.8)
    yline(-sig,'r:', 'Linewidth',0.8)
    if w_max > 0
        ylim([-1.2*w_max 1.2*w_max])
    end
    grid on
    hold off
    
    % plot noise in state plots
%     subplot(3,2,1)
%     hold on 
%     plot(steps, w(:,1), 'b--', 'Linewidth',0.8)    
%     subplot(3,2,3)
%     hold on 
%     plot(steps, w(:,2), 'b--', 'Linewidth',0.8)

    subplot(3,2,6)
    hold on
    title('computation time (s)')
    plot(steps, comp_time, 'b', 'Linewidth',0.8)
    grid on
    hold off
    
end


