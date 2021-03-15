function [x, u, x1_limit, sig, beta, s, comp_time] = run_mpc(varargin)
% (S)MPC setup by Tim Bruedigam
% based on the nonlinear MPC routine by Gruene and Pannek (details: http://numerik.mathematik.uni-bayreuth.de/~lgruene/nmpc-book/matlab_nmpc.html)
% example system based on Lorenzen et al. 2017: Constraint-Tightening and Stability in Stochastic Model Predictive Control

% function use:
% - standard: run_mpc()    with standard parameters as defined below
% - specific: run_mpc(N, beta, sigma, x1_limit, [x_initial])    with specific parameters

    clearvars -except varargin;
    close all;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%% make changes here %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    % MPC settings
    mpciterations = 20;             % simulation steps
    N             = 11;             % prediction steps (horizon)
    T             = 0.1;            % samling time
    xmeasure      = [2.5, 4.8];     % initial state
    u0            = 0.2*ones(1,N);  % initial input guess
%     u0            = [0.2*ones(1,N); 0.001*ones(1,N)];  % only if slack variable is needed (change also: cost-function, constraints)

    plot_pause    = 0.2;            % pause between updating plot after each step


    % choose MPC design    
    mpc_mode = 4; % MPC design (specifics below, e.g., variance, risk parameter)
    % 0) no MPC, no constraint, no uncertainty (MPC input set to 0; only stabilizing feedback matrix K)
    % 1) MPC,  no constraint, no uncertainty
    % 2) MPC,  x1-constraint, no uncertainty
    % 3) MPC,  x1-constraint, uncertainty
    % 4) SMPC, x1-constraint, uncertainty
    
    switch mpc_mode
        
        case 0  % no MPC (MPC input set to 0),  no constraint, no uncertainty
            sig = 0.00;             % sigma of Gaussian distribution -> covariance matrix with sigma^2 (here: no uncertainty)
            beta = 0.50;            % smpc risk parameter, [0.5 to 0.999] (here: 0.50 means no SMPC constraint tightening)
            x1_limit = 50;         % limit for x1 - (chance) constraint (here: inactive limit, too far to the right)
        
        case 1  % MPC,  no constraint, no uncertainty
            sig = 0.00;             % sigma of Gaussian distribution -> covariance matrix with sigma^2 (here: no uncertainty)
            beta = 0.50;            % smpc risk parameter, [0.5 to 0.999] (here: 0.50 means no SMPC constraint tightening)
            x1_limit = 50;         % limit for x1 - (chance) constraint (here: inactive limit, too far to the right)
            
        case 2  % MPC,  x1-constraint, no uncertainty  
            sig = 0.00;             % sigma of Gaussian distribution -> covariance matrix with sigma^2 (here: no uncertainty)
            beta = 0.50;            % smpc risk parameter, [0.5 to 0.999] (here: 0.50 means no SMPC constraint tightening)
            x1_limit = 2.8;         % limit for x1 - (chance) constraint
            
        case 3  % MPC,  x1-constraint, uncertainty
            sig = 0.08;             % sigma of Gaussian distribution -> covariance matrix with sigma^2 (here: uncertainty considered)
            beta = 0.50;            % smpc risk parameter, [0.5 to 0.999] (here: 0.50 means no SMPC constraint tightening)
            x1_limit = 2.8;         % limit for x1 - (chance) constraint
            
        case 4  % SMPC, x1-constraint, uncertainty
            sig = 0.08;             % sigma of Gaussian distribution -> covariance matrix with sigma^2 (here: uncertainty considered)
            beta = 0.90;            % smpc risk parameter, [0.5 to 0.999] (here: high beta means low risk)
            x1_limit = 2.8;         % limit for x1 - (chance) constraint
            
    end
            
    
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%% no more changes necessary in the following %%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % use function arguments if available (for monte carlo simulations)
    % !! overwrites previous settings !!
    % argument order: N, beta, sig, x1_limit, [x_initial], u_bound
    if (nargin>=1)
        N = varargin{1};
    end
    if (nargin>=2)
        beta = varargin{2};
    end
    if (nargin>=3)
        sig = varargin{3};
    end
    if (nargin>=4)
        x1_limit = varargin{4};
    end
    if (nargin>=5)
        pseudo_xmeasure = varargin{5};
        xmeasure(1) = pseudo_xmeasure(1);
        xmeasure(2) = pseudo_xmeasure(2);
    end
    if (nargin>=6)
        % input limitation, not available yet
        disp('input limitation argument not available yet')
    end
    if (nargin>=7)
        disp('argument number limit exceeded')
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    

    % optimizatoin options
    tol_opt       = 1e-8;
    opt_option    = 0;
    iprint        = 10;             % see nmpc.m line 81+
    type          = 'difference equation';
    atol_ode_real = 1e-12;
    rtol_ode_real = 1e-12;
    atol_ode_sim  = 1e-4;
    rtol_ode_sim  = 1e-4;
        
    tmeasure      = 0.0;            % initial time (do not change)
        
    params = [x1_limit, plot_pause, mpc_mode]; % parameters necessary for optimal control problem
    
    % random number generator settings
%     rng(30,'twister');               % seed selection: change to get different noise
    rng('shuffle');                 % random seed
    s = rng;                        % save rng setting
    
    % run mpc
    [t,x,u,comp_time] = nmpc(@runningcosts, @terminalcosts, @constraints, ...
         @terminalconstraints, @linearconstraints, @system, @cov_propagation, ...
         mpciterations, N, T, tmeasure, xmeasure, u0, ...
         sig, beta, params, ...
         tol_opt, opt_option, ...
         type, atol_ode_real, rtol_ode_real, atol_ode_sim, rtol_ode_sim, ...
         iprint, @printHeader, @printClosedloopData, @plotTrajectories);

    
end







%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Definition of the NMPC functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function cost = runningcosts(t, x, u)
    
    Q = [1 0 ; 0 10];
    R = 1;
    
    cost = x*Q*x' + u(1)*R*u(1)';
    
    % next two lines only necessary for slack variable
%     lambda = 5000;  
%     cost = x*Q*x' + u(1)*R*u(1)' + lambda*u(2);
end

function cost = terminalcosts(t, x)
    cost = 0.0;
end

function [c,ceq] = constraints(t, x, u, gamma, K, params)

    x1_limit = params(1);                       % get x1 constraint
    
    c   = [];
    
    % input limitations (after input decomposition)
    c(end+1) = (u(1) - K*[x(1); x(2)]) - 0.2;
    c(end+1) = -(u(1) - K*[x(1); x(2)]) - 0.2;
    
    % state constraint (comment out next line for no state constraint)
    c(end+1) = x(1) -x1_limit + gamma;            % g'*x-2.8 = [1 0]*[x(1);x(2)]-2.8
    
    % following two lines only for slack variable
%     c(end+1) = x(1) -x1_limit + gamma - u(2);
%     c(end+1) = -u(2);


    ceq = [];
end

function [c,ceq] = terminalconstraints(t, x)
    c   = [];
    ceq = [];
end

function [A, b, Aeq, beq, lb, ub] = linearconstraints(t, x, u)
    A   = [];
    b   = [];
    Aeq = [];
    beq = [];
    lb = [];
    ub = [];
end



function y = system(t, x, u, T, apply_flag, sig, params)
    % apply_flag: 1) noise is applied for real system; 0) no noise for prediction

    % system matrices
    A = [1 0.0075; -0.143 0.996];
    B = [4.798; 0.115];
    
    % stabilizing feedback matrix (computation shown as comments)
%     Q = [1 0; 0 10];
%     R = 1;
%     K = dlqr(A,B,Q,R);
    K = [0.2858 -0.4910];
    
    % for mpc_mode=0, set u(1,1)=0 (only stabilizing feedback matrix K)
    if params(3) == 0 % (mpc_mode==0)
        u(1,1) = 0;
    end
    
    % determine next (prediction step) state
    y = A*x'+B*(u(1,1) - K*[x(1); x(2)]);
    
    % noise only for application
    if apply_flag == 1
        D = [1 0; 0 1];
        w = [0;0];
        % Gaussian noise with variance sig^2
        w(1) = normrnd(0,sig);
        w(2) = normrnd(0,sig);
        % determine next state with uncertainty
        y = y + D*w;
    end
            
    y = y';
    
end


function sigma_e = cov_propagation(N, sig)
    % covariance matrix propagation

    w_cov = [sig^2 0; 0 sig^2];
    
    A = [1 0.0075; -0.143 0.996];
    B = [4.798; 0.115];

    D = [1 0; 0 1];

    K = [0.2858 -0.4910];
    phi = A-B*K;

    % initial covariance matrix
    sigma_e = zeros(2,2,N);
    % sigma_e(:,:,1) is 0 matrix, i.e., '1' refers to current state, which is known

    % updated covariance matrix for each step
    for i = 2:N
        sigma_e(:,:,i) = phi*sigma_e(:,:,i-1)*phi' + D*w_cov*D';
    end
    
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Definition of output format
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function printHeader()
    fprintf('   k  |       u(k)         x(1)        x(2)       Time   | Solver messages\n');
    fprintf('--------------------------------------------------------------------------\n');
end

function printClosedloopData(mpciter, u, x, t_Elapsed)
    fprintf(' %3d  | %+11.3f %+11.2f %+11.2f      %+6.3f  |', mpciter, ...
            u(1,1) - [0.2858 -0.4910]*[x(1); x(2)], x(1), x(2), t_Elapsed);
end

function plotTrajectories(dynamic, system, T, t0, x0, u, ...
                          atol_ode, rtol_ode, type, params)
                      
                     
                      
    [x, t_intermediate, x_intermediate] = dynamic(system, T, t0, ...
                                          x0, u, atol_ode, rtol_ode, type, 0, 0, params);
                                      
    % set up figure
    figure(1);
        title('x_1/x_2 trajectory');
        xlabel('x_1');
        ylabel('x_2');
%         title('$x_1$/$x_2$ trajectory','Interpreter','Latex');
%         xlabel('$x_1$','Interpreter','Latex');
%         ylabel('$x_2$','Interpreter','Latex');
        grid on;
        hold on;
        
        % initial plot details
        if t0 == 0
            % x1 - limit
            x1_limit = params(1);
            h1=xline(x1_limit);
            h2=fill([x1_limit x1_limit x1_limit+100 x1_limit+100],[-1000 1000 1000 -1000],'black','FaceColor',[0 0 0],'FaceAlpha',0.1,'LineStyle','none');
            set(get(get(h1,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
            set(get(get(h2,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
            
            % start
            h3 = plot(x_intermediate(1,1),x_intermediate(1,2),'ob','MarkerSize',10, 'linewidth',0.6);
            set(get(get(h3,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
            % target point
            h4 = plot(0,0,'kx','MarkerSize',12, 'linewidth',2);
            set(get(get(h4,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
        end

        % trajectory: 
        % - blue: current real position
        % - red: planned next position with optimized input
        % - orange: planned trajectory
        hc = plot(x_intermediate(:,1),x_intermediate(:,2),'r:','Linewidth',0.6);         % connection current state to next predicted state
        set(get(get(hc,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
        hb = plot(x_intermediate(1,1),x_intermediate(1,2),'ob', ...     % current state
             'MarkerFaceColor','b','MarkerSize',6);
        hr = plot(x_intermediate(2,1),x_intermediate(2,2),'or', ...     % next predicted state
             'MarkerFaceColor','r');
        axis([-2.5 6.0 -2 6.5]);
        axis square;
%         axis equal;
        if params(3) == 0 % (no MPC, mpc_mode==0) 
            axis([-2.5 12.0 -2 6.5]); % if u==0 --> system moves farther in x1-direction
            axis equal;
        end
        
        % mark positions with constraint violation
        if x_intermediate(1,1) > params(1)+0.000001  % account for numerical uncertainty
            he = plot(x_intermediate(1,1),x_intermediate(1,2),'ow','MarkerSize',3, 'MarkerFaceColor','w');
            set(get(get(he,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
        end
                
               
        % show prediction
        x_pred = zeros(length(u), length(x));
        x_pred(1,:) = x0;        
        for i = 2:length(u)
            x_pred(i,:) = system(0, x_pred(i-1,:), u(1,i-1), T, 0, 0, params);
        end        
        hm = plot(x_pred(:,1),x_pred(:,2),'m*-','Linewidth',0.5, 'Markersize', 7, 'Color', [1,0.4,0]);
        
        % legend (only once)
        if t0 == 0
            if params(3) == 0 % (no MPC, mpc_mode==0) 
                legend([hb,hr,hm],{'real state','next predicted step','prediction'},'Location','southeast', 'FontSize',10,'AutoUpdate','off')
            else
                legend([hb,hr,hm],{'real state','next predicted step','prediction'},'Location','northwest', 'FontSize',10,'AutoUpdate','off')
            end
        end
        
        % pause if necessary
        pause(params(2))
        
        
        % delete predicted inputs
        children = get(gca, 'children');
        delete(children(1));
        


end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
