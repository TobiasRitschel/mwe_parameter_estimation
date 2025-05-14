%% Solve a basic dynamic optimization problem
% Clear command window
clc;

% Clear variables
clear all; %#ok

% Close figures
close all;

% Remove added paths
restoredefaultpath;

%% Formatting
% Font size
fs = 12;

% Line width
lw = 3;

% Marker size
ms = 12;

% Set default font size
set(groot, 'DefaultAxesFontSize',   fs);

% Set default line widths
set(groot, 'DefaultLineLineWidth',  lw);
set(groot, 'DefaultStairLineWidth', lw);
set(groot, 'DefaultStemLineWidth',  lw);

% Set default marker size
set(groot, 'DefaultLineMarkerSize', ms);

% Set default figure size
set(groot, 'DefaultFigureUnits', 'Inches', 'DefaultFigurePosition', [1, 1, 8, 6]);

% Set renderer
% (Otherwise, eps figures can become pixelated in Latex.)
% (Save as pdf if you have shaded areas in your figure, e.g., because
% you're using fill.)
set(groot, 'DefaultFigureRenderer', 'Painters');

%% Initialize
% Right-hand side function
fun = @(t, x, u, p) -p*x.^3 + u;

% Measurement function
output = @(t, x, u, p) x;

% Stage cost
stageCost = @(y, ymeas, p) (y - ymeas).^2;

%% Simulate measurements (should be replaced by true measurements)
% Random number generator seed
rng(0);

% Number of control intervals
N = 10;

% Initial condition
x0 = 0.2;

% Parameter
p = 1;

% Manipulated inputs
ubar = linspace(1, 0.5, N);

% Generate measurement noise
ebar = 0*randn(1, N+1);

% Time span
tspan = linspace(0, 10, N+1);

% Options
odeopts = odeset();

% Open-loop simulation
[T, X, idx] = openLoopSimulation(ubar, x0, tspan, fun, p, odeopts);

% Outputs
Y = output(tspan, X, ubar, p);

% Measurements (with noise)
tbar = T(idx)';
ybar = Y(idx)' + ebar;

% Create figure
figure(1);

% Visualize simulation
plot(T, X);

% Add more plots
hold on;

% Plot inputs
stairs(tspan, ubar([1:end, end]));

% Measurements
plot(tbar, ybar, 'xk');

% Stop adding plots
hold off;

% Axis limitsM
ylim([-0.1, 1.1]);

% Title
title('Simulation');

%% Optimize
% Inputs for fmincon
A       = [];
B       = [];
Aeq     = [];
Beq     = [];
nonlcon = [];

ub = 2*ones(1, 2);
lb =  zeros(1, 2);

% Options for fmincon
optopts = optimset(    ...
    'Display', 'iter', ...
    'TolFun', 1e-8);

% Initial guess of optimal control
theta0 = [0.5, 0.1];

% Use fmincon to solve the numerical optimization problem
thetaest = fmincon(@evaluateObjectiveFunction, theta0, A, B, Aeq, Beq, lb, ub, nonlcon, optopts, ...
    ybar, ubar, tspan, fun, output, stageCost, odeopts);

% True solution (for comparison)
thetatrue = [x0, p];

%% Simulate
% Estimate initial state
x0est = thetaest(1);

% Estimated parameter
pest = thetaest(2);

% Open-loop simulation
[Topt, Xopt] = openLoopSimulation(ubar, x0est, tspan, fun, pest, odeopts);

% Create figure
figure(2);

% Visualize simulation
plot(Topt, Xopt);

% Add more plots
hold on;

% Plot inputs
stairs(tspan, ubar([1:end, end]));

% Plot setpoint
plot(tbar, ybar, 'xk');

% Stop adding plots
hold off;

% Axis limits
ylim([-0.1, 1.1]);

% Title
title('Optimal control');

%% Functions
function J = evaluateObjectiveFunction(theta, ybar, ubar, tspan, fun, outp, stageCost, odeopts)
% Number of control intervals
N = size(ubar, 2);

% Initial time, state, and input
x0 = theta(1);

% Parameters
p = theta(2:end);

% Open-loop simulation
[~, X, idx] = openLoopSimulation(ubar, x0, tspan, fun, p, odeopts);

% Outputs
Y = outp(tspan, X, ubar, p);

% Measurements
y = Y(idx)';

% Initialize
J = 0;

for k = 1:N+1
    % Measurement
    ymeas = ybar(:, k);

    % Evaluate stage cost
    Phi = stageCost(y(k), ymeas);

    % Evaluate objective function
    J = J + Phi;
end
end

% function J = evaluateObjectiveFunction_DEPRECATED(theta, ybar, ubar, tspan, fun, outp, stageCost, odeopts)
% % Number of control intervals
% N = size(ubar, 2);
% 
% % Initial time, state, and input
% t0 = tspan(1);
% x0 = theta(1);
% u0 = ubar (1);
% 
% % Parameters
% p = theta(2:end);
% 
% % Measurement
% ymeas = ybar(:, 1);
% 
% % Evaluate outputs
% y0 = outp(t0, x0, u0);
% 
% % Evaluate stage cost
% Phi = stageCost(y0, ymeas);
% 
% % Initialize objective function value
% J = Phi;
% 
% for k = 1:N
%     % Manipulated inputs in current control interval
%     uk = ubar(:, k);
% 
%     % Measurement
%     ymeas = ybar(:, k+1);
% 
%     % Simulate
%     [t, x] = ode45(fun, tspan([k, k+1]), x0, odeopts, ...
%         uk, p);
% 
%     % Evaluate outputs
%     y = outp(t, x(end, :)', uk);
% 
%     % Evaluate stage cost
%     Phi = stageCost(y, ymeas);
% 
%     % Evaluate objective function
%     J = J + Phi;
% 
%     % Update initial states
%     x0 = x(end, :);
% end
% end

function [T, X, idx] = openLoopSimulation(ubar, x0, tspan, fun, p, odeopts)

% Number of control intervals
N = size(ubar, 2);

% Initialize
T   = tspan(1);
X   = x0;
idx = 1;

% Initial states
xk = x0;

for k = 1:N
    % Manipulated input in current control interval
    uk = ubar(:, k);

    % Simulate
    [t, x] = ode45(fun, tspan([k, k+1]), xk, odeopts, ...
        uk, p);

    % Update initial states
    xk = x(end, :);

    % Store results
    T   = [T;                    t ]; %#ok
    X   = [X;                    x ]; %#ok
    idx = [idx; idx(end) + numel(x)]; %#ok
end
end