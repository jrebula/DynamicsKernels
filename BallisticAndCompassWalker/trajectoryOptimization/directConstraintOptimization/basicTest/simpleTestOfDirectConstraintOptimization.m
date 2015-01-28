%% simple test of: 
% Posa, 2013ish, "A Direct Method for Trajectory Optimization of Rigid Bodies Through Contact"

% system is a pair of rotating pendula, both of length 1, with pivot points
% connected to ground, with one pivot point 1.5 higher than the other.
% There's gravity. The endpoint of the bottom pendulum can inelastically 
% contact the top pendulum anywhere along the top pendulum. The goal is to
% get the top pendulum to be vertically upright. There is a torque source
% at the base of the bottom pendulum.

% optimization state has, at each point in time:
% - theta1, theta1Dot, theta2, theta2Dot
% - normalForce between pin and top pendulum

rng(0);

format compact;
numStatesInTrajectory = 11;
dimensionOfEachTrajectoryPoint = 5;

addpath(genpath('C:\Users\john\workspace\HBCL Gait Analysis Toolbox v1.3'));
% addpath(genpath('C:\Users\hbclStudent\workspace\dynamicWalking\Branches\jrebula\ThreeDWalker'));
% currentTrajectory = zeros(dimensionOfEachTrajectoryPoint, numStatesInTrajectory);

%%
trajectoryState = TrajectoryState(numStatesInTrajectory);

system = BasicTestSystem();
system.getConstraintMatrices(trajectoryState);

lowerBounds = trajectoryState;
lowerBounds.stateTrajectory = ones(size(lowerBounds.stateTrajectory)) * -Inf;
lowerBounds.inputTape = ones(size(lowerBounds.inputTape)) * -Inf;
lowerBounds.constraintForceTrajectory = ones(size(lowerBounds.constraintForceTrajectory)) * -Inf;
lowerBounds.subtickRatios = ones(size(lowerBounds.subtickRatios)) * 0;

upperBounds = trajectoryState;
upperBounds.stateTrajectory = ones(size(upperBounds.stateTrajectory)) * Inf;
upperBounds.inputTape = ones(size(upperBounds.inputTape)) * Inf;
upperBounds.constraintForceTrajectory = ones(size(upperBounds.constraintForceTrajectory)) * Inf;
upperBounds.subtickRatios = ones(size(upperBounds.subtickRatios)) * 1;

A = [];
b = [];
AEq = [];
bEq = [];
lb = lowerBounds.getVector();
ub = upperBounds.getVector();
errorFunction = @(x) system.errorFunction(trajectoryState.setFromVector(x));
nonlcon = @(x) system.getConstraintMatrices(trajectoryState.setFromVector(x));
options = optimset('algorithm', 'sqp', 'display', 'iter', 'MaxFunEvals', 1e6);
% options = optimset('algorithm', 'interior-point', 'display', 'iter', ...
%   'MaxFunEvals', 1e6, 'scaleproblem', 'obj-and-constr');

x0 = trajectoryState.getVector();
x0 = rand(size(x0)) * 0.1;
[c, ce] = system.getConstraintMatrices(trajectoryState.setFromVector(x0));

xFinal = x0;

% gaOptions = gaoptimset('display', 'iter', 'StallGenLimit', 5);
% xFinal = ga(errorFunction, length(x0), A, b, AEq, bEq, lb, ub, nonlcon, gaOptions);


% xFinal = fmincon(@(x) 0, x0, A, b, AEq, bEq, lb, ub, nonlcon, options);


for i = 1 : 20
  xFinal = fmincon(errorFunction, xFinal, A, b, AEq, bEq, lb, ub, nonlcon, options);
end


%%
trajectoryState = trajectoryState.setFromVector(xFinal);
fprintf('inputs:\n');
trajectoryState.inputTape
fprintf('states:\n');
trajectoryState.stateTrajectory
fprintf('subtick ratios:\n');
trajectoryState.subtickRatios
fprintf('constraint forces:\n');
trajectoryState.constraintForceTrajectory

system.plotStateTrajectory(trajectoryState);
system.plotConstraints(trajectoryState)


%%
nonPen = system.getNonPenetrationConstraintMatrix(trajectoryState.stateTrajectory);
fprintf('non penetrating constraint: %s\n', sprintf('%g, ', reshape(nonPen, [1 numel(nonPen)])));

