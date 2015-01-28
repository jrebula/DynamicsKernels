
format compact


%% Some flaggies, parameters:

showStepResponses = 0;

desiredComPosition = 1; %[]; %
finalTime = 4.0;
timeToSwitchDesiredPosition = 2.0; %[]; %
controlGainConstant = 0; %1; %

shouldKeepBSimpleConstant = 1; %0; %

initialConditionComplicatedSystem = [0.3, 0.9, 0.3, 0.4]';

% complicated system parameters:
m1 = 1;
m2 = 1;
k1 = 20;
k2 = 1.0;
b1 = 5;

% simple system parameters (these are initial sorta "desired" behaviors):
mSimple = 1; %m1 + m2;
kSimple = 1000;
bSimple = 10.0;

lqrStateCostWeighting = [1 0; 0 0.01]; % (position and velocity of simple system's mass)
% lqrStateCostWeighting = [1 0; 0 1 / mSimple];

lqrControlCostWeight = 0.0001;





%% Constants:

times = linspace(0, finalTime, 100);


%% here's the "complicated" system we want to control
% note that the state vector is the positions of the masses, followed by
% velocities... also, rest positions of springs are all 0.

A = [
  0, 0, 1, 0
  0, 0, 0, 1
  -(k2 + k1) / m1, k2 / m1, -b1 / m1, b1 / m1
  k2 / m2, -k2 / m2, 0, -b1 / m2];

B = [0; 0; 0; 1/m2];

complicatedSystem = ss(A, B, eye(4), []);

if (showStepResponses)
  figure('Name', 'Complicated System step response')
  step(complicatedSystem, times)
end


%% and here's the simple system like which we would like the complicated system to behave:

dimensionalityOfSimpleSystem = 2;

ASimple = [0, 1
  -kSimple/mSimple, -bSimple / mSimple];

BSimple = [0; 1 / mSimple];

simpleSystem = ss(ASimple, BSimple, eye(2), []);

if (showStepResponses)
  figure('Name', 'Simple System step response');
  step(simpleSystem, times);
end

controlGain = [];
if (controlGainConstant)
  N = [];
  KSimple = lqr(ASimple, BSimple, lqrStateCostWeighting, lqrControlCostWeight, N);
  controlGain = KSimple;
end

%% the center of mass jacobians...

comPositionJacobian = (1 / (m1 + m2)) * [m1 m2 0 0];
comVelocityJacobian = (1 / (m1 + m2)) * [ 0 0 m1 m2];

initialConditionSimpleSystem = [comPositionJacobian; comVelocityJacobian] * initialConditionComplicatedSystem;


%% Simulate with simple model based control each tick:

stateDot = @(t, x) complicatedSystemControlledWithSimpleStateDot(t, x, complicatedSystem, simpleSystem, desiredComPosition, comPositionJacobian, comVelocityJacobian, ...
  'lqrStateCostWeighting', lqrStateCostWeighting, ...
  'lqrControlCostWeight', lqrControlCostWeight, ...
  'timeToSwitchDesiredPosition', timeToSwitchDesiredPosition, ...
  'controlGain', controlGain);
[times, states] = ode45(stateDot, times, initialConditionComplicatedSystem);


%% plot some results:

comPositions = comPositionJacobian * states';

figure('Name', 'Complex/Simple linear system Simulation Results')
subplot(2,1,1)
plot(times, states);
hold on;
plot(times, comPositions, 'k', 'LineWidth', 3)


%% Now calculate the response of the simple system alone

simpleStateDotControlled = @(t, x) simpleStateDotWithLQRControl(t, x, simpleSystem, desiredComPosition, ...
  'lqrStateCostWeighting', lqrStateCostWeighting, ...
  'lqrControlCostWeight', lqrControlCostWeight, ...
  'timeToSwitchDesiredPosition', timeToSwitchDesiredPosition, ...
  'controlGain', controlGain);
[times, simpleStates] = ode45(simpleStateDotControlled, times, initialConditionSimpleSystem);


%% compare the simple state's com during independent simulation to the controlled complex state's com

comEstimationError = simpleStates(:, 1) - comPositions';

plot(times, simpleStates(:, 1), 'r--', 'LineWidth', 3)
subplot(2,1,2)
plot(times, comEstimationError, 'r--', 'LineWidth', 3)


%% Let's try to calculate a "better" set of simple model parameters.
% such that the simple model provides a better prediction of the complex
% model's dynamics. We do this iteratively...

figure('Name', 'Simple System Parameter Estimation com estimated and measured (top) and error (bottom)');

desiredComPositionOriginal = desiredComPosition;
desiredComPosition = [];


simpleSystemBestGuess = simpleSystem;
for iterationsOfSimpleSystemParameterEstimation = 1:2
  % first do an integration of the fully controlled complex system,
  % using our current best guess for the simple system
  
  %   lqrStateCostWeighting = [1 0; 0 1/simpleSystemBestGuess.b(2)]
  %   simpleSystemBestGuess
  
  stateDot = @(t, x) complicatedSystemControlledWithSimpleStateDot(t, x, complicatedSystem, simpleSystemBestGuess, desiredComPosition, comPositionJacobian, comVelocityJacobian, ...
    'lqrStateCostWeighting', lqrStateCostWeighting, ...
    'lqrControlCostWeight', lqrControlCostWeight, ...
    'timeToSwitchDesiredPosition', timeToSwitchDesiredPosition, ...
    'controlGain', controlGain);
  [times, states] = ode45(stateDot, times, initialConditionComplicatedSystem);
  comPositions = comPositionJacobian * states';
  
  % now redo integration of latest simple system guess
  simpleStateDotControlled = @(t, x) simpleStateDotWithLQRControl(t, x, simpleSystemBestGuess, desiredComPosition, ...
    'lqrStateCostWeighting', lqrStateCostWeighting, ...
    'lqrControlCostWeight', lqrControlCostWeight, ...
    'timeToSwitchDesiredPosition', timeToSwitchDesiredPosition, ...
    'controlGain', controlGain);
  [times, simpleStates] = ode45(simpleStateDotControlled, times, initialConditionSimpleSystem);
  comEstimationError = simpleStates(:, 1) - comPositions';
  
  % plot, potentially
  %   if (mod(iterationsOfSimpleSystemParameterEstimation, 3) == 1)
  %     figure('Name', sprintf('Simple System Parameter Estimation iteration %g', iterationsOfSimpleSystemParameterEstimation))
  subplot(2,1,1)
  %     plot(times, states);
  %     hold on;
  plot(times, comPositions, 'k', 'LineWidth', 3)
  hold on;
  
  plot(times, simpleStates(:, 1), 'r--', 'LineWidth', 3)
  subplot(2,1,2)
  plot(times, comEstimationError, 'r--', 'LineWidth', 3)
  hold on;
  drawnow
  pause(0.001);
  %   end
  
  complexToSimpleStateJacobian = [comPositionJacobian; comVelocityJacobian];
  complexStateDotToCOMStateDotJacobian = complexToSimpleStateJacobian;
  
  % build up training data structures from the integrations:
  wholeDataMatrix = [];
  wholeDataVector = [];
  for timeIndex = 1:1:length(times)
    t = times(timeIndex);
    x = states(timeIndex, :)';
    
    calculatedXDot = stateDot(t, x);
    
    calculatedCOMDot = complexStateDotToCOMStateDotJacobian * calculatedXDot;
    
    % this, roughly, is the line of math we will want to implement:
    % or we will remove the B if we want to keep that constant
    % stackedRowsAB = [diag(xS') diag(uSDes')] \ calculatedCOMDot
    
    xSimple = complexToSimpleStateJacobian * x;
    diagXSTranspose = [];
    for i = 1:dimensionalityOfSimpleSystem
      diagXSTranspose = blkdiag(diagXSTranspose, xSimple');
    end
    
    [calculatedXDotSimple, uSDesired] = simpleStateDotControlled(t, xSimple);
    diagUSDesiredTranspose = [];
    for i = 1:dimensionalityOfSimpleSystem
      diagUSDesiredTranspose = blkdiag(diagUSDesiredTranspose, uSDesired');
    end
    
    if (shouldKeepBSimpleConstant)
      % if we want to assume we know BSimple for sure (in this case, it
      % means deciding that mSimple = 1, so all other parameters are relative
      % to a unitless mass thing.
      dataMatrixForTick = [diagXSTranspose];
      wholeDataVector = vertcat(wholeDataVector, calculatedCOMDot - BSimple * uSDesired);
      wholeDataMatrix = vertcat(wholeDataMatrix, dataMatrixForTick);
    else
      dataMatrixForTick = [diagXSTranspose diagUSDesiredTranspose];
      wholeDataVector = vertcat(wholeDataVector, calculatedCOMDot);
      wholeDataMatrix = vertcat(wholeDataMatrix, dataMatrixForTick);
    end
    
  end
  
  stackedRowsOfASAndBS = wholeDataMatrix \ wholeDataVector;
  
  alpha = 1;
  
  ASimpleNew = [
    stackedRowsOfASAndBS(1:2)';
    stackedRowsOfASAndBS(3:4)'];
   simpleSystemBestGuess.a = (1 - abs(alpha)) * simpleSystemBestGuess.a + alpha * ASimpleNew


  if (~shouldKeepBSimpleConstant)
    BSimpleNew = stackedRowsOfASAndBS(5:6);
    simpleSystemBestGuess.b = BSimpleNew;
  end
  
  fprintf('negative simple spring stiffness = %g, ratio of bottom two elements of simple A matrix = %g, sum(abs(err)) = %g\n', ...
    simpleSystemBestGuess.a(2,1), simpleSystemBestGuess.a(2,1) / simpleSystemBestGuess.a(2,2), sum(abs(comEstimationError)))
  
end

desiredComPosition = desiredComPositionOriginal;
simpleSystemOptimal = simpleSystemBestGuess;


%% We should now test how well our simple system works to control the complex system:


stateDot = @(t, x) complicatedSystemControlledWithSimpleStateDot(t, x, complicatedSystem, simpleSystemOptimal, ...
  desiredComPosition, comPositionJacobian, comVelocityJacobian, ...
  'lqrStateCostWeighting', lqrStateCostWeighting, ...
  'lqrControlCostWeight', lqrControlCostWeight, ...
  'timeToSwitchDesiredPosition', timeToSwitchDesiredPosition, ...
  'controlGain', controlGain);
[times, states] = ode45(stateDot, times, initialConditionComplicatedSystem);
comPositions = comPositionJacobian * states';

simpleStateDotControlled = @(t, x) simpleStateDotWithLQRControl(t, x, simpleSystemOptimal, desiredComPosition, ...
  'lqrStateCostWeighting', lqrStateCostWeighting, ...
  'lqrControlCostWeight', lqrControlCostWeight, ...
  'timeToSwitchDesiredPosition', timeToSwitchDesiredPosition, ...
  'controlGain', controlGain);
[times, simpleStates] = ode45(simpleStateDotControlled, times, initialConditionSimpleSystem);
comEstimationError = simpleStates(:, 1) - comPositions';

figure('Name', 'Complex/Simple linear system Simulation Results')
subplot(2,1,1)
plot(times, states);
hold on;
plot(times, comPositions, 'k', 'LineWidth', 3)
plot(times, simpleStates(:, 1), 'r--', 'LineWidth', 3)
subplot(2,1,2)
plot(times, comEstimationError, 'r--', 'LineWidth', 3)









%%



