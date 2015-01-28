%% Some lateral variability experiments on a model of a 3D walker with torso
% John Rebula

% States:
% pelvis position, (q1, q2, q3)
% pelvis roll angle, (q4)
% stance and swing leg angle, (q5, q6)
%
% [pelvisX, pelvisY, pelvisZ,
%  pelvisRoll,
%  stanceTheta, swingTheta];
%

% addpath(genpath('C:\Users\hbclStudent\Desktop\control'));
% addpath(genpath('C:/Users/hbclStudent/control'), '-end');


% look at how step length variability changes
% compare to eyes closed (step length variability doesn't change)


% smoothness stuff:
% look at inverse dynamics
% cost of actvating muscle, how it relates to other studies
% ankle bouncing, leg swinging, minimum jerk principle of reaching
% need a certain amount of caclcium to activate muscle, first order affect
%   if you need same muscle force in less time, 
% 


%% setup
% clc;
projectsFolder = 'C:/Users/jrebula/myProjectsSmaller/';
% projectsFolder = 'C:/Users/hbclStudent/workspace/';
% projectsFolder = 'D:/jrebula/';
rng(0);


% cd([projectsDir '\SpringyWalking\mathematica models from kishore\springyWalkerWithKnees']);
% cd([projectsDir '\DynamicWalking\trunk']);
% cd('C:/Users/hbclStudent/workspace/dynamicWalking/Branches/jrebula/ThreeDWalker');
% cd('C:/Users/hbclStudent/workspace/dynamicWalking/Branches/jrebula/ThreeDWalker');

cd([projectsFolder 'dynamicWalking/Branches/jrebula/ThreeDWalker']);
% cd([projectsFolder 'ThreeDWalker']);

addpath(genpath('.'))

format compact;

%% Time for simulation
t0 = 0;
tf = 20;
numberOfStepsToRun = 10; %5; %1; %3;

shouldAntiAlias = 0; %1; %
writeOutUnperturbedVideos = 0; %1; %
saveOutPerturbedWalkingVideo = 0; %1; %
runPerturbationExperiment = 0;

%% flaggies
forceReprocessAll = 0;

refindInitialFixedPoint = forceReprocessAll || 0; %1; %
refindLimitCycleAtDesiredSpeed = forceReprocessAll || 0; %1; %
refindLimitCycleAtSpeedAndStepLength = forceReprocessAll || 0; %1; %
refindLimitCycleForHipRollWalker = forceReprocessAll || 0; %1; %
refindLimitCycleForUprightTorso = forceReprocessAll || 0; %1; %
refindLimitCycleForABitUprightTorso = forceReprocessAll || 0; %1; %

animateUnperturbedWalker = forceReprocessAll || 1; %0; %1; %
animateLaterallyPerturbedUncontrolledFalling = forceReprocessAll || 0; %1; %
animateLaterallyPerturbedControlledAnkles = forceReprocessAll || 0; %1; %

showSomeSteps = 0;

stepsToPlotUnperturbed = 3;
stepsToPlotPerturbed = 8;


%% Initial conditions chosen by hand to give something that looks like a step, but isn't
initialConditions = ThreeDWalkerState();

walker = ThreeDWalker();
walker.plot(initialConditions);

% walker.KSwing = 0.0;

initialConditions.pelvis.xDot = 0.08;
initialConditions.pelvis.yDot = -0.1;
initialConditions.pelvis.zDot = 0.1;

initialConditions.pelvis.roll = 0.2;
initialConditions.pelvis.rollDot = -0.7;

initialConditions.stanceLeg.pitch = -0.08;
initialConditions.stanceLeg.pitchDot = 0.03;

initialConditions.swingLeg.pitch = 0.08;
initialConditions.swingLeg.pitchDot = -0.3;

% ensure the velocities start consistent with the stance phase
initialConditions = walker.modeTransition(0, initialConditions.getVector(), 'stanceFootRolling');
initialConditions = ThreeDWalkerState(initialConditions);

%% search from the crap initial state to find a limit cycle at some speed with default walker parameters
% if (refindInitialFixedPoint)
%   initialConditionGuess = initialConditions.getVector();
%   [finalState, finalParameters, limitCycleError] = walker.findLimitCycle(initialConditionGuess);
%   [finalState, finalParameters, limitCycleError] = walker.findLimitCycle(finalState);
%   fprintf('found initial limit cycle, error = %g\n', norm(limitCycleError));
%   initialConditions = initialConditions.setFromVector(finalState);
% else
%   initialConditions = initialConditions.setFromVector([
%     0.0000 0.0000 1.0000 -0.0998 -0.2226 0.1652 0.2155 0.0046 0.0897 -0.9222 0.2204 0.1637]);
% end

walker.plot(initialConditions);


%% animate the limit cycle
% if (showSomeSteps)
%   figure;
%   subplot(1, 2, 1);
%   [nextState] = walker.oneStep(initialConditions, 'interleaveAnimation', 1);
%   subplot(1, 2, 2);
%   [nextState] = walker.oneStep(nextState, 'interleaveAnimation', 1);
% end

fprintf('initial guess at limit cycle, ');
walker.printStepCharacteristics(initialConditions);

%% alter groundAngle to find a limit cycle at a nominal speed
desiredSpeed = 0.4;
desiredStepLength = desiredSpeed ^ 0.42; %0.6394; %0.8; %

% first find right speed:
parametersToAlter = {'groundAngle'};
if (refindLimitCycleAtDesiredSpeed)
  fprintf('finding limit cycle with speed = %g L / time\n', desiredSpeed);
  [limitCycleAtDesiredSpeed, finalParameters, limitCycleError] = walker.findLimitCycle(initialConditions.getVector(), ...
    'parametersToAlter', parametersToAlter, ...
    'desiredSpeed', desiredSpeed);
else
  finalParameters = 0.1966;
  limitCycleAtDesiredSpeed = [ ...
    0.0000 -0.0000 1.0000 -0.2671 -0.4809 0.4130 0.3961 0.0069 0.2087 -0.5201 0.4412 0.1033]';
end
walker = walker.setParametersFromList(parametersToAlter, finalParameters);

fprintf('limit cycle at desired speed, ');
walker.printStepCharacteristics(limitCycleAtDesiredSpeed);

%% alter groundAngle and KSwing to find a limit cycle at a nominal speed and step length
parametersToAlter = {'groundAngle', 'KSwing'};
if (refindLimitCycleAtSpeedAndStepLength)
  fprintf('finding limit cycle with speed = %g L / time, step length = %g\n', desiredSpeed, desiredStepLength);
  [limitCycleAtDesiredSpeedAndStepLength, finalParameters, limitCycleError] = walker.findLimitCycle(limitCycleAtDesiredSpeed, ...
    'parametersToAlter', parametersToAlter, ...
    'desiredSpeed', desiredSpeed, 'desiredStepLength', desiredStepLength);
  walker = walker.setParametersFromList(parametersToAlter, finalParameters);
else
  limitCycleAtDesiredSpeedAndStepLength = [0.0000   -0.0000    1.0000   -0.1547   -0.3718    0.3214    0.4164    0.0035    0.1674   -0.4515    0.4436    0.2394]';
  walker = walker.setParametersFromList(parametersToAlter, [0.1266 0.0679]);
end
fprintf('limit cycle at desired speed and step length ');
walker.printStepCharacteristics(limitCycleAtDesiredSpeedAndStepLength);

walker2 = walker;
%% get a walker with an added hip joint:

walker = ThreeDWalkerSplay(walker2);



% see ThreeDWalkerSplay.testFindLimitCycle(), 2013.8.6, 6pm to see where
% this came from.
limitCycleHipRollWalker = ThreeDWalkerSplayState([0.000000000000000  -0.000000000000000   1.000000000000000   0.092782839469035  -0.332422165389413 ...
  -0.050771981351022   0.363939245170571  -0.137939942205706   0.441917526331963  -0.085969598806670 ...
  0.095554796329744  -0.757883683880587   0.480711208135867   0.801777232517588   0.249007236095320 ...
  0.843764731844449]');
walker.groundAngle = 0.096686630828179;
walker.KSwing = 0.165721074223436;


[states] = walker.walkNSteps(limitCycleHipRollWalker.getVector(), stepsToPlotUnperturbed, ...
  'interleaveAnimation', 0);

fprintf('limit cycle of hip roll walker at desired speed and step length, ');
walker.printStepCharacteristics(limitCycleHipRollWalker.getVector());
diff(states')';

walker2 = walker;
%% get a walker with an added torso:

walker = ThreeDWalkerSplayTorso(walker2);

% limitCycleTorso = ThreeDWalkerSplayTorsoState([-0.000000000000000  -0.000000000000000   1.000000000000000   0.192889855820959  -0.319778773220563  -0.113805853674610   0.389404375914300  -0.271245715093369  -0.179592185733507   0.458380832087009  -0.149213501947876 ...
%   0.052664859559352  -1.464852169473239   0.546209395497800   1.543078559580523   0.163619579672197   1.592761148835523   1.220308385327722]');
% walker.groundAngle = 0.095191754303090;
% walker.KSwing = 0.142733467588286;

limitCycleTorso = ThreeDWalkerSplayTorsoState([0.000000000000000   0.000000000000000   1.000000000000000   0.120356044428635  -0.328777877993236  -0.067786184174453   0.370245043866664  -0.175891161712126  -0.108099742563932   0.445948264948026  -0.104909226878422 ...
  0.083914906811382  -0.951091390267835   0.494802722875279   1.004917104880350   0.232721204989729   1.052797376298956   0.738321863675509]');
walker.groundAngle = 0.096969457913419;
walker.KSwing = 0.161949688973179;

%%
figure('position', [40, 40, 800, 600])
if (writeOutUnperturbedVideos)
  set(0, 'DefaultFigureRenderer', 'OpenGL'); %'zbuffer'); %'zbuffer'); %
  opengl software;
  
  %   aviWriter = [];
  aviWriter = VideoWriter('unperturbed.avi');
  aviWriter.FrameRate = 30;
  open(aviWriter);
  
  stepsToPlotUnperturbed = 2;
  [states] = walker.walkNSteps(limitCycleTorso.getVector(), stepsToPlotUnperturbed, ...
    'interleaveAnimation', 1, 'aviWriter', aviWriter, 'shouldAntiAlias', shouldAntiAlias); %); %
  
  close(aviWriter);
end

fprintf('limit cycle of torso walker at desired speed and step length, ');
walker.printStepCharacteristics(limitCycleTorso.getVector());
diff(states')';


%% now perturb the nominal limit cycle:
perturber = LateralNoisePerturber;
perturber.standardDeviationOfNoise = 0.01; %0.05; %0.1; %0.2; %
perturber.meanOfNoise = 0; %1; % if you set this to 1, the walker falls (yay)

animateLaterallyPerturbedUncontrolledFalling = 1; %0; %
% stepsToPlotPerturbed = 12;
% animateLaterallyPerturbedUncontrolledFalling = 0; %1; %
% if (animateLaterallyPerturbedUncontrolledFalling)
figure('position', [20, 60, 800, 600]);
if (saveOutPerturbedWalkingVideo)
  walkerPerturbed = walker;
  walkerPerturbed.controllers{1} = perturber;
  
  set(0, 'DefaultFigureRenderer', 'OpenGL'); %'zbuffer'); %'zbuffer'); %
  opengl software;
  
  %   aviWriter = [];
  aviWriter = VideoWriter('perturbed.avi');
  aviWriter.FrameRate = 30;
  open(aviWriter);
  
  perturbedCycle = limitCycleTorso.getVector();
  perturbedCycle(5) = perturbedCycle(5) + 0.03;
  walkerPerturbed.controllers = {};
  
  %   [states] = walkerPerturbed.walkNSteps(limitCycleTorso.getVector(), stepsToPlotPerturbed, ...
  %     'interleaveAnimation', 1); %animateLaterallyPerturbedUncontrolledFalling); %
  [states] = walkerPerturbed.walkNSteps(perturbedCycle, stepsToPlotPerturbed, ...
    'interleaveAnimation', 1, ...
    'aviWriter', aviWriter, ...
    'shouldAntiAlias', shouldAntiAlias); %animateLaterallyPerturbedUncontrolledFalling); %
  
  close(aviWriter);
  
  diff(states')'
end
%     0.6786    0.6787    0.6763    0.6747    0.6798    0.6773    0.6703    0.6139    0.2395
%     0.0116   -0.0129    0.0143   -0.0118    0.0077   -0.0125    0.0215   -0.0112    0.0014
%    -0.0005    0.0014    0.0008   -0.0027   -0.0017    0.0067    0.0016   -0.0046    0.0478
%    -0.0202   -0.0095    0.0386    0.0161   -0.0931   -0.0268    0.2178    0.0225   -0.3148
%    -0.0051    0.0045    0.0103   -0.0108   -0.0207    0.0253    0.0592   -0.0446    0.4507
%     0.0161    0.0184   -0.0289   -0.0420    0.0716    0.0908   -0.1707   -0.1670    0.3899
%    -0.0024   -0.0059    0.0041    0.0107   -0.0096   -0.0253    0.0342    0.0101   -0.4397
%     0.0203    0.0133   -0.0474   -0.0253    0.1191    0.0467   -0.2807   -0.0704    0.4595
%    -0.0019   -0.0009    0.0048    0.0012   -0.0102   -0.0027    0.0293    0.0097   -0.0435
%    -0.0002    0.0007    0.0015   -0.0010   -0.0013    0.0012    0.0142    0.0078    0.0629
%     0.0021   -0.0042   -0.0037    0.0101    0.0070   -0.0240   -0.0178    0.0502   -0.1586
%     0.0034   -0.0259   -0.0007    0.0661   -0.0097   -0.1378    0.0232    0.4486    0.5752
%    -0.0061   -0.0066    0.0121    0.0137   -0.0284   -0.0312    0.0717    0.0394   -0.1139
%    -0.0021    0.0262   -0.0021   -0.0670    0.0159    0.1387   -0.0406   -0.4599   -0.5971
%     0.0034    0.0073   -0.0062   -0.0134    0.0171    0.0348   -0.0443    0.0261    0.1627
%    -0.0059    0.0213    0.0077   -0.0553   -0.0133    0.1211    0.0107   -0.3959   -0.7132

%% add a stance ankle torque controller to stabilize gait
linearizationPerturbationAmount = 1e-4; %1e-7;
controlInputs = {'stanceAnkleTorque', 'forceBetweenLegs', 'torsoRollTorque', 'extraSwingTorque'}; %}; %
% controlInputs = {'forceBetweenLegs', 'torsoRollTorque'}; % 'extraSwingTorque'};
[A, B] = walker.linearizeOneStepReturnMap(limitCycleTorso, controlInputs, 'perturbationAmount', linearizationPerturbationAmount); %1e-2); %

%%
if (0)
  AMin = A;
  BMin = B';
  
  statesToRemove = [1 2 3 10 11 12];
  % statesToRemove = [1 2 3 9 10 11];
  % statesToRemove = [1 2 3 5 7 9 10 11 13 15];
  goodStates = 1:length(A);
  goodStates(statesToRemove) = [];
  theseGoodStates{1} = goodStates;
  goodStates = theseGoodStates;
  goodStates.subs = goodStates;
  goodStates.type = '()';
  
  AMin(statesToRemove, :) = [];
  AMin(:, statesToRemove) = [];
  BMin(statesToRemove, :) = [];
  
  sys = ss(AMin, BMin, eye(size(AMin)), [], []);
  
  % minSys = minreal(sys);
  minSys = sys;
  
  Q = eye(size(minSys.a));
  pelvisAndTorsoGain = 1e-3; %1e-2; %
  pelvisAndTorsoDotGain = 1e-3;
  Q(1, 1) = pelvisAndTorsoGain;
  Q(6, 6) = pelvisAndTorsoGain;
  Q(1 + 6, 1 + 6) = pelvisAndTorsoDotGain;
  Q(6 + 6, 6 + 6) = pelvisAndTorsoDotGain;
  
  % R = diag([1e5, 1e-1]);
  R = diag([1e1, 1e0, 1e0]);
  
  N = [];
  eig(minSys.a);
  [K, s, e] = lqr(minSys, Q, R, N);
  
  K
  e
  
  %% test controller
  
  if (runControlledWalkerSimulation)
    calculateControlFromKAndInitialState = @(initialState, K) -K * subsref(initialState - limitCycleTorso.getVector(), goodStates);
    
    % perturber = LateralNoisePerturber;
    % perturber.standardDeviationOfNoise = 0.5;
    perturber = perturber.resetToDefault();
    walkerPerturbed = walker;
    walkerPerturbed.controllers{1} = perturber;
    %   walkerPerturbed.controllers = {};
    
    figure;
    controlledStates = [limitCycleTorso.getVector()];
    
    [nextState] = walkerPerturbed.oneStep(limitCycleTorso, 'interleaveAnimation', animateLaterallyPerturbedControlledAnkles);
    controlledStates = [controlledStates nextState.getVector()];
    
    for i = 1 : 8
      walkerPerturbed = walkerPerturbed.setParametersFromList(controlInputs, calculateControlFromKAndInitialState(nextState.getVector(), K));
      fprintf('stanceAnkleTorque = %g, extraSwingTorque = %g, torsoRollTorque = %g\n', ...
        walkerPerturbed.stanceAnkleTorque, walkerPerturbed.extraSwingTorque, walkerPerturbed.torsoRollTorque);
      [nextState] = walkerPerturbed.oneStep(nextState, 'interleaveAnimation', animateLaterallyPerturbedControlledAnkles, 'plotLegsSwitched', 1);
      nextState.pelvis.y = -nextState.pelvis.y;
      controlledStates = [controlledStates nextState.getVector()];
      points = walkerPerturbed.getKinematicPoints(nextState);
      if (points.centerOfMassPosition(3) < 0.51)
        break;
      end
      walkerPerturbed = walkerPerturbed.setParametersFromList(controlInputs, calculateControlFromKAndInitialState(nextState.getVector(), K));
      fprintf('stanceAnkleTorque = %g, extraSwingTorque = %g, torsoRollTorque = %g\n', ...
        walkerPerturbed.stanceAnkleTorque, walkerPerturbed.extraSwingTorque, walkerPerturbed.torsoRollTorque);
      [nextState] = walkerPerturbed.oneStep(nextState, 'interleaveAnimation', animateLaterallyPerturbedControlledAnkles, 'plotLegsSwitched', 0);
      points = walkerPerturbed.getKinematicPoints(nextState);
      nextState.pelvis.y = -nextState.pelvis.y;
      controlledStates = [controlledStates nextState.getVector()];
      if (points.centerOfMassPosition(3) < 0.51)
        break;
      end
    end
    
    diff(controlledStates')'
    
    controlledStates(:, 1:size(states, 2)) - states
  end
end

%% run experiments with different lqr gains:
if (runPerturbationExperiment)
  
  warning off;
  
  perturbationStandardDeviation = 1e-4; %5; %3; %1; %1e-2; %1e0; %
  
  Q = eye(12, 12); %minSys.a)); %
  % Q(7:end, 7:end) = 0;
  % pelvisAndTorsoGain = 1e-3; %1e-2; %
  % pelvisAndTorsoDotGain = 1e-3;
  % pelvisAndTorsoGain = 1e1; %1e-2; %
  % pelvisAndTorsoDotGain = 1e1;
  % Q(1, 1) = pelvisAndTorsoGain;
  % Q(6, 6) = pelvisAndTorsoGain;
  % Q(1 + 6, 1 + 6) = pelvisAndTorsoDotGain;
  % Q(6 + 6, 6 + 6) = pelvisAndTorsoDotGain;
  
  Q = Q * 1;
  
  % R = diag([1e5, 1e-1]);
  % order is: stanceAnkleTorque, forceBetweenLegs, torsoRollTorque
  % R = diag([1e1, 1e0, 1e0]);
  % RLessAnkleTorque = diag([2e1, 1e0, 1e0]);
  % RLessFootPlacement = diag([1e1, 1.1e0, 1e0]);
  
  % ankleR = 1e1; %1e0; %
  % footR = 1e-2; %1e1; %
  % torsoR = 1e-3; %1e1; %
  %
  % controlWeightDeltaAnkle = 1e-4; %1e-3; %
  % controlWeightDeltaFoot = 1e-4; %1e-3; %
  % R = diag([ankleR, footR, torsoR]);
  % RLessAnkleTorque = diag([ankleR + controlWeightDeltaAnkle, footR, torsoR]);
  % RLessFootPlacement = diag([ankleR, footR + controlWeightDeltaFoot, torsoR]);
  
  % R = diag([1e1, 1e0, 1e0]);
  % RLessAnkleTorque = diag([1e4, 1e0, 1e0]);
  % RLessFootPlacement = diag([1e1, 1e3, 1e0]);
  
  
  R = diag([1e1, 1e0, 1e0, 1e0]);
  RLessAnkleTorque = diag([1e4, 1e0, 1e0, 1e0]);
  RLessFootPlacement = diag([1e1, 1e3, 1e0, 1e0]);
  
  experiment = WalkerVariabilityExperiment();
  experiment.maxNumberOfStepsToWalkAtOnce = 10;
  
  %%
  rng(0);
  % fprintf('normal:\n');
  resultsNormal = experiment.runVariabilityExperiment(A, B, Q, R, controlInputs, ...
    walker, perturbationStandardDeviation, limitCycleTorso);
  % stepLengthDiffs = diff(diff(resultsNormal.controlledStates(1, :)));
  % mean(stepLengthDiffs)
  
  
  %%
  rng(0);
  % fprintf('less ankle torque:\n');
  resultsLessAnkleTorque = experiment.runVariabilityExperiment(A, B, Q, RLessAnkleTorque, controlInputs, ...
    walker, perturbationStandardDeviation, limitCycleTorso);
  % stepLengthDiffs = diff(diff(resultsLessAnkleTorque.controlledStates(1, :)));
  % mean(stepLengthDiffs)
  
  %%
  rng(0);
  % fprintf('less foot placement:\n');
  resultsLessFootPlacement = experiment.runVariabilityExperiment(A, B, Q, RLessFootPlacement, controlInputs, ...
    walker, perturbationStandardDeviation, limitCycleTorso);
  % stepLengthDiffs = diff(diff(resultsLessFootPlacement.controlledStates(1, :)));
  % mean(stepLengthDiffs)
  
  
  
  
  %%
  
  % y = 2x, then y has 4 times the variance = 2 * xCov * 2
  % Ap = (A-BK), say
  % W: covaraince of process noise:
  
  % A * X * A' + B * W * B'
  
  
  %%
  fprintf('less torso inertia:\n');
  
  inertiaAmountToChangeBy = 5e-2;
  walkerLessTorsoInertia = walker;
  walkerLessTorsoInertia.ITorso = walkerLessTorsoInertia.ITorso * (1 - inertiaAmountToChangeBy); %(1 + 1e-4); %0.8;
  
  refindLimitCycleForAlteredTorsoInertia = 1;
  if (refindLimitCycleForAlteredTorsoInertia)
    fprintf('finding limit cycle for lower inertia torso, with speed = %g L / time, step length = %g\n', desiredSpeed, desiredStepLength);
    [limitCycleLessTorso, finalParameters, limitCycleError] = walkerLessTorsoInertia.findLimitCycle(limitCycleTorso.getVector(), ...
      'parametersToAlter', parametersToAlter, ...
      'desiredSpeed', desiredSpeed, 'desiredStepLength', desiredStepLength);
    walkerLessTorsoInertia = walkerLessTorsoInertia.setParametersFromList(parametersToAlter, finalParameters);
  else
    %   limitCycleAtDesiredSpeedAndStepLength = [0.0000   -0.0000    1.0000   -0.1547   -0.3718    0.3214    0.4164    0.0035    0.1674   -0.4515    0.4436    0.2394]';
    %   walkerLessTorsoInertia = walkerLessTorsoInertia.setParametersFromList(parametersToAlter, [0.1266 0.0679]);
  end
  
  [ALessTorso, BLessTorso] = walkerLessTorsoInertia.linearizeOneStepReturnMap(limitCycleLessTorso, controlInputs, 'perturbationAmount', linearizationPerturbationAmount); %1e-2); %
  
  
  rng(0);
  resultsLessTorsoInertia = experiment.runVariabilityExperiment(ALessTorso, BLessTorso, Q, R, controlInputs, ...
    walkerLessTorsoInertia, perturbationStandardDeviation, ThreeDWalkerSplayTorsoState(limitCycleLessTorso));
  % stepLengthDiffs = diff(diff(resultsLessTorsoInertia.controlledStates(1, :)));
  % mean(stepLengthDiffs)
  
  %%
  % rng(0);
  fprintf('more torso inertia:\n');
  
  walkerMoreTorsoInertia = walker;
  walkerMoreTorsoInertia.ITorso = walkerMoreTorsoInertia.ITorso * (1 + inertiaAmountToChangeBy); %(1 + 1e-4); %0.8;
  
  refindLimitCycleForAlteredTorsoInertia = 1;
  if (refindLimitCycleForAlteredTorsoInertia)
    fprintf('finding limit cycle for higher inertia torso, with speed = %g L / time, step length = %g\n', desiredSpeed, desiredStepLength);
    [limitCycleMoreTorso, finalParameters, limitCycleError] = walkerMoreTorsoInertia.findLimitCycle(limitCycleTorso.getVector(), ...
      'parametersToAlter', parametersToAlter, ...
      'desiredSpeed', desiredSpeed, 'desiredStepLength', desiredStepLength);
    walkerMoreTorsoInertia = walkerMoreTorsoInertia.setParametersFromList(parametersToAlter, finalParameters);
  else
    limitCycleAtDesiredSpeedAndStepLength = [0.000000000000000   0.000000000000000   1.000000000000000   0.120567601436005  -0.329064602665633  -0.067896365825421   0.370578191085329  -0.176216135919916  -0.108522290587947   0.445947020359458  -0.105106964014976 ...
      0.084025472954101  -0.951051095565299   0.494912810276100   1.004972876469711   0.232254761996496   1.052986373931895   0.743280164268791]';
    walkerLessTorsoInertia = walkerLessTorsoInertia.setParametersFromList(parametersToAlter, [ 0.097115313480100 0.161605308623616]');
  end
  
  [AMoreTorso, BMoreTorso] = walkerMoreTorsoInertia.linearizeOneStepReturnMap(limitCycleMoreTorso, controlInputs, 'perturbationAmount', linearizationPerturbationAmount); %1e-2); %
  
  rng(0);
  resultsMoreTorsoInertia = experiment.runVariabilityExperiment(AMoreTorso, BMoreTorso, Q, R, controlInputs, ...
    walkerMoreTorsoInertia, perturbationStandardDeviation, ThreeDWalkerSplayTorsoState(limitCycleMoreTorso));
  % stepLengthDiffs = diff(diff(resultsMoreTorsoInertia.controlledStates(1, :)));
  % mean(stepLengthDiffs)
  
  %% look at results
  normalVars = experiment.printResults('normal', resultsNormal);
  lessAnkleTorqueVars = experiment.printResults('lessAnkleTorque', resultsLessAnkleTorque);
  lessFootPlacementVars = experiment.printResults('lessFootPlacement', resultsLessFootPlacement);
  resultsLessTorsoVars = experiment.printResults('resultsLessTorsoInertia', resultsLessTorsoInertia);
  resultsMoreTorsoVars = experiment.printResults('resultsMoreTorsoInertia', resultsMoreTorsoInertia);
  
  %   controlInputs
  %
  %   fprintf('normal vars\n');
  %   normalVars
  %
  %   fprintf('less ankle torque vars\n');
  %   lessAnkleTorqueVars
  %   % fprintf('less ankle torque vars diff from normal\n');
  %   % (lessAnkleTorqueVars - normalVars) ./ controlWeightDeltaAnkle
  %
  %   fprintf('less foot placement vars\n');
  %   lessFootPlacementVars
  %   % fprintf('less foot placement vars diff from normal\n');
  %   % (lessFootPlacementVars  - normalVars) ./ controlWeightDeltaFoot
  %
  %   fprintf('less torso inertia vars\n');
  %   resultsLessTorsoVars
  %
  %   fprintf('more torso inertia vars\n');
  %   resultsMoreTorsoVars
  
  %% plot the results:
  
  listOfExperimentNames = { ...
    'less ankle torque', ...
    'less foot placement', ...
    'less torso inertia', ...
    'more torso inertia', ...
    };
  listOfExperimentResults = { ...
    resultsLessAnkleTorque, ...
    resultsLessFootPlacement, ...
    resultsLessTorsoInertia, ...
    resultsMoreTorsoInertia, ...
    };
  plotVariabilityChangesFromNormal(resultsNormal, listOfExperimentNames, listOfExperimentResults)
  
  %   % figure;
  %   fig = GridFigure(4, 1);
  %   xTicks = []; %[1:3];
  %   yLims = [-1.5 1.5];
  %   xLims = [0.5 3.5];
  %
  %   fig = fig.nextAvailableSubplot();
  %   valuesMinusNorm = lessAnkleTorqueVars - normalVars;
  %   valuesMinusNorm = valuesMinusNorm ./ abs(valuesMinusNorm);
  %   stem(valuesMinusNorm);
  %   set(gca, 'xtick', xTicks);
  %   axis off;
  %   box off;
  %   xlim(xLims);
  %   ylim(yLims);
  %   title('less ankle torque');
  %
  %   fig = fig.nextAvailableSubplot();
  %   valuesMinusNorm = lessFootPlacementVars - normalVars;
  %   valuesMinusNorm = valuesMinusNorm ./ abs(valuesMinusNorm);
  %   stem(valuesMinusNorm);
  %   set(gca, 'xtick', xTicks);
  %   axis off;
  %   box off;
  %   xlim(xLims);
  %   ylim(yLims);
  %   title('lessFootPlacementVars');
  %
  %   fig = fig.nextAvailableSubplot();
  %   valuesMinusNorm = resultsLessTorsoVars - normalVars;
  %   valuesMinusNorm = valuesMinusNorm ./ abs(valuesMinusNorm);
  %   stem(valuesMinusNorm);
  %   set(gca, 'xtick', xTicks);
  %   axis off;
  %   box off;
  %   xlim(xLims);
  %   ylim(yLims);
  %   title('resultsLessTorsoVars');
  %
  %   fig = fig.nextAvailableSubplot();
  %   valuesMinusNorm = resultsMoreTorsoVars - normalVars;
  %   valuesMinusNorm = valuesMinusNorm ./ abs(valuesMinusNorm);
  %   stem(valuesMinusNorm);
  %   % axis off;
  %   box off;
  %   xlim(xLims);
  %   ylim(yLims);
  %   title('resultsMoreTorsoVars');
  %
  %   set(gca, 'xtick', [1:3]);
  %   set(gca, 'xticklabel', controlInputs(1:3));
  %   xlabel('deviations');
  
  
  %%
  % p = ModelMechanismsResults(resultsNormal);
  % p.plot();
  %
  % p = ModelMechanismsResults(resultsLessAnkleTorque);
  % p.plot();
  %
  % p = ModelMechanismsResults(resultsLessFootPlacement);
  % p.plot();
  
end




%% look at what happns when we control trunk to upright, vs around the limit cycle (i.e. which requires
% more control effort to stabilize?)
% first we have just the normal uncontrolled limit cycle...

[finalState, finalTime, allStates, allTimes] = walker.oneStep(limitCycleTorso);
allRolls = zeros(size(allStates, 2), 1);
allRollDots = allRolls;
for i = 1 : size(allStates, 2)
  state = walker.getWalkerStateObjectFromVector(allStates(:, i));
  allRolls(i) = state.torso.roll;
  allRollDots(i) = state.torso.rollDot;
end

allRollsUncontrolled = allRolls;
allRollDotsUncontrolled = allRollDots;
allTimesUncontrolled = allTimes;

limitCycleTorsoRolls = spline(allTimes(2:end), allRolls(2:end));
limitCycleTorsoRollDots = spline(allTimes(2:end), allRollDots(2:end));

uncontrolledColor = ones(3, 1) * 0.8;
fig = GridFigure(1, 1);
plot(allTimes, allRolls, '-', 'LineWidth', 4, 'color', uncontrolledColor);
hold on;
plot(allTimes, allRollDots, '--', 'LineWidth', 4, 'color', uncontrolledColor)
legendString = {'roll uncontrolled', 'rollDot uncontrolled'};

%% create a walker that controls it's torso around the limit cycle
torsoControlledToLimitCycleWalker = walker;
torsoControlledToLimitCycleWalker.controllers = {ContinuousTrunkController(limitCycleTorsoRolls, limitCycleTorsoRollDots)};
% [states] = torsoControlledToLimitCycleWalker.walkNSteps(limitCycleTorso, 3, 'interleaveAnimation', 0);
fprintf('limit cycle for torso controlled around limit cycle: ');
torsoControlledToLimitCycleWalker.printStepCharacteristics(limitCycleTorso);

[finalState, finalTime, allStates, allTimes] = torsoControlledToLimitCycleWalker.oneStep(limitCycleTorso);
allRolls = zeros(size(allStates, 2), 1);
allRollDots = allRolls;
for i = 1 : size(allStates, 2)
  state = torsoControlledToLimitCycleWalker.getWalkerStateObjectFromVector(allStates(:, i));
  allRolls(i) = state.torso.roll;
  allRollDots(i) = state.torso.rollDot;
end

plot(allTimes, allRolls, '-', 'color', 'k');
hold on;
plot(allTimes, allRollDots, '--', 'color', 'k')
legendString = [legendString, 'roll controlled to limit cycle', 'rollDot controlled to limit cycle'];


%% create a walker that controls its torso to upright
uprightTorsoRolls = spline(allTimes(2:end), allRolls(2:end) * 0);
uprightTorsoRollDots = spline(allTimes(2:end), allRollDots(2:end) * 0);
torsoControlledToUprightWalker = walker;
torsoControlledToUprightWalker.controllers = {ContinuousTrunkController(uprightTorsoRolls, uprightTorsoRollDots)};

% [states] = torsoControlledToUprightWalker.oneStep(limitCycleTorso, 'interleaveAnimation', 1);

if (refindLimitCycleForUprightTorso)
  fprintf('finding limit cycle with speed = %g L / time, step length = %g\n', desiredSpeed, desiredStepLength);
  [limitCycleUprightTorso, finalParameters, limitCycleError] = torsoControlledToUprightWalker.findLimitCycle(limitCycleTorso.getVector(), ...
    'parametersToAlter', parametersToAlter, ...
    'desiredSpeed', desiredSpeed, 'desiredStepLength', desiredStepLength);
  torsoControlledToUprightWalker = torsoControlledToUprightWalker.setParametersFromList(parametersToAlter, finalParameters);
else
  limitCycleUprightTorso = [-1.86116e-24, 0, 1, 0.0654981, -0.336539, -0.0326787, 0.358491, -0.101486, 0.00872666, 0.439636, -0.0789944, 0.100657, -0.695126, 0.472552, 0.736438, 0.26243, 0.771531, 0.2298, ]';
  torsoControlledToUprightWalker = torsoControlledToUprightWalker.setParametersFromList(parametersToAlter, [0.0957 0.1677]');
end
fprintf('limit cycle for upright torso: ');
torsoControlledToUprightWalker.printStepCharacteristics(limitCycleUprightTorso);

% [states] = torsoControlledToUprightWalker.walkNSteps(limitCycleUprightTorso, 3, 'interleaveAnimation', 0);

[finalState, finalTime, allStates, allTimes] = torsoControlledToUprightWalker.oneStep(limitCycleUprightTorso);
allRolls = zeros(size(allStates, 2), 1);
allRollDots = allRolls;
for i = 1 : size(allStates, 2)
  state = torsoControlledToUprightWalker.getWalkerStateObjectFromVector(allStates(:, i));
  allRolls(i) = state.torso.roll;
  allRollDots(i) = state.torso.rollDot;
end

plot(allTimes, allRolls, '-', 'color', 'b');
hold on;
plot(allTimes, allRollDots, '--', 'color', 'b')
legendString = [legendString, 'roll controlled to upright', 'rollDot controlled to upright'];

%% create a walker that controls its torso to upright just a tiny bit
torsoControlledToUprightABitWalker = walker;

desiredTorsoTrajectoryGain = (1 - 1e-3);
aBitUprightTorsoRolls = spline(allTimesUncontrolled(2:end), allRollsUncontrolled(2:end) * desiredTorsoTrajectoryGain);
aBitUprightTorsoRollDots = spline(allTimesUncontrolled(2:end), allRollDotsUncontrolled(2:end) * desiredTorsoTrajectoryGain);
torsoControlledToUprightABitWalker.controllers = ...
  {ContinuousTrunkController(aBitUprightTorsoRolls, aBitUprightTorsoRollDots)};

% use this to control to upright, just with tiny gains
% torsoControlledToUprightABitWalker.controllers = {ContinuousTrunkController(uprightTorsoRolls, uprightTorsoRollDots)};
% epsilonUprightTorsoPGain = 1e-3;
% epsilonUprightTorsoDGain = epsilonUprightTorsoPGain;
% torsoControlledToUprightABitWalker.controllers{1}.p = epsilonUprightTorsoPGain;
% torsoControlledToUprightABitWalker.controllers{1}.d = epsilonUprightTorsoDGain;

% [states] = torsoControlledToUprightWalker.oneStep(limitCycleTorso, 'interleaveAnimation', 1);

% parametersToAlter = {'groundAngle'};
if (refindLimitCycleForABitUprightTorso)
  fprintf('finding limit cycle with speed = %g L / time, step length = %g\n', desiredSpeed, desiredStepLength);
  [limitCycleABitUprightTorso, finalParameters, limitCycleError] = torsoControlledToUprightABitWalker.findLimitCycle(limitCycleTorso.getVector(), ...
    'parametersToAlter', parametersToAlter, ...
    'desiredSpeed', desiredSpeed, 'desiredStepLength', desiredStepLength);
  torsoControlledToUprightABitWalker = torsoControlledToUprightABitWalker.setParametersFromList(parametersToAlter, finalParameters);
else
  limitCycleABitUprightTorso = [-1.42172e-25, -1.55096e-25, 1, 0.120154, -0.329118, -0.0674752, 0.370486, -0.175817, -0.107321, 0.445941, -0.105098, 0.0840508, -0.950985, 0.494758, 1.00492, 0.232435, 1.05288, 0.737299,]';
  torsoControlledToUprightABitWalker = torsoControlledToUprightABitWalker.setParametersFromList(parametersToAlter, ...
    [0.097155, 0.161645]');
end
fprintf('limit cycle for a bit upright torso: ');
torsoControlledToUprightABitWalker.printStepCharacteristics(limitCycleABitUprightTorso);

[finalState, finalTime, allStates, allTimes] = torsoControlledToUprightABitWalker.oneStep(limitCycleABitUprightTorso);
allRolls = zeros(size(allStates, 2), 1);
allRollDots = allRolls;
for i = 1 : size(allStates, 2)
  state = torsoControlledToUprightABitWalker.getWalkerStateObjectFromVector(allStates(:, i));
  allRolls(i) = state.torso.roll;
  allRollDots(i) = state.torso.rollDot;
end

plot(allTimes, allRolls, '-', 'color', 'c');
hold on;
plot(allTimes, allRollDots, '--', 'color', 'c')
legendString = [legendString, 'roll controlled a bit to upright', 'rollDot controlled a bit to upright'];
legend(legendString);
box off;



%%
runVariabilityExperimentForUprightTorso = 1; %0;
if (runVariabilityExperimentForUprightTorso)
  
  [AControlledToLimitCycle, BControlledToLimitCycle] = ...
    torsoControlledToLimitCycleWalker.linearizeOneStepReturnMap(limitCycleTorso, ...
    controlInputs, 'perturbationAmount', linearizationPerturbationAmount); %1e-2); %
  rng(0);
  resultsTorsoControlledToLimitCycle = experiment.runVariabilityExperiment( ...
    AControlledToLimitCycle, BControlledToLimitCycle, Q, R, ...
    controlInputs, ...
    torsoControlledToLimitCycleWalker, perturbationStandardDeviation, limitCycleTorso);
  
  [AControlledToUpright, BControlledToUpright] = ...
    torsoControlledToUprightWalker.linearizeOneStepReturnMap(limitCycleTorsoControlledToUpright, ...
    controlInputs, 'perturbationAmount', linearizationPerturbationAmount); %1e-2); %
  rng(0);
  resultsTorsoControlledToUpright = experiment.runVariabilityExperiment( ...
    AControlledToUpright, BControlledToUpright, QControlledToUpright, RControlledToUpright, ...
    controlInputs, ...
    torsoControlledToUprightWalker, perturbationStandardDeviation, limitCycleTorsoControlledToUpright);
  
  [AControlledABitToUpright, BControlledABitToUpright] = ...
    torsoControlledToUprightABitWalker.linearizeOneStepReturnMap(limitCycleTorsoControlledABitToUpright, ...
    controlInputs, 'perturbationAmount', linearizationPerturbationAmount); %1e-2); %
  rng(0);
  resultsTorsoControlledABitToUpright = experiment.runVariabilityExperiment( ...
    AControlledABitToUpright, BControlledABitToUpright, Q, R, ...
    controlInputs, ...
    torsoControlledToUprightABitWalker, perturbationStandardDeviation, limitCycleTorsoControlledABitToUpright);
  
  normalVars = experiment.printResults('normal', resultsNormal);
  controlledToLimitCycleVars = experiment.printResults('torsoControlledToLimitCycle', resultsTorsoControlledToLimitCycle);
  controlledToUprightVars = experiment.printResults('torsoControlledToUpright', resultsTorsoControlledToUpright);
  controlledABitToUprightVars = experiment.printResults('torsoControlledABitToUpright', resultsTorsoControlledABitToUpright);
  
  %% plot the results:
  
  listOfExperimentNames = { ...
    'controlled to limit cycle', ...
    'controlled to upright', ...
    'controlled to a bit upright', ...
    };
  listOfExperimentResults = { ...
    resultsTorsoControlledToLimitCycle, ...
    resultsTorsoControlledToUpright, ...
    resultsTorsoControlledABitToUpright ...
    };
  experiment.plotVariabilityChangesFromNormal(resultsNormal, listOfExperimentNames, listOfExperimentResults)
  
end

