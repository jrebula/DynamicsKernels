%% 3D walker with torso
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


%% setup
% clc;
% projectsFolder = 'C:/Users/jrebula/myProjectsSmaller/';
projectsFolder = 'C:/Users/hbclStudent/workspace/';
rng(0);

% cd([projectsDir '\SpringyWalking\mathematica models from kishore\springyWalkerWithKnees']);
% cd([projectsDir '\DynamicWalking\trunk']);
% cd('C:/Users/hbclStudent/workspace/dynamicWalking/Branches/jrebula/ThreeDWalker');
% cd('C:/Users/hbclStudent/workspace/dynamicWalking/Branches/jrebula/ThreeDWalker');

cd([projectsFolder 'dynamicWalking/Branches/jrebula/ThreeDWalker']);

addpath(genpath('.'))

format compact;

%% Time for simulation
t0 = 0;
tf = 20;
numberOfStepsToRun = 10; %5; %1; %3;

%% flaggies
forceReprocessAll = 0;

refindInitialFixedPoint = forceReprocessAll || 0; %1; %
refindLimitCycleAtDesiredSpeed = forceReprocessAll || 0; %1; %
refindLimitCycleAtSpeedAndStepLength = forceReprocessAll || 0; %1; %
refindLimitCycleForHipRollWalker = forceReprocessAll || 0; %1; %

animateUnperturbedWalker = forceReprocessAll || 0; %1; %
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

% walker.plot(initialConditions);
fprintf('initial guess at limit cycle, ');
walker.printStepCharacteristics(initialConditions);

%% alter groundAngle and KSwing to find a limit cycle at a nominal speed and step length
parametersToAlter = {'groundAngle', 'KSwing'};
walker = walker.setParametersFromList(parametersToAlter, [0.1266 0.0679]);
walker2 = walker;

%% get a walker with an added hip joint:
walker = ThreeDWalkerSplay(walker2);
limitCycleHipRollWalker = ThreeDWalkerSplayState([0.000000000000000  -0.000000000000000   1.000000000000000   0.092782839469035  -0.332422165389413 ...
  -0.050771981351022   0.363939245170571  -0.137939942205706   0.441917526331963  -0.085969598806670 ...
  0.095554796329744  -0.757883683880587   0.480711208135867   0.801777232517588   0.249007236095320 ...
  0.843764731844449]');
walker.groundAngle = 0.096686630828179;
walker.KSwing = 0.165721074223436;

walker2 = walker;
%% get a walker with an added torso:

walker = ThreeDWalkerSplayTorso(walker2);

limitCycleTorso = ThreeDWalkerSplayTorsoState([0.000000000000000   0.000000000000000   1.000000000000000   0.120356044428635  -0.328777877993236  -0.067786184174453   0.370245043866664  -0.175891161712126  -0.108099742563932   0.445948264948026  -0.104909226878422 ...
  0.083914906811382  -0.951091390267835   0.494802722875279   1.004917104880350   0.232721204989729   1.052797376298956   0.738321863675509]');
walker.groundAngle = 0.096969457913419;
walker.KSwing = 0.161949688973179;

% try
%   [states] = walker.walkNSteps(limitCycleTorso.getVector(), stepsToPlotUnperturbed, ...
%     'interleaveAnimation', animateUnperturbedWalker);
% catch e
%   e
% end

fprintf('limit cycle of torso walker at desired speed and step length, ');
walker.printStepCharacteristics(limitCycleTorso.getVector());

%%





%% now perturb the nominal limit cycle:
perturber = LateralNoisePerturber;
perturber.standardDeviationOfNoise = 0.01; %0.05; %0.1; %0.2; %
perturber.meanOfNoise = 0; %1; % if you set this to 1, the walker falls (yay)

animateLaterallyPerturbedUncontrolledFalling = 1; %0; %
% stepsToPlotPerturbed = 12;
% animateLaterallyPerturbedUncontrolledFalling = 0; %1; %
if (animateLaterallyPerturbedUncontrolledFalling)
  walkerPerturbed = walker;
  walkerPerturbed.controllers{1} = perturber;
  %   walkerPerturbed.controllers = {};
  
  [states] = walkerPerturbed.walkNSteps(limitCycleTorso.getVector(), stepsToPlotPerturbed, ...
    'interleaveAnimation', 0); %animateLaterallyPerturbedUncontrolledFalling); %
  
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
controlInputs = {'stanceAnkleTorque', 'forceBetweenLegs', 'torsoRollTorque'}; % 'extraSwingTorque'};
[A, B] = walker.linearizeOneStepReturnMap(limitCycleTorso, controlInputs, 'perturbationAmount', 1e-5); %1e-2); %


B

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


% R = 1e5; %1e-5; %1; %
% R = eye(2) * 1e1; %1e5; %1e0; %1e-5; %1; %
% R = eye(1) * 1e5; %1e1; %1e0; %1e-5; %1; %

% R = diag([1e5, 1e-1]);
R = diag([1e1, 1e0, 1e0]);

N = [];
eig(minSys.a);
[K, s, e] = lqr(minSys, Q, R, N);

K
e

%% test controller

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


%% run experiments with different lqr gains:

warning off;

perturbationStandardDeviation = 1e-2; %3; %1e0; %

Q = eye(12, 12); %minSys.a)); %
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

ankleR = 1e1; %1e0; %
footR = 1e-2; %1e1; %
torsoR = 1e-3; %1e1; %

controlWeightDeltaAnkle = 1e-4; %1e-3; %
controlWeightDeltaFoot = 1e-4; %1e-3; %
R = diag([ankleR, footR, torsoR]);
RLessAnkleTorque = diag([ankleR + controlWeightDeltaAnkle, footR, torsoR]);
RLessFootPlacement = diag([ankleR, footR + controlWeightDeltaFoot, torsoR]);

experiment = WalkerVariabilityExperiment();
experiment.maxNumberOfStepsToWalkAtOnce = 10;
%%
rng(0);
fprintf('normal:\n');
resultsNormal = experiment.runVariabilityExperiment(A, B, Q, R, controlInputs, ...
        walker, perturbationStandardDeviation, limitCycleTorso);
stepLengthDiffs = diff(diff(resultsNormal.controlledStates(1, :)));
mean(stepLengthDiffs)

%%
rng(0);
fprintf('less ankle torque:\n');
resultsLessAnkleTorque = experiment.runVariabilityExperiment(A, B, Q, RLessAnkleTorque, controlInputs, ...
        walker, perturbationStandardDeviation, limitCycleTorso);
stepLengthDiffs = diff(diff(resultsLessAnkleTorque.controlledStates(1, :)));
mean(stepLengthDiffs)

%%
rng(0);
fprintf('less foot placement:\n');
resultsLessFootPlacement = experiment.runVariabilityExperiment(A, B, Q, RLessFootPlacement, controlInputs, ...
        walker, perturbationStandardDeviation, limitCycleTorso);
stepLengthDiffs = diff(diff(resultsLessFootPlacement.controlledStates(1, :)));
mean(stepLengthDiffs)


%%
normalVars = experiment.printResults('normal', resultsNormal);

lessAnkleTorqueVars = experiment.printResults('lessAnkleTorque', resultsLessAnkleTorque);
(abs(resultsLessAnkleTorque.controllerGain) - abs(resultsNormal.controllerGain)) ./ abs(resultsNormal.controllerGain);

lessFootPlacementVars = experiment.printResults('lessFootPlacement', resultsLessFootPlacement);
(abs(resultsLessFootPlacement.controllerGain) - abs(resultsNormal.controllerGain)) ./ abs(resultsNormal.controllerGain);

controlInputs
fprintf('less ankle torque vars diff from normal\n');
(lessAnkleTorqueVars - normalVars) ./ controlWeightDeltaAnkle
fprintf('less foot placement vars diff from normal\n');
(lessFootPlacementVars  - normalVars) ./ controlWeightDeltaFoot


%%



%%
return


%%
if (plotExampleAngles)
  [xNext, t, y, phase] = onestep(x0, 'interleaveAnimation', 0);
  figure
  indecesToPlot = [1,2,3,4,8]';
  angles = y(:,indecesToPlot);
  % angles(:,)
  
  minY = -1; %min(min(angles));
  maxY = 1; %max(max(angles));
  
  eventTimes = t(find(diff(phase)));
  
  LineWidth = 3;
  LineWidthEvent = LineWidth;
  
  plot(t, angles, 'LineWidth', LineWidth)
  hold on;
  
  plot([eventTimes(1) eventTimes(1)], [minY maxY], '--k', 'LineWidth', LineWidthEvent)
  hold on;
  plot([eventTimes(2) eventTimes(2)], [minY maxY], '--k', 'LineWidth', LineWidthEvent)
  hold on;
  
  fontSize = 20;
  xlabel('time $$\left( \sqrt{\frac{g}{L}} \right) $$', 'Interpreter', 'latex'); %, 'FontSize', fontSize)
  
  % [stanceTheta, swingTheta, ...
  % stanceFootDisplacement, swingFootDisplacement, ...
  % pelvisX, pelvisY, ...
  % stanceKneeAngle, swingKneeAngle];
  
  legend('stance thigh angle', 'swing thigh angle', 'stance leg compression', 'swing leg compression', 'swing knee angle');
end

%%




