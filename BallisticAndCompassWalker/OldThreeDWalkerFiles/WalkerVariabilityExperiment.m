classdef WalkerVariabilityExperiment
  %WalkerVariabilityExperiment an experiment on a walker and controller,
  % assessing variability of various measures and plotting them
  
  properties
    
    walker;
    initialCondition;
    
    maxNumberOfStepsToWalkAtOnce = 10;
    
    animate = 0;
    
  end
  
  methods
    
    
    function [this] = WalkerVariabilityExperiment
      
      
    end
    
    
    function [variabilityResults] = runVariabilityExperiment(this, A, B, Q, R, controlInputs, ...
        walker, perturbationStandardDeviation, limitCycleTorso)
      %%
      for i = 1 : length(controlInputs)
        input = controlInputs{i};
        variabilityResults.(input) = [];
      end
      variabilityResults.stepLengths = [];
      
      %%
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
      
      %       Q = eye(size(minSys.a));
      %       pelvisAndTorsoGain = 1e-2; %1e-3; %
      %       pelvisAndTorsoDotGain = 1e-3;
      %       Q(1, 1) = pelvisAndTorsoGain;
      %       Q(6, 6) = pelvisAndTorsoGain;
      %       Q(1 + 6, 1 + 6) = pelvisAndTorsoDotGain;
      %       Q(6 + 6, 6 + 6) = pelvisAndTorsoDotGain;
      
      
      % R = 1e5; %1e-5; %1; %
      % R = eye(2) * 1e1; %1e5; %1e0; %1e-5; %1; %
      % R = eye(1) * 1e5; %1e1; %1e0; %1e-5; %1; %
      
      % R = diag([1e5, 1e-1]);
      %       R = diag([1e10, 1e0, 1e0]);
      
      N = [];
      eig(minSys.a);
      [K, s, e] = lqr(minSys, Q, R, N);
      %       K
      
      variabilityResults.controlledEigenvalues = e;
      variabilityResults.controllerGain = K;
      
      %% test controller
      animateLaterallyPerturbedControlledAnkles = 0;
      
      calculateControlFromKAndInitialState = @(initialState, K) -K * subsref(initialState - limitCycleTorso.getVector(), goodStates);
      
      perturber = LateralNoisePerturber;
      perturber.standardDeviationOfNoise = perturbationStandardDeviation;
      perturber = perturber.resetWithSeed(randi(1e4, 1));
      walkerPerturbed = walker;
      walkerPerturbed.controllers{1} = perturber;
      
      figure;
      controlledStates = [limitCycleTorso.getVector()];
      
      outputCalculator = []; %@(x) walkerPerturbed.calculateInputVariablesAtState(x);
      %       error('running walker var experiment')
      stepNumber = 1;
      [nextState, ~, ~, ~, ~, additionalOutputs{stepNumber}] = walkerPerturbed.oneStep(limitCycleTorso, 'interleaveAnimation', this.animate, ...
        'additionalOutputCalculator', outputCalculator);
      stepNumber = stepNumber + 1;
      controlledStates = [controlledStates nextState.getVector()];
      
      variabilityResults.fell = 0;
      for i = 1 : floor(this.maxNumberOfStepsToWalkAtOnce / 2)
        walkerPerturbed = walkerPerturbed.setParametersFromList(controlInputs, calculateControlFromKAndInitialState(nextState.getVector(), K));
        %         fprintf('stanceAnkleTorque = %g, extraSwingTorque = %g, torsoRollTorque = %g\n', ...
        %           walkerPerturbed.stanceAnkleTorque, walkerPerturbed.extraSwingTorque, walkerPerturbed.torsoRollTorque);
        for j = 1 : length(controlInputs)
          input = controlInputs{j};
          variabilityResults.(input) = [variabilityResults.(input) walkerPerturbed.(input)];
        end
        
        [nextState, ~, ~, ~, ~, additionalOutputs{stepNumber}] = walkerPerturbed.oneStep(nextState, 'interleaveAnimation', animateLaterallyPerturbedControlledAnkles, ...
          'additionalOutputCalculator', outputCalculator, 'plotLegsSwitched', 1);
        stepNumber = stepNumber + 1;
        nextState.pelvis.y = -nextState.pelvis.y;
        controlledStates = [controlledStates nextState.getVector()];
        points = walkerPerturbed.getKinematicPoints(nextState);
        stepLength = points.stanceFootContactPoint(1) - points.swingFootContactPoint(1);
        variabilityResults.stepLengths = [variabilityResults.stepLengths stepLength];
        if (points.centerOfMassPosition(3) < 0.51)
          variabilityResults.fell = 1;
          break;
        end
        walkerPerturbed = walkerPerturbed.setParametersFromList(controlInputs, calculateControlFromKAndInitialState(nextState.getVector(), K));
        
        for j = 1 : length(controlInputs)
          input = controlInputs{j};
          variabilityResults.(input) = [variabilityResults.(input) walkerPerturbed.(input)];
        end
        
        %         fprintf('stanceAnkleTorque = %g, extraSwingTorque = %g, torsoRollTorque = %g\n', ...
        %           walkerPerturbed.stanceAnkleTorque, walkerPerturbed.extraSwingTorque, walkerPerturbed.torsoRollTorque);
        [nextState, ~, ~, ~, ~, additionalOutputs{stepNumber}] = walkerPerturbed.oneStep(nextState, 'interleaveAnimation', animateLaterallyPerturbedControlledAnkles, 'plotLegsSwitched', 0, ...
          'additionalOutputCalculator', outputCalculator);
        stepNumber = stepNumber + 1;
        points = walkerPerturbed.getKinematicPoints(nextState);
        stepLength = points.stanceFootContactPoint(1) - points.swingFootContactPoint(1);
        variabilityResults.stepLengths = [variabilityResults.stepLengths stepLength];
        nextState.pelvis.y = -nextState.pelvis.y;
        controlledStates = [controlledStates nextState.getVector()];
        if (points.centerOfMassPosition(3) < 0.51)
          variabilityResults.fell = 1;
          break;
        end
      end
      
      variabilityResults.additionalOutputs = additionalOutputs;
      variabilityResults.controlledStates = controlledStates;
    end
    
    function [outputMeasures, outputMeasureNames] = getOutputMeasures(this, results)
      %%
      outputMeasures = [...
        std(results.stanceAnkleTorque), ...
        std(results.forceBetweenLegs), ...
        std(results.torsoRollTorque), ...
        std(results.stepLengths) ...
        ];
      
      outputMeasureNames = {'stanceAnkleTorqueVar', 'forceBetweenLegsVar', 'torsoRollTorqueVar', 'stepLengthVar'};      
    end

    
    function [] = printResults(this, experimentName, results)
      %%
      [outputMeasures, outputMeasureNames] = this.getOutputMeasures(results);
      fprintf([experimentName ' measures: ']);
      for i = 1 : length(outputMeasures)
        fprintf('%s = %g, ', outputMeasureNames{i}, outputMeasures(i));
      end
      fprintf('fell = %g\n', results.fell);
    end
    
    function [] = plotVariabilityChangesFromNormal(resultsNormal, listOfExperimentNames, listOfExperimentResults)
      %%
      
      fig = GridFigure(length(listOfExperimentNames), 1);
      
      [normalVars, outputMeasureNames] = this.getOutputMeasures(resultsNormal);
      
      xTicks = []; %[1:3];
      yLims = [-1.5 1.5];
      xLims = [0 length(normalVars)] + 0.5;
      
      for i = 1 : length(listOfExperimentNames)

        theseVars = this.getOutputMeasures(listOfExperimentResults{i});
        
        fig = fig.nextAvailableSubplot();
        valuesMinusNorm = theseVars - normalVars;
        valuesMinusNorm = valuesMinusNorm ./ abs(valuesMinusNorm);
        stem(valuesMinusNorm);
        set(gca, 'xtick', xTicks);
        axis off;
        box off;
        xlim(xLims);
        ylim(yLims);
        title(listOfExperimentNames{i});
      end
      
      set(gca, 'xtick', [1:length(outputMeasureNames)]);
      set(gca, 'xticklabel', outputMeasureNames);
      xlabel('deviations');
      
    end
    
  end
  
end

