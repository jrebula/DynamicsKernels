classdef TrajectoryOptimizer
  % TrajectoryOptimizer

  properties
    errorFunction = @(x) 0;
    equalityConstraints = @(x) 0;
    inequalityConstraints = @(x) 0;
    parametersToAlter = [];
  end
  
  properties (Access = private)
    dynamicSystem;
    
    %     numberOfStates;
    %     numberOfControls;
    %     numberOfControlPoints;
    %     finalTime;
  end
  
  methods
    
    function [this] = TrajectoryOptimizer(dynamicSystem) %numberOfStates, numberOfControls, numberOfControlPoints)
      %%
      this.dynamicSystem = dynamicSystem;
      %       this.numberOfStates = numberOfStates;
      %       this.numberOfControls = numberOfControls;
      %       this.numberOfControlPoints = numberOfControlPoints;
      %       this.finalTime = finalTime;
    end
    
    function [finalState, finalParameters, limitCycleError, ...
        extraInequalityConstraintError, extraEqualityConstraintErrors] = ...
        optimizeTrajectory(this, initialTrajectoryOptimizationState, varargin)
      %% optimizes a trajectory, no optional arguments at the moment.
      %
      %
      % can optionally specify in fields
      %
      % additionalConstraintsFunction,
      % of the form [c, ceq] = constraintFunction(trajectoryOptimizationState)
      %
      % parametersToAlter
      % cell array of strings, each is a name of a parameter in the
      % DynamicSystem which the optimization is allowed to alter to find a 
      % limit cycle
      %
      
      initialCondition = initialTrajectoryOptimizationState.getVector();
      
      options = optimset('algorithm', 'sqp', 'display', 'iter', 'DiffMinChange', 1e-3); %1e-2); %, 'RelLineSrchBnd', 1e-6); %1e-3); %); %); %
      %             options = optimset('display', 'iter', 'DiffMinChange', 1e-3); %'algorithm', 'sqp',
      
      finalState = fmincon(errorFunction, initialCondition, ...
        [], [], [], [], [], [], constraintFunction, options);
      
      [tmp, tmp, ...
        limitCycleError, extraInequalityConstraintError, extraEqualityConstraintErrors] = ...
        constraintFunction(finalState);
      
      this.printStepCharacteristics(finalState);
      
      [finalState, finalParameters] = this.separateStatesAndParameters(finalState, parametersToAlter);
    end
    
    function [parameterValues] = getParametersFromList(this, parameterNames)
      %%
      parameterValues = zeros(length(parameterNames), 1);
      for i = 1 : length(parameterNames)
        parameterValues(i) = this.(parameterNames{i});
      end
    end
    
    function [this] = setParametersFromList(this, parameterNames, parameterValues)
      %%
      for i = 1 : length(parameterNames)
        this.(parameterNames{i}) = parameterValues(i);
      end
    end
    
  end
  
  methods (Access = protected)
    
%     function [states, parameterValues] = separateStatesAndParameters(this, states, parameterNames)
%       parameterValues = [];
%       if (~isempty(parameterNames))
%         numParams = length(parameterNames);
%         parameterValues = states((end-(numParams-1)) : end);
%         states = states(1 : (end - numParams));
%         %         this = this.setParameterValues(parameterNames, parameterValues);
%       end
%     end
    
    function [c, ceq, limitCycleError, cExtra, ceqExtra] = ...
        fixedPointConstraint(this, x0, ...
        degreesOfFreedomToIgnoreInLimitCycleConstraint, ...
        additionalConstraintFunction, ...
        desiredSpeed, desiredStepLength, parametersToAlter)
      %%
      
      x0Original = x0;
      if (~isempty(parametersToAlter))
        [x0, parameterValues] = this.separateStatesAndParameters(x0, parametersToAlter);
        this = this.setParametersFromList(parametersToAlter, parameterValues);
      end
      
      % try
      [xNext, tFinal, allStates, allTimes, this] = this.oneStep(x0);
      % catch exception
      %   exception
      %   ceq = ones(size(x0));
      %   c = 1;
      %   return;
      % end
      
      try
        ceq = xNext.getVector() - x0;
      catch e
        e
      end
      ceq(degreesOfFreedomToIgnoreInLimitCycleConstraint) = [];
      limitCycleError = ceq;
      c = [];
      
      if (~isempty(desiredSpeed))
        ceq = [ceq; desiredSpeed - this.calculateSpeed(x0, xNext.getVector(), tFinal)];
      end
      
      if (~isempty(desiredStepLength))
        ceq = [ceq; desiredStepLength - this.calculateStepLength(x0, xNext.getVector())];
      end
      
      if isempty(additionalConstraintFunction)
        cExtra = [];
        ceqExtra = [];
      else
        [cExtra, ceqExtra] = additionalConstraintFunction(x0, xNext.getVector());
        c = [c; cExtra];
        ceq = [ceq; ceqExtra];
      end
      
      %       c
      %       ceq
      
      %       fprintf('norm(ceq) = %g\n', norm(ceq));
    end
    
  end
  
end

