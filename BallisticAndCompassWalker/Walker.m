classdef Walker
  %Walker generic representation of a walker. Helps integrate dynamics,
  % find limit cycles, etc.
  % To use this class, you make a subclass, and you will have to implement
  % a bunch of methods, depending on what functionality you would like to
  % have from Walker.
  
  properties
  end
  
  methods
    
    function [this] = Walker()
      %%
    end
    
    function [xdot, uDot, constraintForces] = stateDerivative(this, time, x, mode)
      %%
      % mode is the string passed to getConstraintMatrix, it determines the current constraint
      % configuration of the machine (e.g. 'doubleSupport', 'singleSupport', etc.)
      %
      % There are two equations integrated here:
      % MM xDDot - CTranspose lambda = RHS
      % C * xDDot = -CDot xDot
      %
      % where C is the constraint jacobian, of the form C*xDot = 0. The second
      % equation above follows directly from the definition of the constraint
      % jacobian, simply by taking the derivative (C*xDDot + CDot*xDot = 0)
      %
      
      %             fprintf('Walker.stateDerivative: time = %g\n', time)
      
      shouldCheckConstraint = 0; %1; %
      
      [qIndeces, uIndeces] = this.getQAndUIndeces();
      
      % Mass matrix and right hand side
      [MM, rhs] = this.getMassMatrixAndRightHandSide(time, x);
      %       [MM, rhs] = this.getMassMatrixAndRightHandSideContinuous(time, x);
      
      % Constraint matrix
      [Jc, Jcdot] = this.getConstraintMatrix(x, mode);
      %       Jc
      
      [n1, n2] = size(Jc);
      % Solving the equation of motion
      MMnew = [MM -Jc'; Jc zeros(n1,n1)];
      if (numel(Jcdot) > 0)
        
        % Jc * x(uIndeces)
        
        % see, e.g. http://www.cs.ucsb.edu/~cse/Files/Stabilization071993.pdf
        % stabilization of constrained mechanical systems with DAEs and
        % invariant manifolds, Ascher et al
        % particularly eqs 1.5 and 2.1
        baumgarteStabilizationDamper = 0; %10; %1; %
        baumgarteStabilization = -baumgarteStabilizationDamper * Jc * x(uIndeces);
        
        rhsnew = [rhs; -Jcdot * x(uIndeces) + baumgarteStabilization];
      else
        rhsnew = rhs;
      end
      %       cla;
      %       this.plot(x);
      %       drawnow;
      %       pause(0.01);
      %       MM
      accelerationsAndConstraintForces = MMnew \ rhsnew;
      
      
      %       accelerationsAndConstraintForces = pinv(MMnew) * rhsnew;


      uDot = accelerationsAndConstraintForces(qIndeces,1);
      constraintForces = accelerationsAndConstraintForces(uIndeces(1):(uIndeces(1) - 1 + n1), 1);
      
      xdot = [x(uIndeces); uDot];
      
      if (shouldCheckConstraint)
        % Check if constraints are satisfied in acceleration space
        if (numel(Jcdot) > 0)
          constraint = Jc * uDot + Jcdot * x(uIndeces);
        else
          constraint = 0;
        end
        if norm(constraint)>1e-2
          error('Walker.stateDerivative: acceleration constraint failed at t = %g, norm = %g, this seems like a bug in the integrator! Bad!\n', t, norm(constraint));
        end
        
        % and in velocity space
        if (numel(Jcdot) > 0)
          velocityConstraintCheck = Jc * x(uIndeces);
          if (any(abs(velocityConstraintCheck) > 1e-3))
            abs(velocityConstraintCheck)
            fprintf(['the current state you''re trying to find the derivative at does not satisfy the velocity constraints of the given mode (%s).\n' ...
              'You probably want to make sure you''ve transitioned into this mode before you try to do this.\n' ...
              'This is supposed to be a helpful debug error so that you don''t go insane trying to track down why your machine seems to \n' ...
              'be violating constraints. The constraint matrices are not to blame for this, nor is the integrator, it''s probably your initial condition.\n'], mode);
            %             error('debug this');
          end
        end
      end
    end
    
    function [xNext, transitionImpulse] = modeTransition(this, time, state, modeToTransitionTo)
      %%
      % this sets a state to be consistent with a given mode of the machine
      % (so the velocities might be changed so that Jc * xNew = 0). It does
      % not change the position variables. It returns the new state and the
      % generalized impulses that were required to change the old state to
      % the new one. You are responsible for any change of states to
      % account for, e.g., end of swing, events.
      
      [qIndeces, uIndeces] = this.getQAndUIndeces();
      
      [MM, rhs] = this.getMassMatrixAndRightHandSide(time, state);
      
      % Constraint jacobian
      [Jc, Jcdot] = this.getConstraintMatrix(state, modeToTransitionTo);
      [n1, n2] = size(Jc);
      
      MMnew = [MM -Jc';
        Jc zeros(n1,n1)];
      
      %       try
      rhsnew1 = MM * state(uIndeces);
      %       catch e
      %         e
      %       end
      % rhsnew2=(Jc+Jcdot)*initial(7:12);
      
      rhsnew2 = zeros(n1,1);
      rhsnew = [rhsnew1; rhsnew2];
      
      soln = MMnew \ rhsnew;
      xDotNextTick = soln(qIndeces);
      transitionImpulse = soln(uIndeces(1):(n1 + uIndeces(1) - 1));
      
      x(qIndeces) = state(qIndeces);
      x(uIndeces) = xDotNextTick;
      
      xNext = x;
      
      if (sum(isnan(xNext)))
        xNext
        modeToTransitionTo
        error('xNext cannot have any NaNs!')
      end
    end
    
    function [finalState, finalParameters, limitCycleError, ...
        extraInequalityConstraintError, extraEqualityConstraintErrors] = ...
        findLimitCycle(this, initialConditionGuess, varargin)
      %% findLimitCycle of a walker using onestep, can optionally specify:
      %
      % degreesOfFreedomToIgnoreInLimitCycleConstraint (indeces into state)
      %
      % desiredSpeed
      % (walker must define [speed] = calculateSpeed(xInitial, xFinal, tFinal))
      %
      % desiredStepLength
      % (walker must define [stepLength] = calculateStepLength(xInitial, xFinal))
      %
      % additionalConstraintFunction,
      % of the form [c, ceq] = additionalConstraintFunction(xInitial, xNext)
      %
      % parametersToAlter
      % cell array of strings, each is a name of a parameter in this walker
      % which the optimization is allowed to alter to find a limit cycle
      %
      
      degreesOfFreedomToIgnoreInLimitCycleConstraint = [];
      additionalConstraintFunction = [];
      desiredSpeed = [];
      desiredStepLength = [];
      parametersToAlter = {};
      for i = 1 : 2 : length(varargin)
        option = varargin{i};
        value = varargin{i + 1};
        switch option
          case 'degreesOfFreedomToIgnoreInLimitCycleConstraint'
            degreesOfFreedomToIgnoreInLimitCycleConstraint = value;
          case 'additionalConstraintFunction'
            additionalConstraintFunction = value;
          case 'desiredSpeed'
            desiredSpeed = value;
          case 'desiredStepLength'
            desiredStepLength = value;
          case 'parametersToAlter'
            parametersToAlter = value;
        end
      end
      
      constraintFunction = @(x) this.fixedPointConstraint(x, ...
        degreesOfFreedomToIgnoreInLimitCycleConstraint, ...
        additionalConstraintFunction, desiredSpeed, desiredStepLength, parametersToAlter ...
        );
      
      errorFunction = @(x) 0;
      %       if (~isempty(desiredSpeed))
      %         errorFunction = @(x) (desiredSpeed - this.calculateSpeed(x, [])^2);
      %       end
      
      initialCondition = initialConditionGuess;
      if (~isempty(parametersToAlter))
        initialCondition = [initialConditionGuess; this.getParametersFromList(parametersToAlter)];
      end
      
      options = optimset('algorithm', 'sqp', 'display', 'iter', 'DiffMinChange', 1e-4); %, 1e-3); %1e-2); %, 'RelLineSrchBnd', 1e-6); %1e-3); %); %); %
      %             options = optimset('display', 'iter', 'DiffMinChange', 1e-3); %'algorithm', 'sqp',
      
      finalState = fmincon(errorFunction, initialCondition, ...
        [], [], [], [], [], [], constraintFunction, options);
      
      %       errorFunction = @(x) this.fixedPointConstraintError(x, ...
      %         degreesOfFreedomToIgnoreInLimitCycleConstraint, ...
      %         additionalConstraintFunction, desiredSpeed, desiredStepLength, parametersToAlter ...
      %         );
      %       finalState = fminsearch(errorFunction, initialCondition, options);
      
      [tmp, tmp, ...
        limitCycleError, extraInequalityConstraintError, extraEqualityConstraintErrors] = ...
        constraintFunction(finalState);
      
      this.printStepCharacteristics(finalState);
      
      [finalState, finalParameters] = this.separateStatesAndParameters(finalState, parametersToAlter);
    end
    
    function [stepEndStates] = walkNSteps(this, initialState, N, varargin)
      %%
      interleaveAnimation = 0;
      doneFunction = @(x) 0;
      for i = 1 : 2 : length(varargin)
        option = varargin{i};
        value = varargin{i + 1};
        switch option
          case 'interleaveAnimation'
            interleaveAnimation = value;
          case 'doneFunction'
            doneFunction = value;
        end
      end
      
      states = initialState;
      
      %       [nextState]
      [nextState, finalTime, allStates, allTimes, this] = this.oneStep(initialState, 'interleaveAnimation', interleaveAnimation);
      states = [states; nextState];
      
      for i = 1 : ceil(N/2)
        %         [nextState] =
        [nextState, finalTime, allStates, allTimes, this] = this.oneStep(nextState, 'interleaveAnimation', interleaveAnimation, 'plotLegsSwitched', 1);
        states = [states; nextState];
        if (doneFunction(nextState))
          break;
        end
        %         points = this.getKinematicPoints(nextState);
        %         if (points.centerOfMassPosition(3) < 0.51)
        %           break;
        %         end
        
        %         [nextState] =
        [nextState, finalTime, allStates, allTimes, this] = this.oneStep(nextState, 'interleaveAnimation', interleaveAnimation, 'plotLegsSwitched', 0);
        states = [states; nextState];
        if (doneFunction(nextState))
          break;
        end
      end
      stepEndStates = states;
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
    
    function [] = printStepCharacteristics(this, state, degreesOfFreedomToIgnoreInLimitCycleConstraint)
      state = this.getWalkerStateObjectFromVector(state);
      state = state.getVector();
      [finalState, finalTime] = this.oneStep(state);
      
      [c, ceq, limitCycleError, cExtra, ceqExtra] = ...
        this.fixedPointConstraint(state, ...
        degreesOfFreedomToIgnoreInLimitCycleConstraint, ...
        [], [], [], {});
      
      currentSpeed = this.calculateSpeed(state, finalState, finalTime);
      currentStepLength = this.calculateStepLength(state, finalState);
      fprintf('step parmeters: speed: %g, step length = %g, limitCycleError = %g\n', currentSpeed, currentStepLength, norm(limitCycleError));
    end
    
  end
  
  methods (Access = protected)
    
    function [states, parameterValues] = separateStatesAndParameters(this, states, parameterNames)
      parameterValues = [];
      if (~isempty(parameterNames))
        numParams = length(parameterNames);
        parameterValues = states((end-(numParams-1)) : end);
        states = states(1 : (end - numParams));
        %         this = this.setParameterValues(parameterNames, parameterValues);
      end
    end
    
    function [errVal] = ...
        fixedPointConstraintError(this, x0, ...
        degreesOfFreedomToIgnoreInLimitCycleConstraint, ...
        additionalConstraintFunction, ...
        desiredSpeed, desiredStepLength, parametersToAlter)
      [c, ceq, limitCycleError, cExtra, ceqExtra] = ...
        fixedPointConstraint(this, x0, ...
        degreesOfFreedomToIgnoreInLimitCycleConstraint, ...
        additionalConstraintFunction, ...
        desiredSpeed, desiredStepLength, parametersToAlter);
      errVal = norm(ceq);
    end
    
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

