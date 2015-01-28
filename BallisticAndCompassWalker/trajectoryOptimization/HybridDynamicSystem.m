classdef (Abstract) HybridDynamicSystem
  % DynamicSystem I don't know how many times I've written this
  % abstraction
  
  properties
    %     controller = @(x) [];
    constraints = {};
  end
  
  %   properties (Abstract)
  %     allModes; % cell array of strings listing the possible modes of the system
  %   end
  
  methods (Abstract)
    
    % returns two lists, specifying indeces of the position and velocities
    % in the continuous state vector
    [qIndeces, uIndeces] = getQAndUIndeces(this);
    
    [MM, rhs] = getMassMatrixAndRightHandSide(this, time, x, input);
    [Jc, JcDot] = getConstraintMatrix(this, x);
    [state] = convertToStateObject(this, stateOrVector);
    
  end
  
  methods
    
    function [stateError, constraintError, allTimes, allStates] = getDynamicConstraintViolations( ...
        this, initialState, finalState, timeRange, inputs, constraintForces)
      %%
      % There are two equations that are usually integrated:
      % MM xDDot - JcTranspose lambda = rhs
      % Jc * xDDot = -JcDot xDot
      %
      % where Jc is the constraint jacobian, of the form Jc*xDot = 0. The second
      % equation above follows directly from the definition of the constraint
      % jacobian, simply by taking the derivative (Jc*xDDot + JcDot*xDot = 0)
      %
      % to calculate stateError,
      % xFinalProposed = double integral of:
      % MM xDDot - JcTranspose lambda = rhs(inputs), then
      % stateError = xFinalProposed - finalState
      %
      % to calculate constraintError, there are several things that must be
      % satisfied if a given constraint forces is nonzero,
      % position, velocity, and acceleration conditions:
      % C(x) = Pc
      % Jc * xDot = PcDot
      % Jc * xDDot + JcDot xDot = PcDDot
      %
      % we should probably only check this condition at one point in the
      % interval, otherwise the constraints will never be exactly
      % feasible. In fact, we probably must check this condition at the end
      % of the tick, since it's possible the constraint force must be
      % active for one tick to ensure the velocity constraint part is
      % feasible (think of throwing a ball onto ice)
      %
      % Assume we have grouped constraint forces based on conditions. For
      % each constraint force lambda_i in the group, add elements to
      % constraintError:
      % constrainError_{j} = (C(x) - Pc) * lambda_i
      % constrainError_{j+1} = (Jc * xDot - PcDot) * lambda_i
      % constrainError_{j+2} = (Jc * xDDot + JcDot xDot - PcDDot) * lambda_i
      % 
      % and these are all for x, xDot, xDDot @ t = say 0
      
      % first integrate state throughout time period:
      [allTimes, allStates] = integrateEquation(@(time, x) ...
        this.stateDerivativeGivenConstraintForcesAndInputs(time, x, constraintForces, inputs), ...
        timeRange);
      stateError = allStates(end, :) - finalState;
      
      [xdot, uDot] = stateDerivativeGivenConstraintForcesAndInputs( ...
        this, time, x, constraintForces, inputs);
      [qIndeces, uIndeces] = this.getQAndUIndeces();
      q = x(qIndeces);
      u = x(uIndeces);
      allConstraintFeasibilities = [];
      for constraintNumber = 1 : length(this.constraints)
        constraint = this.constraints{constraintNumber};
        [constrantFeasibility] = constraint.getFeasibility(q, u, uDot);
        allConstraintFeasibilities = [allConstraintFeasibilities constrantFeasibility];
      end
      
      [MM, rhs] = this.getMassMatrixAndRightHandSide(time, x);
      [Jc, Jcdot] = this.getConstraintMatrix(x, mode);
      
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
      accelerationsAndConstraintForces = MMnew \ rhsnew;
      % accelerationsAndConstraintForces = pinv(MMnew) * rhsnew;

      uDot = accelerationsAndConstraintForces(qIndeces,1);
      constraintForces = accelerationsAndConstraintForces(uIndeces(1):(uIndeces(1) - 1 + n1), 1);
      
      xdot = [x(uIndeces); uDot];
      
      if (shouldCheckConstraint)
        % Check if constraints are satisfied in acceleration space
          constraint = Jc * uDot + Jcdot * x(uIndeces);
        % and in velocity space
          velocityConstraintCheck = Jc * x(uIndeces);
      end
    end
    
    function [xdot, uDot] = stateDerivativeGivenConstraintForcesAndInputs( ...
        this, time, x, constraintForces, inputs)
      %%
      %
      % There are is one set of equations integrated here:
      % MM xDDot - JcTranspose lambda = rhs
      %
      % where Jc is the constraint jacobian, of the form Jc*xDot = 0. 
      % 
      % someday might be able to add the constraint force finding stuff
      % back in, so that the constraint force changes in a dynamically
      % feasible way during this integration, and the constraintForce input
      % becomes part of the integrated state, and we just use the given
      % constraintForce as the initial condition for it. That would be very
      % nice.
      
      % get information specific to this particular dynamic system
      % (implemented by a concrete subclass)
      [qIndeces, uIndeces] = this.getQAndUIndeces();
      [MM, rhs] = this.getMassMatrixAndRightHandSide(time, x, inputs);
      [Jc, Jcdot] = this.getConstraintMatrix(x);
      
      %       MMnew = [MM -Jc'];
      MMnew = MM;
      rhsNew = rhs - (Jc' * constraintForces);
      
      uDot = MMnew \ rhsNew;
      xdot = [x(uIndeces); uDot];
    end
    
    
    function [xdot, uDot, constraintForces] = stateDerivative(this, time, x, mode)
      %%
      % mode is the string passed to getConstraintMatrix, it determines the current constraint
      % configuration of the machine (e.g. 'doubleSupport', 'singleSupport', etc.)
      %
      % There are two equations integrated here:
      % MM xDDot - JcTranspose lambda = rhs
      % Jc * xDDot = -JcDot xDot
      %
      % where Jc is the constraint jacobian, of the form Jc*xDot = 0. The second
      % equation above follows directly from the definition of the constraint
      % jacobian, simply by taking the derivative (Jc*xDDot + JcDot*xDot = 0)
      %
      
      shouldCheckConstraint = 0; %1; %
      
      % get information specific to this particular dynamic system
      % (implemented by a concrete subclass)
      [qIndeces, uIndeces] = this.getQAndUIndeces();
      [MM, rhs] = this.getMassMatrixAndRightHandSide(time, x);
      [Jc, Jcdot] = this.getConstraintMatrix(x, mode);
      
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
      accelerationsAndConstraintForces = MMnew \ rhsnew;
      % accelerationsAndConstraintForces = pinv(MMnew) * rhsnew;

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
          error('DynamicSystem.stateDerivative: acceleration constraint failed at t = %g, norm = %g, this seems like a bug in the integrator! Bad!\n', t, norm(constraint));
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
      
      rhsnew1 = MM * state(uIndeces);
      
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
    
    function [allTimes, allStates, allControlInputs, this] = simulate( ...
        this, initialState, timeToSimulate, varargin)
      %%
      RelTol = 1e-6; %10; %
      AbsTol = 1e-6; %10; %
      tMax = timeToSimulate;
      dt = timeToSimulate / 1e2;
      
      for i = 1 : 2 : length(varargin)
        option = varargin{i};
        value = varargin{i + 1};
        switch option
          case 'dt'
            dt = value;
          case 'RelTol'
            RelTol = value;
          case 'AbsTol'
            AbsTol = value;
        end
      end
      
      initialState = this.convertToStateObject(initialState);
      modeNumber = initialState.getDiscreteStateVector();
      
      %% Ensure initial state is in the desired initial mode:
      
      %       initialState = this.modeTransition(0, initialState.getVector(), allModes{modeNumber});
      %       initialState = this.getWalkerStateObjectFromVector(initialState);
      
      tStart = 0;
      allTimes = tStart;
      %       allStates = initialState.getContinuousStateVector();
      allStates = initialState.getVector();
      %       allPhases = [modeNumber];
      
      %% simulate continuous mode:
%       mode = allModes{modeNumber};
      
      options = odeset('Events', @(t, x) this.fellOrSwingFootTouchdown(t, x), ...
        'RelTol', RelTol, 'AbsTol', AbsTol);
      %       options = odeset('RelTol', RelTol, 'AbsTol', AbsTol);
      
      %       try
      
      [t, newStates] = ode45(@(t, x) this.stateDerivative(t, x, mode), ...
        tStart : dt : tMax, ...
        initialState.getVector(), ....
        options);
      
      %       [t, newStates] = ode23t(@(t, x) this.stateDerivative(t, x, mode), ...
      %         tStart : dt : tMax, ...
      %         initialState.getVector(), ....
      %         options);
      
      %       catch e
      %         e
      %       end
      
      tEnd = t(end);
      initialState = initialState.setFromVector(newStates(end, :)');
      allTimes = [allTimes t'];
      allStates = [allStates newStates'];
      allPhases = [allPhases; ones(length(t),1) * modeNumber];
      tStart = tEnd;
      
      if (interleaveAnimation)
        for i = 1 : interleaveAnimationFrameskip : size(newStates, 1)
          cla;
          thisState = initialState.setFromVector(newStates(i, :));
          this.plot(thisState, 'plotLegsSwitched', plotLegsSwitched);
          %           title(sprintf('in mode %s', mode));
          pause(0.01);
        end
      end
      
      finalTime = tEnd;
      if (tMax - tEnd) <= dt % || (~isConsistent && checkStatesForConsistency)
        fprintf('Walker ends simulation in %s\n',  mode);
        finalState = initialState;
        return;
      end
      
      %% switch states for next step
      initialState = initialState.switchLegs(this);
      finalState = this.getWalkerStateObjectFromVector(this.modeTransition(finalTime, initialState.getVector(), mode));
      
      %       isConsistent = checkStateIsConsistent(x0, mode);
    end
    
  end
  
end

