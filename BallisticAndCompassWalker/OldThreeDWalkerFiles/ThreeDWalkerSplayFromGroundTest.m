classdef ThreeDWalkerSplayFromGroundTest < ThreeDWalker
  % ThreeDWalkerSplayFromGround
  
  properties
    
    ILegSmall = 0.003;
    
    IPelvisX = 0.05; %;
    IPelvisY = 0.03; %;
    IPelvisZ = 0.01; %;
    
    stanceAnkleTorque = 0; %
    alwaysInFlightPhase = 0; %1; %
    
  end
  
  methods (Static)
    
    function [] = test()
      %% test energy conservation during random falling
      
      % Initial conditions chosen by hand to give something that looks like a step, but isn't
      format compact;
      
      initialConditions = ThreeDWalkerSplayFromGroundTestState();
      
      walker = ThreeDWalkerSplayFromGroundTest();
      
      % walker.legMassVerticalOffset = 0;
      % walker.MLeg = 1e-5;
      % walker.ILeg = 1e-5;
      % walker.KSwing = 0.0;
      % walker.KSwing = 0.3;
      % walker.MPelvis = 1e-5; %
      
      initialConditions.stanceAnkle.x = 0;
      initialConditions.stanceAnkle.y = 0;
      initialConditions.stanceAnkle.z = 0;
      
      initialConditions.pelvis.roll = 0;
      initialConditions.pelvis.rollDot = -0.1;
      
      initialConditions.stanceLeg.roll = -0.08;
      initialConditions.stanceLeg.pitch = -0.08;
      initialConditions.stanceLeg.pitchDot = 0.03;
      
      initialConditions.swingLeg.roll = -0.2;
      initialConditions.swingLeg.pitch = 0.08;
      initialConditions.swingLeg.pitchDot = -0.3;
      
      walker.plot(initialConditions);
      
      %%
      % ensure the velocities start consistent with the stance phase
      mode = 'stanceFootRolling';
      [qs, us] = walker.getQAndUIndeces();
      x = initialConditions.getVector();
      x = walker.modeTransition(0, x, mode);
      %       if (norm(walker.getConstraintMatrix(initialConditions, mode) * x(us)') > 1e-6)
      %         error('transition into constrained state failed!');
      %       end
      
      initialConditions = ThreeDWalkerSplayFromGroundTestState(x);
      x = initialConditions.getVector();
      %       if (norm(walker.getConstraintMatrix(initialConditions, mode) * x(us)) > 1e-6)
      %         error('transition into constrained state failed after recreating state object (check constructor?)!');
      %       end
      
      ThreeDWalkerSplay.testEnergyConservation(walker, initialConditions);
      
      %%
      %       walker.groundAngle = 0.1329;
      %       walker.KSwing = 0.0632;
      %       limitCycleHipRollWalker =[ -0.0000   -0.0000    1.0000   -0.1616   -0.3811    0.1657    0.3297    0.1575    0.4117    0.0276    0.1690   -0.4373    0.4462    0.4373    0.2290 0.3716]';
      %
      %       ThreeDWalkerSplay.testEnergyConservation(walker, limitCycleHipRollWalker);
      
    end
    
    function [] = testEnergyConservation(walker, initialConditions)
      %%
      [finalState, finalTime, allStates, allTimes] = walker.oneStep(initialConditions.getVector(), ...
        'interleaveAnimation', 1);
      
      initiaEnergies = walker.getEnergyOfState(allStates(:, 1));
      for i = 1 : (length(allStates) - 1)
        energies = walker.getEnergyOfState(allStates(:, i));
        energies.total
        if abs(energies.total - initiaEnergies.total) > 1e-4
          error('energy conservation failed');
        end
      end
      
    end
    
  end
  
  methods
    
    function [this] = ThreeDWalkerSplayFromGroundTest(input)
      %%
      if (nargin == 0)
        input = [];
      end
      this = this@ThreeDWalker(input);
      
      if (isa(input, 'ThreeDWalkerSplayFromGround'))
        
        this.IPelvisX = input.IPelvisX;
        this.IPelvisY = input.IPelvisY;
        this.IPelvisZ = input.IPelvisZ;
        
        this.ILegSmall = input.ILegSmall;
        this.stanceAnkleTorque = input.stanceAnkleTorque;
        this.alwaysInFlightPhase = input.alwaysInFlightPhase;
      end
      
    end
    
    function [state] = getWalkerStateObjectFromVector(this, stateVector)
      %%
      state = ThreeDWalkerSplayFromGroundTestState(stateVector);
    end
    
    function [value, isTerminal, direction] = swingFootTouchdown(this, t, state)
      %%
      %       points = this.getKinematicPoints(this.getWalkerStateObjectFromVector(state));
      %       value = points.swingFootContactPoint(3) - points.stanceFootContactPoint(3);
      %       isTerminal = 1;
      %       direction = -1;
      value = 1;
      isTerminal = 0;
      direction = 1;
    end
    
    function [value, isTerminal, direction] = fellOrSwingFootTouchdown(this, t, state)
      %%
      value = 1;
      isTerminal = 0;
      direction = 1;
    end
    
    function [finalState, finalTime, allStates, allTimes] = oneStep(this, initialState, varargin)
      %%
      initialState = this.getWalkerStateObjectFromVector(initialState);
      initialState = initialState.getVector();
      
      if (isempty(varargin))
        [finalState, finalTime, allStates, allTimes] = ...
          oneStep@ThreeDWalker(this, initialState);
      else
        [finalState, finalTime, allStates, allTimes] = ...
          oneStep@ThreeDWalker(this, initialState, varargin{:});
      end
    end
    
    function [c, ceq] = limitCycleConstraintsOnInitialPositionAndSplayAngle(this, xInitial, xFinal)
      %%
      %       [c, ceq] = this.limitCycleConstraintsOnInitialPosition(xInitial, xFinal);
      
      xInitial = this.getWalkerStateObjectFromVector(xInitial);
      c = [];
      ceq = [xInitial.stanceAnkle.x; xInitial.stanceAnkle.y; xInitial.stanceAnkle.z];
      
      
      %       c = [c -xInitial.stanceLeg.roll];
      
      %       ceq = [ceq; ...
      %         xInitial.stanceLeg.roll + xInitial.swingLeg.roll; ...
      %         xInitial.stanceLeg.rollDot + xInitial.swingLeg.rollDot; ...
      %         ];
    end
    
    function [initialCondition, limitCycleError, ...
        extraInequalityConstraintError, extraEqualityConstraintErrors] = findLimitCycle(this, initialConditionGuess, varargin)
      %% gets a limit cycle for a 3d walker, uses Walker.findLimitCycle, ignoring appropriate dofs (pelvis x and y),
      % and adds a constaint on the otherwise free initial position of the
      % pelvis to (0, 0, 1). Takes all of the same optional arguments as
      % Walker.findLimitCycle, any args you specify take precedence over
      % the default ones generated by this function.
      
      varargin = [ ...
        {'additionalConstraintFunction', @(xInitial, xFinal) this.limitCycleConstraintsOnInitialPositionAndSplayAngle(xInitial, xFinal)}, ...
        varargin];
      
      [initialCondition, limitCycleError, ...
        extraInequalityConstraintError, extraEqualityConstraintErrors] = ...
        findLimitCycle@ThreeDWalker(this, initialConditionGuess, varargin{:});
    end
    
    function [qs, us] = getQAndUIndeces(this)
      %%
      qs = 1:5;
      us = 6:10;
    end
    
    function [] = setWalkerParamsInCurrentFunction(this)
      %%
      ws = 'caller';
      assignin(ws, 'L', this.L);
      assignin(ws, 'R', this.R);
      assignin(ws, 'MPelvis', this.MPelvis);
      %       assignin(ws, 'IPelvis', this.IPelvis);
      assignin(ws, 'pelvisWidth', this.pelvisWidth);
      assignin(ws, 'MLeg', this.MLeg);
      assignin(ws, 'ILeg', this.ILeg);
      assignin(ws, 'legMassVerticalOffset', this.legMassVerticalOffset);
      assignin(ws, 'legMassForwardOffset', this.legMassForwardOffset);
      assignin(ws, 'splayAngle', this.splayAngle);
      assignin(ws, 'g', this.g);
      assignin(ws, 'groundAngle', this.groundAngle);
      assignin(ws, 'KSwing', this.KSwing);
      assignin(ws, 'lateralPelvisForce', this.lateralPelvisForce);
      
      assignin(ws, 'IPelvisX', this.IPelvisX);
      assignin(ws, 'IPelvisY', this.IPelvisY);
      assignin(ws, 'IPelvisZ', this.IPelvisZ);
      
      assignin(ws, 'ILegSmall', this.ILegSmall);
      assignin(ws, 'stanceAnkleTorque', this.stanceAnkleTorque);
    end
    
    function [points] = getKinematicPoints(this, state, uDot)
      %%
      state.setQsUsAndTrigInCurrentFunction();
      this.setWalkerParamsInCurrentFunction();
      
      points.centerOfMassPosition(1) = q1 + s4*legMassVerticalOffset;
      points.centerOfMassPosition(2) = q2 - c4*s5*legMassVerticalOffset;
      points.centerOfMassPosition(3) = q3 + c4*c5*legMassVerticalOffset;
      
      points.stanceLeg.position(1) = q1 + s4*legMassVerticalOffset;
      points.stanceLeg.position(2) = q2 - c4*s5*legMassVerticalOffset;
      points.stanceLeg.position(3) = q3 + c4*c5*legMassVerticalOffset;
      
      
      points.stanceLeg.R(1,1) = c4*(c5 + (1 - c5)*(c4*c4) + (1 - c5)*(s4*s4)); ...
        points.stanceLeg.R(1,2) = 0; points.stanceLeg.R(1,3) = s4*(c5 + (1 - ...
        c5)*(c4*c4) + (1 - c5)*(s4*s4));
      points.stanceLeg.R(2,1) = s4*s5; points.stanceLeg.R(2,2) = c5; ...
        points.stanceLeg.R(2,3) = -(c4*s5);
      points.stanceLeg.R(3,1) = -(c5*s4); points.stanceLeg.R(3,2) = s5; ...
        points.stanceLeg.R(3,3) = c4*c5;
      
      
      points.stanceFootCenter(1) = q1;
      points.stanceFootCenter(2) = q2;
      points.stanceFootCenter(3) = q3;
      
      
      points.stanceFootContactPoint(1) = q1;
      points.stanceFootContactPoint(2) = q2;
      points.stanceFootContactPoint(3) = q3 - R;
      
      
      points.velStanceFootContactPoint(1) = u1;
      points.velStanceFootContactPoint(2) = u2;
      points.velStanceFootContactPoint(3) = u3;
      
      
      if (nargin > 2)
        u1dot = uDot(1);
        u2dot = uDot(2);
        u3dot = uDot(3);
        u4dot = uDot(4);
        u5dot = uDot(5);
        u6dot = uDot(6);
        u7dot = uDot(7);
        u8dot = uDot(8);
      end
      
    end
    
    function [MM, rhs] = getMassMatrixAndRightHandSide(this, time, state)
      %%
      
      state = this.getWalkerStateObjectFromVector(state);
      
      state.setQsUsAndTrigInCurrentFunction();
      this.setWalkerParamsInCurrentFunction();
      
      for i = 1 : length(this.controllers)
        this.controllers{i}.calculateControlAndSetInCurrentFunction(this, time, state);
      end
      
      MM = zeros(5,5); rhs = zeros(5,1);
      
      % Mass Matrix
      MM(1,1) = MLeg; MM(1,2) = 0; MM(1,3) = 0; MM(1,4) = ...
        c4*c5*legMassVerticalOffset*MLeg; MM(1,5) = 0;
      MM(2,1) = MM(1,2); MM(2,2) = MLeg; MM(2,3) = 0; MM(2,4) = 0; MM(2,5) = ...
        -(c4*c5*legMassVerticalOffset*MLeg);
      MM(3,1) = MM(1,3); MM(3,2) = MM(2,3); MM(3,3) = MLeg; MM(3,4) = ...
        -(s4*legMassVerticalOffset*MLeg); MM(3,5) = ...
        -(c4*s5*legMassVerticalOffset*MLeg);
      MM(4,1) = MM(1,4); MM(4,2) = MM(2,4); MM(4,3) = MM(3,4); MM(4,4) = ...
        ILeg*(c5*c5) + ILegSmall*(c4*c4)*(s5*s5) + ILeg*(s4*s4)*(s5*s5) + ...
        MLeg*(c5*c5)*(legMassVerticalOffset*legMassVerticalOffset) + ...
        MLeg*(s4*s4)*(s5*s5)*(legMassVerticalOffset*legMassVerticalOffset); MM(4,5) = ...
        c4*s4*s5*ILeg - c4*s4*s5*ILegSmall + ...
        c4*s4*s5*MLeg*(legMassVerticalOffset*legMassVerticalOffset);
      MM(5,1) = MM(1,5); MM(5,2) = MM(2,5); MM(5,3) = MM(3,5); MM(5,4) = MM(4,5); ...
        MM(5,5) = ILeg*(c4*c4) + ILegSmall*(s4*s4) + ...
        MLeg*(c4*c4)*(legMassVerticalOffset*legMassVerticalOffset);
      
      % righthand side terms
      rhs(1) = s5*u4*u5*(c4*legMassVerticalOffset*MLeg*(s4*s4) + ...
        legMassVerticalOffset*MLeg*power(c4,3)) + ...
        u4*u4*(s4*(legMassVerticalOffset*MLeg*(c5*c5) + ...
        legMassVerticalOffset*MLeg*(c4*c4)*(s5*s5)) + ...
        legMassVerticalOffset*MLeg*(s5*s5)*power(s4,3)) + g*MLeg*sin(groundAngle);
      rhs(2) = s5*(u5*u5)*(-(c4*legMassVerticalOffset*MLeg*(s4*s4)) - ...
        legMassVerticalOffset*MLeg*power(c4,3)) + ...
        u4*u5*(s4*(-(legMassVerticalOffset*MLeg*(c5*c5)) - ...
        legMassVerticalOffset*MLeg*(c4*c4)*(s5*s5)) - ...
        legMassVerticalOffset*MLeg*(s5*s5)*power(s4,3));
      rhs(3) = cos(groundAngle)*(-(g*MLeg*(c5*c5)) - g*MLeg*(s5*s5)) + ...
        c4*(u4*u4)*(c5*legMassVerticalOffset*MLeg*(s5*s5) + ...
        legMassVerticalOffset*MLeg*power(c5,3)) + ...
        c5*(u5*u5*(c4*legMassVerticalOffset*MLeg*(s4*s4) + ...
        legMassVerticalOffset*MLeg*power(c4,3)) + ...
        s5*u4*u5*(s4*(-(legMassVerticalOffset*MLeg) + ...
        legMassVerticalOffset*MLeg*(c4*c4)) + ...
        legMassVerticalOffset*MLeg*power(s4,3)));
      rhs(4) = s4*g*legMassVerticalOffset*MLeg*cos(groundAngle) + ...
        c4*c5*s4*(u5*u5)*(-ILeg + ILegSmall - ...
        MLeg*(legMassVerticalOffset*legMassVerticalOffset)) + ...
        c5*s5*u4*u5*(ILeg*(c4*c4) - ILegSmall*(c4*c4) + ...
        MLeg*(c4*c4)*(legMassVerticalOffset*legMassVerticalOffset)) + ...
        c4*c5*g*legMassVerticalOffset*MLeg*sin(groundAngle);
      rhs(5) = c4*s5*g*legMassVerticalOffset*MLeg*cos(groundAngle) + ...
        c4*c5*s4*u4*u5*(ILeg - ILegSmall + ...
        MLeg*(legMassVerticalOffset*legMassVerticalOffset)) + ...
        c5*s5*(u4*u4)*(-(ILeg*(c4*c4)) + ILegSmall*(c4*c4) - ...
        MLeg*(c4*c4)*(legMassVerticalOffset*legMassVerticalOffset));
      
    end
    
    function [C, CDot] = getConstraintMatrix(this, state, mode)
      %%
      state = this.getWalkerStateObjectFromVector(state);
      
      state.setQsUsAndTrigInCurrentFunction();
      this.setWalkerParamsInCurrentFunction();
      
      if (this.alwaysInFlightPhase)
        C = []; %constraintJacobianStanceFootRolling;
        CDot = []; %constraintJacobianStanceFootRollingDot;
        return;
      end
      
      switch mode
        
        case 'stanceFootRolling'
          
          constraintJacobianStanceFootRolling(3, 5) = 0;
          constraintJacobianStanceFootRollingDot(3, 5) = 0;
          
          constraintJacobianStanceFootRolling(1,1) = 0; ...
            constraintJacobianStanceFootRolling(1,2) = 1; ...
            constraintJacobianStanceFootRolling(1,3) = 0; ...
            constraintJacobianStanceFootRolling(1,4) = 0; ...
            constraintJacobianStanceFootRolling(1,5) = 0;
          constraintJacobianStanceFootRolling(2,1) = 0; ...
            constraintJacobianStanceFootRolling(2,2) = 0; ...
            constraintJacobianStanceFootRolling(2,3) = 1; ...
            constraintJacobianStanceFootRolling(2,4) = 0; ...
            constraintJacobianStanceFootRolling(2,5) = 0;
          constraintJacobianStanceFootRolling(3,1) = 1; ...
            constraintJacobianStanceFootRolling(3,2) = 0; ...
            constraintJacobianStanceFootRolling(3,3) = 0; ...
            constraintJacobianStanceFootRolling(3,4) = -R; ...
            constraintJacobianStanceFootRolling(3,5) = 0;
          
          
          constraintJacobianStanceFootRollingDot(1,1) = 0; ...
            constraintJacobianStanceFootRollingDot(1,2) = 0; ...
            constraintJacobianStanceFootRollingDot(1,3) = 0; ...
            constraintJacobianStanceFootRollingDot(1,4) = 0; ...
            constraintJacobianStanceFootRollingDot(1,5) = 0;
          constraintJacobianStanceFootRollingDot(2,1) = 0; ...
            constraintJacobianStanceFootRollingDot(2,2) = 0; ...
            constraintJacobianStanceFootRollingDot(2,3) = 0; ...
            constraintJacobianStanceFootRollingDot(2,4) = 0; ...
            constraintJacobianStanceFootRollingDot(2,5) = 0;
          constraintJacobianStanceFootRollingDot(3,1) = 0; ...
            constraintJacobianStanceFootRollingDot(3,2) = 0; ...
            constraintJacobianStanceFootRollingDot(3,3) = 0; ...
            constraintJacobianStanceFootRollingDot(3,4) = 0; ...
            constraintJacobianStanceFootRollingDot(3,5) = 0;
          
          
          C = constraintJacobianStanceFootRolling;
          CDot = constraintJacobianStanceFootRollingDot;
          
        otherwise
          error('unknown mode for walker: %s', mode);
      end
      
    end
    
    function [energies] = getEnergyOfState(this, state)
      %%
      state = this.getWalkerStateObjectFromVector(state);
      
      state.setQsUsAndTrigInCurrentFunction();
      this.setWalkerParamsInCurrentFunction();
      
      energies.PE = g*MLeg*(q3*cos(groundAngle) + ...
        c4*c5*legMassVerticalOffset*cos(groundAngle) - q1*sin(groundAngle) - ...
        s4*legMassVerticalOffset*sin(groundAngle));
      
      energies.PEGravity = g*MLeg*(q3*cos(groundAngle) + ...
        c4*c5*legMassVerticalOffset*cos(groundAngle) - q1*sin(groundAngle) - ...
        s4*legMassVerticalOffset*sin(groundAngle));
      
      energies.PESpring = 0;
      
      energies.KE = (MLeg*((s4*u1 + c4*(-(s5*u2) + c5*u3))*(s4*u1 + c4*(-(s5*u2) + c5*u3)) + ...
        (c5*u2 + s5*u3 - (s4*s5*u4 + c4*u5)*legMassVerticalOffset)*(c5*u2 + s5*u3 - ...
        (s4*s5*u4 + c4*u5)*legMassVerticalOffset) + (c4*u1 + s4*s5*u2 + c5*(-(s4*u3) ...
        + u4*legMassVerticalOffset))*(c4*u1 + s4*s5*u2 + c5*(-(s4*u3) + ...
        u4*legMassVerticalOffset))))/2. + ((ILeg*(c5*c5) + (ILegSmall*(c4*c4) + ...
        ILeg*(s4*s4))*(s5*s5))*(u4*u4) + (ILeg*(c4*c4) + ILegSmall*(s4*s4))*(u5*u5) + ...
        s5*u4*u5*(ILeg - ILegSmall)*sin(2*q4))/2.;
      
      energies.total = energies.KE + energies.PE;
    end
    
  end
  
end

