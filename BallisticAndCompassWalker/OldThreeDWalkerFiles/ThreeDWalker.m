classdef ThreeDWalker < Walker
  % ThreeDWalker
  
  properties
    
    L = 1;
    R = 0.1; %0; %
    
    MPelvis = 0.68;
    IPelvis = 0.05; %100; %
    pelvisWidth = 0.1; %0; %
    
    MLeg = 0.16; %0; %
    ILeg = 0.017; %100; %
    legMassVerticalOffset = 0.5;
    legMassForwardOffset = 0;
    
    splayAngle = 0;
    
    g = 1; %0; %
    groundAngle = 0.05; %0.04; %
    KSwing = 0;
    
    % in the absence of controllers, we have to set default input values:
    lateralPelvisForce = 0; %1; %

    % these controllers can set anything on a per tick basis. Be careful.
    controllers = {};
    
    
    annotatePlotsWithText = 0;
    
  end
  
  methods (Static)
    
    
    function [] = test()
      %% test energy conservation during random falling
      
      % Initial conditions chosen by hand to give something that looks like a step, but isn't
      format compact;
      
      initialConditions = ThreeDWalkerState();
      
      walker = ThreeDWalker();
      walker.plot(initialConditions);
      
      walker.KSwing = 0.1;
      
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
      
      [finalState, finalTime, allStates, allTimes] = walker.oneStep(initialConditions.getVector());
      
      initiaEnergies = walker.getEnergyOfState(allStates(:, 1));
      for i = 1 : length(allStates)
        energies = walker.getEnergyOfState(allStates(:, i))
        if abs(energies.total - initiaEnergies.total) > 1e-3
          error('energy conservation failed');
        end
      end
      
    end
    
  end
  
  methods
    
    function [this] = ThreeDWalker(input)
      %%
      this = this@Walker();
      
      if (nargin == 1 && ~isempty(input))
        this.L = input.L;
        this.R = input.R;
        this.MPelvis = input.MPelvis;
        this.IPelvis = input.IPelvis;
        this.pelvisWidth = input.pelvisWidth;
        this.MLeg = input.MLeg;
        this.ILeg = input.ILeg;
        this.legMassVerticalOffset = input.legMassVerticalOffset;
        this.legMassForwardOffset = input.legMassForwardOffset;
        this.splayAngle = input.splayAngle;
        this.g = input.g;
        this.groundAngle = input.groundAngle;
        this.KSwing = input.KSwing;
        this.lateralPelvisForce = input.lateralPelvisForce;
        this.controllers = input.controllers;
      end
      
    end
    
    function [state] = getWalkerStateObjectFromVector(this, stateVector)
      %%
      state = ThreeDWalkerState(stateVector);
    end
    
    function [value, isTerminal, direction] = swingFootTouchdown(this, t, state)
      %%
      points = this.getKinematicPoints(this.getWalkerStateObjectFromVector(state));
      value = points.swingFootContactPoint(3) - points.stanceFootContactPoint(3);
      isTerminal = 1;
      direction = -1;
    end
    
    function [value, isTerminal, direction] = stanceFootTouchdown(this, t, state)
      %%
      points = this.getKinematicPoints(this.getWalkerStateObjectFromVector(state));
      value = points.stanceFootContactPoint(3) - points.swingFootContactPoint(3);
      isTerminal = 1;
      direction = -1;
    end
    
    function [value, isTerminal, direction] = fell(this, t, state)
      %%
      points = this.getKinematicPoints(this.getWalkerStateObjectFromVector(state));
      value = points.centerOfMassPosition(3) - 0.5;
      isTerminal = 1;
      direction = 0;
    end
    
    function [value, isTerminal, direction] = fellOrSwingFootTouchdown(this, t, state)
      %%
      [value1, isTerminal1, direction1] = this.fell(t, state);
      [value2, isTerminal2, direction2] = this.swingFootTouchdown(t, state);
      
      value = [value1; value2];
      isTerminal = [isTerminal1; isTerminal2];
      direction = [direction1; direction2];
      
      %       fprintf('ThreeDWalker.fellOrSwingFootTouchdown: value = %s\n', sprintf('%g, ', value));
    end
    
    function [value, isTerminal, direction] = fellOrStanceFootTouchdown(this, t, state)
      %%
      [value1, isTerminal1, direction1] = this.fell(t, state);
      [value2, isTerminal2, direction2] = this.stanceFootTouchdown(t, state);
      value = [value1; value2];
      isTerminal = [isTerminal1; isTerminal2];
      direction = [direction1; direction2];
    end
    
    function [this] = setControlBasedOnGains(this, currentState, controller)
      %%
      inputNames = controller.inputNames;
      controlValues = controller.controLaw(currentState);
      for i = 1 : length(inputNames)
        this.(inputNames{i}) = controlValues(i);
      end
    end
    
    function [this] = getLQRControllerToStabilizeSystem(this, currentState, state)
      %%
      inputNames = controller.inputNames;
      controlValues = controller.controLaw(currentState);
      for i = 1 : length(inputNames)
        this.(inputNames{i}) = controlValues(i);
      end
    end
    
    function [A, B] = linearizeOneStepReturnMap(this, initialCondition, inputs, varargin)
      %%
      perturbationAmount = 1e-4;
      
      for i = 1 : 2 : length(varargin)
        option = varargin{i};
        value = varargin{i + 1};
        switch option
          case 'perturbationAmount'
            perturbationAmount = value;
        end
      end
      
      if (isempty(varargin))
        varargin = {'', {}};
      end
      
      %%
      for i = 1 : length(inputs)
        B(i, :) = this.calculateSensitivityOfOneStepToAnInput(initialCondition, inputs{i}, ...
          'perturbationAmount', perturbationAmount, varargin{:})';
      end
      
      A = this.calculateSensitivityOfOneStepToStates(initialCondition, 'perturbationAmount', perturbationAmount, varargin{:});
    end
    
    function [BMatrix] = calculateSensitivityOfOneStepToAnInput(this, initialState, inputName, varargin)
      %%
      perturbationAmount = 1e-4;
      
      for i = 1 : 2 : length(varargin)
        option = varargin{i};
        value = varargin{i + 1};
        switch option
          case 'perturbationAmount'
            perturbationAmount = value;
        end
      end
      
      if (isempty(varargin))
        varargin = {'', {}};
      end
      
      xNextUnperturbed = this.oneStep(initialState, varargin{:});
      this.(inputName) = this.(inputName) + perturbationAmount;
      xNextPerturbed = this.oneStep(initialState, varargin{:});
      
      BMatrix = (xNextPerturbed.getVector() - xNextUnperturbed.getVector()) / perturbationAmount;
    end
    
    function [AMatrix] = calculateSensitivityOfOneStepToStates(this, initialState, varargin)
      %%
      perturbationAmount = 1e-4;
      
      for i = 1 : 2 : length(varargin)
        option = varargin{i};
        value = varargin{i + 1};
        switch option
          case 'perturbationAmount'
            perturbationAmount = value;
        end
      end
      
      %%
      initialState = this.getWalkerStateObjectFromVector(initialState);
      if (isempty(varargin))
        varargin = {'', {}};
      end
      
      x0 = initialState.getVector();
      AMatrix = ones(length(x0)) * NaN;
      xNextUnperturbed = this.oneStep(x0, varargin{:});
      for i = 1 : length(x0)
        x0Perturbed = x0;
        x0Perturbed(i) = x0Perturbed(i) + perturbationAmount;
        xNextPerturbed = this.oneStep(x0Perturbed, varargin{:});
        AMatrix(:, i) = (xNextPerturbed.getVector() - xNextUnperturbed.getVector()) / perturbationAmount;
      end
    end
    
    function [finalState, finalTime, allStates, allTimes, this] = oneStep(this, initialState, varargin)
      %%
      RelTol = 1e-6; %10; %
      AbsTol = 1e-6; %10; %
      tMax = 5; %2; %6;
      dt = 1e-2;
      interleaveAnimation = 0; %1; %
      interleaveAnimationFrameskip = 2;
      plotLegsSwitched = 0;
      
      for i = 1 : 2 : length(varargin)
        option = varargin{i};
        value = varargin{i + 1};
        switch option
          case 'interleaveAnimation'
            interleaveAnimation = value;
          case 'interleaveAnimationFrameskip'
            interleaveAnimationFrameskip = value;
          case 'plotLegsSwitched'
            plotLegsSwitched = value;
          case 'tMax'
            tMax = value;
        end
      end
      
      allModes = {...
        'stanceFootRolling', ...
        'swingFootRolling'};
      modeNumber = 1;
      
      initialState = this.getWalkerStateObjectFromVector(initialState);
      
      %% Ensure initial state is in the desired initial mode:
      
      initialState = this.modeTransition(0, initialState.getVector(), allModes{modeNumber});
      initialState = this.getWalkerStateObjectFromVector(initialState);
      
      %       [satisfied, constraintValues] = this.checkConstraints(initialState, allModes{modeNumber});
      %       if (~satisfied)
      %         fprintf('warning, initial conditions not in the %s mode!', allModes{modeNumber});
      %       end
      
      tStart = 0;
      allTimes = tStart;
      allStates = initialState.getVector();
      allPhases = [modeNumber];
      
      %% simulate continuous mode:
      mode = allModes{modeNumber};
      
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
      
        
        
      end
      
      %       isConsistent = checkStateIsConsistent(x0, mode);
    
    function [energies] = plot(this, state, varargin)
      %%
      
      plotLegsSwitched = 0;
      initialPelvisY = 0;
      aviWriter = [];
      shouldAntiAlias = 0;
      drawCOMs = 1;
      
      for i = 1 : 2 : length(varargin)
        option = varargin{i};
        value = varargin{i + 1};
        switch option
          case 'plotLegsSwitched'
            plotLegsSwitched = value;
          case 'initialPelvisY'
            initialPelvisY = value;
          case 'aviWriter'
            aviWriter = value;
          case 'shouldAntiAlias'
            shouldAntiAlias = value;
          case 'drawCOMs'
            drawCOMs = value;
        end
      end
      
      %%
      state = this.getWalkerStateObjectFromVector(state);
      
      if (plotLegsSwitched)
        %         state = state.switchLegs(this);
        %         state = state.switchLegsNamesNotSigns(this);
        state = state.switchLegsSignsNotNames(this, initialPelvisY);
        this.pelvisWidth = -this.pelvisWidth;
      end
      
      % tic
      points = this.getKinematicPoints(state);
      % toc
      % clarice
      
      plotter = WalkerPlotter;
      
      stanceFoot.R = points.stanceLeg.R;
      stanceFoot.position = points.stanceFootCenter;
      plotter.plotFoot(stanceFoot, this.R);
      
      try
        swingFoot.R = points.swingLeg.R;
        swingFoot.position = points.swingFootCenter;
        plotter.plotFoot(swingFoot, this.R);
      catch e
      end
      
      try
        if (drawCOMs)
          plotter.drawCOMSymbol(points.pelvis);
        end
      catch e
      end
      
      if (drawCOMs)
        plotter.drawCOMSymbol(points.stanceLeg);
      end
      
      try
        if (drawCOMs)
          plotter.drawCOMSymbol(points.swingLeg);
        end
      catch e
      end
      
      try
        plotter.plotSegmentBetweenPoints(points.stanceHipPoint, points.swingHipPoint);
      catch e
      end
      
      try
        plotter.plotSegmentBetweenPoints(points.stanceHipPoint, points.stanceFootCenter);
      catch e
      end
      
      %       plotter.plotSegmentBetweenPoints(points.stanceFootCenter, points.stanceFootContactPoint);
      
      try
        plotter.plotSegmentBetweenPoints(points.swingHipPoint, points.swingFootCenter);
        %         plotter.plotSegmentBetweenPoints(points.swingFootCenter, points.swingFootContactPoint);
      catch e
      end
      
      
      
      %       plotVectorFromPoint(points.swingFoot, points.swingFootSpringForce, 'vectorColor', 'g', 'scaleFactor', 0.02);
      %       plotVectorFromPoint(points.stanceFoot, points.stanceFootSpringForce, 'vectorColor', 'c', 'scaleFactor', 0.02);
      
      
      axis equal;
      
      try
        if (~isnan(points.pelvis.position(1)))
          xlim(points.pelvis.position(1) + [-0.5 0.5]);
          %         ylim(points.pelvis.position(2) + [-0.5 0.5]);
          %         zlim([-0.05, 1]);
          %         xlim([-0.5 1.5]);
        else
          xlim(points.stanceLeg.position(1) + [-0.5 0.5]);
        end
      catch e
        xlim(points.stanceLeg.position(1) + [-0.5 0.5]);
      end
      %
      yLims = [-0.5, 0.5];
      ylim(yLims);
      zlim([-0.05, 1.3]);
      
      %       xlim(points.pelvis.position(1) + [-0.5 0.5]);
      %       ylim(points.pelvis.position(2) + [-0.5 0.5]);
      %       zlim(points.pelvis.position(3) + [-0.5 0.5]);
      
      %       set(gca, 'XGrid', 'on');
      %
      %       set(gca,'GridLineStyle','-')
      
      xLims = points.pelvis.position(1) + [-0.5 0.5];
      
      linesStart = floor(xLims(1) * 5) / 5;
      linesEnd = floor(xLims(2) * 5) / 5;
      %       linesStart = (xLims(1) * 10) / 10;
      %       linesEnd = (xLims(2) * 10) / 10;
      linesToDraw = linesStart : 0.2 : linesEnd;
      %       fprintf('lines to draw = %s\n', sprintf('%g, ', linesToDraw));
      for x = linesToDraw
        %         x
        line([x x], yLims/2, 'color', ones(3,1) * 0.3)
      end
      
      set(gcf, 'color', 'w');
      
      axis off;
      box off;
      
      %       line(xLims)
      
      set(gca, 'ActivePositionProperty', 'OuterPosition');
      set(gca, 'Position', [0, 0, 1, 1]);
      
      
      if (~isempty(aviWriter))
        if (shouldAntiAlias)
          antiAliasedFig = myaa();
          drawnow;
          pause(0.01);
          thisFrame = getframe(antiAliasedFig);
          close(antiAliasedFig);
        else
          thisFrame = getframe(gcf);
        end
        writeVideo(aviWriter, thisFrame);
      end
      
      if (this.annotatePlotsWithText)
        axis on;
        energies = this.getEnergyOfState(state);
        
        try
          [xDot, uDot, lambda] = this.stateDerivative(0, state.getVector(), 'stanceFootRolling');
          
          text(points.pelvis.position(1), points.pelvis.position(2), points.pelvis.position(3) - 0.2, ...
            sprintf(['E = %g, KE = %g, PE = %g, PEg = %g, \npelvis vertical acc. = %g, vep = %g\n' ...
            ...'KLinear = %g, KRot = %g' ...
            ], ...
            energies.total, energies.KE, energies.PE, energies.PEGravity, uDot(3), xDot(3) ..., ...
            ...energies.KELinear, energies.KERotational
            ));
          
          %         text(points.stanceFootContactPoint(1), points.stanceFootContactPoint(2), points.stanceFootContactPoint(3), ...
          %           sprintf('contact point height = %g\nstance foot contact v = %g, %g, %g', ...
          %           points.stanceFootContactPoint(3), points.velStanceFootContactPoint));
          
          text(points.stanceFootContactPoint(1), points.stanceFootContactPoint(2), points.stanceFootContactPoint(3), ...
            sprintf('stance point x,y,z = %s\n', ...
            sprintf('%g, ', points.stanceFootContactPoint))); %, points.velStanceFootContactPoint));
          
          text(points.swingFootContactPoint(1), points.swingFootContactPoint(2), points.swingFootContactPoint(3), ...
            sprintf('swing point x,y,z = %s\n', ...
            sprintf('%g, ', points.swingFootContactPoint))); %, points.velStanceFootContactPoint));
          
          text(points.stanceFootCenter(1), points.stanceFootCenter(2), points.stanceFootCenter(3), ...
            sprintf('stance foot center = %g, %g, %g', points.stanceFootCenter));
          
          text(points.stanceLeg.position(1), points.stanceLeg.position(2), points.stanceLeg.position(3), ...
            sprintf('stance leg com height = %g', points.stanceLeg.position(3)));
        catch e
        end
        
      end
      
    end
    
    function [c, ceq] = limitCycleConstraintsOnInitialPosition(this, xInitial, xFinal)
      %% ensures the initial pelvis is at some nominal position
      c = [];
      ceq = [xInitial(1); xInitial(2); 1 - xInitial(3)];
    end
    
    function [initialCondition, limitCycleError, ...
        extraInequalityConstraintError, extraEqualityConstraintErrors] = findLimitCycle(this, initialConditionGuess, varargin)
      %% gets a limit cycle for a 3d walker, uses Walker.findLimitCycle, ignoring appropriate dofs (pelvis x and y),
      % and adds a constaint on the otherwise free initial position of the
      % pelvis to (0, 0, 1). Takes all of the same optional arguments as
      % Walker.findLimitCycle, any args you specify take precedence over
      % the default ones generated by this function.
      
      varargin = [ ...
        {'degreesOfFreedomToIgnoreInLimitCycleConstraint', [1 2 3]}, ...
        {'additionalConstraintFunction', @(xInitial, xFinal) this.limitCycleConstraintsOnInitialPosition(xInitial, xFinal)}, ...
        varargin];
      
      [initialCondition, limitCycleError, ...
        extraInequalityConstraintError, extraEqualityConstraintErrors] = ...
        findLimitCycle@Walker(this, initialConditionGuess, varargin{:});
    end
    
    function [speed] = calculateSpeed(this, xInitial, xFinal, tFinal)
      %%
      xInitial = this.getWalkerStateObjectFromVector(xInitial);
      if (isempty(xFinal))
        [xFinal, tFinal] = this.oneStep(xInitial);
      end
      xFinal = this.getWalkerStateObjectFromVector(xFinal);
      speed = (xFinal.pelvis.x - xInitial.pelvis.x) / tFinal;
    end
    
    function [stepLength] = calculateStepLength(this, xInitial, xFinal)
      %%
      xInitial = this.getWalkerStateObjectFromVector(xInitial);
      if (isempty(xFinal))
        [xFinal, tFinal] = this.oneStep(xInitial);
      end
      xFinal = this.getWalkerStateObjectFromVector(xFinal);
      stepLength = (xFinal.pelvis.x - xInitial.pelvis.x);
    end
    
    function [] = printStepCharacteristics(this, state)
      %%
      printStepCharacteristics@Walker(this, state, [1 2]);
    end
    
    function [qs, us] = getQAndUIndeces(this)
      %%
      qs = 1:6;
      us = 7:12;
    end
    
    function [] = setWalkerParamsInCurrentFunction(this)
      %%
      ws = 'caller';
      assignin(ws, 'L', this.L);
      assignin(ws, 'R', this.R);
      assignin(ws, 'MPelvis', this.MPelvis);
      assignin(ws, 'IPelvis', this.IPelvis);
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
    end
    
    function [points] = getKinematicPoints(this, state, uDot)
      %%
      state.setQsUsAndTrigInCurrentFunction();
      this.setWalkerParamsInCurrentFunction();
      
      points.pelvis.position(1) = q1;
      points.pelvis.position(2) = q2;
      points.pelvis.position(3) = q3;
      
      
      points.pelvis.R(1,1) = 1; points.pelvis.R(1,2) = 0; points.pelvis.R(1,3) = 0;
      points.pelvis.R(2,1) = 0; points.pelvis.R(2,2) = c4; points.pelvis.R(2,3) = ...
        -s4;
      points.pelvis.R(3,1) = 0; points.pelvis.R(3,2) = s4; points.pelvis.R(3,3) = ...
        c4;
      
      
      points.stanceLeg.position(1) = q1 + c5*legMassForwardOffset - ...
        s5*legMassVerticalOffset*cos(splayAngle);
      points.stanceLeg.position(2) = q2 + (c4*pelvisWidth)/2. - ...
        legMassVerticalOffset*sin(splayAngle);
      points.stanceLeg.position(3) = q3 - s5*legMassForwardOffset + ...
        (s4*pelvisWidth)/2. - c5*legMassVerticalOffset*cos(splayAngle);
      
      
      points.stanceLeg.R(1,1) = c5; points.stanceLeg.R(1,2) = ...
        -(s5*sin(splayAngle)); points.stanceLeg.R(1,3) = s5*cos(splayAngle);
      points.stanceLeg.R(2,1) = 0; points.stanceLeg.R(2,2) = cos(splayAngle); ...
        points.stanceLeg.R(2,3) = sin(splayAngle);
      points.stanceLeg.R(3,1) = -s5; points.stanceLeg.R(3,2) = ...
        -(c5*sin(splayAngle)); points.stanceLeg.R(3,3) = c5*cos(splayAngle);
      
      
      points.swingLeg.position(1) = q1 + c6*legMassForwardOffset - ...
        s6*legMassVerticalOffset*cos(splayAngle);
      points.swingLeg.position(2) = q2 - (c4*pelvisWidth)/2. + ...
        legMassVerticalOffset*sin(splayAngle);
      points.swingLeg.position(3) = q3 - s6*legMassForwardOffset - ...
        (s4*pelvisWidth)/2. - c6*legMassVerticalOffset*cos(splayAngle);
      
      
      points.swingLeg.R(1,1) = c6; points.swingLeg.R(1,2) = s6*sin(splayAngle); ...
        points.swingLeg.R(1,3) = s6*cos(splayAngle);
      points.swingLeg.R(2,1) = 0; points.swingLeg.R(2,2) = cos(splayAngle); ...
        points.swingLeg.R(2,3) = -sin(splayAngle);
      points.swingLeg.R(3,1) = -s6; points.swingLeg.R(3,2) = c6*sin(splayAngle); ...
        points.swingLeg.R(3,3) = c6*cos(splayAngle);
      
      
      points.stanceFootCenter(1) = q1 + s5*(-L + R)*cos(splayAngle);
      points.stanceFootCenter(2) = q2 + (c4*pelvisWidth)/2. + (-L + ...
        R)*sin(splayAngle);
      points.stanceFootCenter(3) = q3 + (s4*pelvisWidth)/2. + c5*(-L + ...
        R)*cos(splayAngle);
      
      
      points.swingFootCenter(1) = q1 + s6*(-L + R)*cos(splayAngle);
      points.swingFootCenter(2) = q2 - (c4*pelvisWidth)/2. - (-L + ...
        R)*sin(splayAngle);
      points.swingFootCenter(3) = q3 - (s4*pelvisWidth)/2. + c6*(-L + ...
        R)*cos(splayAngle);
      
      
      points.stanceFootContactPoint(1) = q1 + s5*(-L + R)*cos(splayAngle);
      points.stanceFootContactPoint(2) = q2 + (c4*pelvisWidth)/2. + (-L + ...
        R)*sin(splayAngle);
      points.stanceFootContactPoint(3) = q3 + (s4*pelvisWidth)/2. - R + c5*(-L + ...
        R)*cos(splayAngle);
      
      
      points.swingFootContactPoint(1) = q1 + s6*(-L + R)*cos(splayAngle);
      points.swingFootContactPoint(2) = q2 - (c4*pelvisWidth)/2. - (-L + ...
        R)*sin(splayAngle);
      points.swingFootContactPoint(3) = q3 - (s4*pelvisWidth)/2. - R + c6*(-L + ...
        R)*cos(splayAngle);
      
      
      points.stanceHipPoint(1) = q1;
      points.stanceHipPoint(2) = q2 + (c4*pelvisWidth)/2.;
      points.stanceHipPoint(3) = q3 + (s4*pelvisWidth)/2.;
      
      
      points.swingHipPoint(1) = q1;
      points.swingHipPoint(2) = q2 - (c4*pelvisWidth)/2.;
      points.swingHipPoint(3) = q3 - (s4*pelvisWidth)/2.;
      
      
      points.comVelocity(1) = u1 - s5*u5*legMassForwardOffset*MLeg*power(2*MLeg + ...
        MPelvis,-1) - c5*u5*legMassVerticalOffset*MLeg*cos(splayAngle)*power(2*MLeg + ...
        MPelvis,-1) - c6*u6*legMassVerticalOffset*MLeg*cos(splayAngle)*power(2*MLeg + ...
        MPelvis,-1) - ...
        s6*u6*legMassForwardOffset*MLeg*(cos(splayAngle)*cos(splayAngle))*power(2*MLeg + MPelvis,-1) - s6*u6*legMassForwardOffset*MLeg*(sin(splayAngle)*sin(splayAngle))*power(2*MLeg + MPelvis,-1); ...
        
      points.comVelocity(2) = u2;
      points.comVelocity(3) = u3 - c5*u5*legMassForwardOffset*MLeg*power(2*MLeg + ...
        MPelvis,-1) + s5*u5*legMassVerticalOffset*MLeg*cos(splayAngle)*power(2*MLeg + ...
        MPelvis,-1) + s6*u6*legMassVerticalOffset*MLeg*cos(splayAngle)*power(2*MLeg + ...
        MPelvis,-1) - ...
        c6*u6*legMassForwardOffset*MLeg*(cos(splayAngle)*cos(splayAngle))*power(2*MLeg + MPelvis,-1) - c6*u6*legMassForwardOffset*MLeg*(sin(splayAngle)*sin(splayAngle))*power(2*MLeg + MPelvis,-1); ...
        
      
      
      points.centerOfMassPosition(1) = c5*legMassForwardOffset*MLeg*power(2*MLeg + ...
        MPelvis,-1) + c6*legMassForwardOffset*MLeg*power(2*MLeg + MPelvis,-1) + ...
        (2*q1*MLeg + q1*MPelvis)*power(2*MLeg + MPelvis,-1) + ...
        cos(splayAngle)*(-(s5*legMassVerticalOffset*MLeg*power(2*MLeg + MPelvis,-1)) ...
        - s6*legMassVerticalOffset*MLeg*power(2*MLeg + MPelvis,-1));
      points.centerOfMassPosition(2) = (2*q2*MLeg + q2*MPelvis)*power(2*MLeg + ...
        MPelvis,-1);
      points.centerOfMassPosition(3) = -(s5*legMassForwardOffset*MLeg*power(2*MLeg ...
        + MPelvis,-1)) - s6*legMassForwardOffset*MLeg*power(2*MLeg + MPelvis,-1) + ...
        (2*q3*MLeg + q3*MPelvis)*power(2*MLeg + MPelvis,-1) + ...
        cos(splayAngle)*(-(c5*legMassVerticalOffset*MLeg*power(2*MLeg + MPelvis,-1)) ...
        - c6*legMassVerticalOffset*MLeg*power(2*MLeg + MPelvis,-1));
      
      
      points.velStanceFootContactPoint(1) = u1 - u5*R + c5*u5*(-L + ...
        R)*cos(splayAngle);
      points.velStanceFootContactPoint(2) = u2 - (s4*u4*pelvisWidth)/2.;
      points.velStanceFootContactPoint(3) = u3 + (c4*u4*pelvisWidth)/2. - s5*u5*(-L ...
        + R)*cos(splayAngle);
      
      
      if (nargin > 2)
        u1dot = uDot(1);
        u2dot = uDot(2);
        u3dot = uDot(3);
        u4dot = uDot(4);
        u5dot = uDot(5);
        u6dot = uDot(6);
        
        points.totalForceOnCOM(1) = 2*u1dot*MLeg + u1dot*MPelvis + ...
          cos(splayAngle)*(s5*MLeg*(-(u5dot*legMassForwardOffset*cos(splayAngle)) + ...
          legMassVerticalOffset*(u5*u5)*(cos(splayAngle)*cos(splayAngle))) + ...
          s6*MLeg*(-(u6dot*legMassForwardOffset*cos(splayAngle)) + ...
          legMassVerticalOffset*(u6*u6)*(cos(splayAngle)*cos(splayAngle)))) + ...
          c5*MLeg*(-(u5dot*legMassVerticalOffset*cos(splayAngle)) - ...
          legMassForwardOffset*(u5*u5)*(cos(splayAngle)*cos(splayAngle)) - ...
          legMassForwardOffset*(u5*u5)*(sin(splayAngle)*sin(splayAngle))) + ...
          c6*MLeg*(-(u6dot*legMassVerticalOffset*cos(splayAngle)) - ...
          legMassForwardOffset*(u6*u6)*(cos(splayAngle)*cos(splayAngle)) - ...
          legMassForwardOffset*(u6*u6)*(sin(splayAngle)*sin(splayAngle))) - ...
          s5*MLeg*sin(splayAngle)*(u5dot*legMassForwardOffset*sin(splayAngle) - ...
          legMassVerticalOffset*cos(splayAngle)*(u5*u5)*sin(splayAngle)) + ...
          s6*MLeg*sin(splayAngle)*(-(u6dot*legMassForwardOffset*sin(splayAngle)) + ...
          legMassVerticalOffset*cos(splayAngle)*(u6*u6)*sin(splayAngle));
        points.totalForceOnCOM(2) = 2*u2dot*MLeg + u2dot*MPelvis + ...
          MLeg*(-(u5dot*legMassForwardOffset*cos(splayAngle)) + ...
          legMassVerticalOffset*(u5*u5)*(cos(splayAngle)*cos(splayAngle)))*sin(splayAngle) - MLeg*(-(u6dot*legMassForwardOffset*cos(splayAngle)) + legMassVerticalOffset*(u6*u6)*(cos(splayAngle)*cos(splayAngle)))*sin(splayAngle) + cos(splayAngle)*(MLeg*(u5dot*legMassForwardOffset*sin(splayAngle) - legMassVerticalOffset*cos(splayAngle)*(u5*u5)*sin(splayAngle)) + MLeg*(-(u6dot*legMassForwardOffset*sin(splayAngle)) + legMassVerticalOffset*cos(splayAngle)*(u6*u6)*sin(splayAngle))); ...
          
        points.totalForceOnCOM(3) = 2*u3dot*MLeg + u3dot*MPelvis + ...
          c6*MLeg*cos(splayAngle)*(-(u6dot*legMassForwardOffset*cos(splayAngle)) + ...
          legMassVerticalOffset*(u6*u6)*(cos(splayAngle)*cos(splayAngle))) - ...
          s5*MLeg*(-(u5dot*legMassVerticalOffset*cos(splayAngle)) - ...
          legMassForwardOffset*(u5*u5)*(cos(splayAngle)*cos(splayAngle)) - ...
          legMassForwardOffset*(u5*u5)*(sin(splayAngle)*sin(splayAngle))) - ...
          s6*MLeg*(-(u6dot*legMassVerticalOffset*cos(splayAngle)) - ...
          legMassForwardOffset*(u6*u6)*(cos(splayAngle)*cos(splayAngle)) - ...
          legMassForwardOffset*(u6*u6)*(sin(splayAngle)*sin(splayAngle))) + ...
          c6*MLeg*sin(splayAngle)*(-(u6dot*legMassForwardOffset*sin(splayAngle)) + ...
          legMassVerticalOffset*cos(splayAngle)*(u6*u6)*sin(splayAngle)) + ...
          c5*(MLeg*cos(splayAngle)*(-(u5dot*legMassForwardOffset*cos(splayAngle)) + ...
          legMassVerticalOffset*(u5*u5)*(cos(splayAngle)*cos(splayAngle))) - ...
          MLeg*sin(splayAngle)*(u5dot*legMassForwardOffset*sin(splayAngle) - ...
          legMassVerticalOffset*cos(splayAngle)*(u5*u5)*sin(splayAngle)));
        
        
        points.totalTorqueAroundCOM(1) = u4dot*IPelvis + ...
          cos(splayAngle)*(-(s5*u5dot*ILeg*sin(splayAngle)) + ...
          s6*u6dot*ILeg*sin(splayAngle));
        points.totalTorqueAroundCOM(2) = u5dot*ILeg*(cos(splayAngle)*cos(splayAngle)) ...
          + u6dot*ILeg*(cos(splayAngle)*cos(splayAngle));
        points.totalTorqueAroundCOM(3) = ...
          cos(splayAngle)*(-(c5*u5dot*ILeg*sin(splayAngle)) + ...
          c6*u6dot*ILeg*sin(splayAngle));
        
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
      
      
      MM = zeros(6,6); rhs = zeros(6,1);
      
      % Mass Matrix
      MM(1,1) = 2*MLeg + MPelvis; MM(1,2) = 0; MM(1,3) = 0; MM(1,4) = 0; MM(1,5) = ...
        -(MLeg*(s5*legMassForwardOffset + c5*legMassVerticalOffset*cos(splayAngle))); ...
        MM(1,6) = -(MLeg*(s6*legMassForwardOffset + ...
        c6*legMassVerticalOffset*cos(splayAngle)));
      MM(2,1) = MM(1,2); MM(2,2) = 2*MLeg + MPelvis; MM(2,3) = 0; MM(2,4) = 0; ...
        MM(2,5) = 0; MM(2,6) = 0;
      MM(3,1) = MM(1,3); MM(3,2) = MM(2,3); MM(3,3) = 2*MLeg + MPelvis; MM(3,4) = ...
        0; MM(3,5) = -(c5*legMassForwardOffset*MLeg) + ...
        s5*legMassVerticalOffset*MLeg*cos(splayAngle); MM(3,6) = ...
        -(c6*legMassForwardOffset*MLeg) + ...
        s6*legMassVerticalOffset*MLeg*cos(splayAngle);
      MM(4,1) = MM(1,4); MM(4,2) = MM(2,4); MM(4,3) = MM(3,4); MM(4,4) = IPelvis + ...
        (MLeg*(pelvisWidth*pelvisWidth))/2.; MM(4,5) = ...
        (c4*MLeg*pelvisWidth*(-(c5*legMassForwardOffset) + ...
        s5*legMassVerticalOffset*cos(splayAngle)))/2.; MM(4,6) = ...
        (c4*MLeg*pelvisWidth*(c6*legMassForwardOffset - ...
        s6*legMassVerticalOffset*cos(splayAngle)))/2.;
      MM(5,1) = MM(1,5); MM(5,2) = MM(2,5); MM(5,3) = MM(3,5); MM(5,4) = MM(4,5); ...
        MM(5,5) = MLeg*(legMassForwardOffset*legMassForwardOffset) + (ILeg + ...
        MLeg*(legMassVerticalOffset*legMassVerticalOffset))*(cos(splayAngle)*cos(splayAngle)); MM(5,6) = 0; ...
        
      MM(6,1) = MM(1,6); MM(6,2) = MM(2,6); MM(6,3) = MM(3,6); MM(6,4) = MM(4,6); ...
        MM(6,5) = MM(5,6); MM(6,6) = MLeg*(legMassForwardOffset*legMassForwardOffset) ...
        + (ILeg + ...
        MLeg*(legMassVerticalOffset*legMassVerticalOffset))*(cos(splayAngle)*cos(splayAngle)); ...
        
      
      % righthand side terms
      
      rhs(1) = MLeg*(c5*legMassForwardOffset - ...
        s5*legMassVerticalOffset*cos(splayAngle))*(u5*u5) + ...
        MLeg*(c6*legMassForwardOffset - ...
        s6*legMassVerticalOffset*cos(splayAngle))*(u6*u6) + g*(2*MLeg + ...
        MPelvis)*sin(groundAngle);
      rhs(2) = lateralPelvisForce;
      rhs(3) = -(g*(2*MLeg + MPelvis)*cos(groundAngle)) - ...
        MLeg*(s5*legMassForwardOffset + ...
        c5*legMassVerticalOffset*cos(splayAngle))*(u5*u5) - ...
        MLeg*(s6*legMassForwardOffset + ...
        c6*legMassVerticalOffset*cos(splayAngle))*(u6*u6);
      rhs(4) = (c4*MLeg*pelvisWidth*(-((s5*legMassForwardOffset + ...
        c5*legMassVerticalOffset*cos(splayAngle))*(u5*u5)) + (s6*legMassForwardOffset ...
        + c6*legMassVerticalOffset*cos(splayAngle))*(u6*u6)))/2.;
      rhs(5) = -(q5*KSwing*cos(splayAngle)) + q6*KSwing*cos(splayAngle) + ...
        (MLeg*(s4*pelvisWidth*(-(c5*legMassForwardOffset) + ...
        s5*legMassVerticalOffset*cos(splayAngle))*(u4*u4) + ...
        2*g*(legMassForwardOffset*cos(q5 + groundAngle) - ...
        legMassVerticalOffset*cos(splayAngle)*sin(q5 + groundAngle))))/2.;
      rhs(6) = q5*KSwing*cos(splayAngle) + (-2*q6*KSwing*cos(splayAngle) + ...
        MLeg*(s4*pelvisWidth*(c6*legMassForwardOffset - ...
        s6*legMassVerticalOffset*cos(splayAngle))*(u4*u4) + ...
        2*g*(legMassForwardOffset*cos(q6 + groundAngle) - ...
        legMassVerticalOffset*cos(splayAngle)*sin(q6 + groundAngle))))/2.;
      
      
      
    end
    
    function [C, CDot] = getConstraintMatrix(this, state, mode)
      %%
      state = this.getWalkerStateObjectFromVector(state);
      
      state.setQsUsAndTrigInCurrentFunction();
      this.setWalkerParamsInCurrentFunction();
      
      %       C = [];
      %       CDot = [];
      %       return;
      
      switch mode
        case 'swingFootRolling'
          
          constraintJacobianSwingFootRolling(3,6) = 0;
          constraintJacobianSwingFootRollingDot(3,6) = 0;
          
          
          constraintJacobianSwingFootRolling(1,1) = 1; ...
            constraintJacobianSwingFootRolling(1,2) = 0; ...
            constraintJacobianSwingFootRolling(1,3) = 0; ...
            constraintJacobianSwingFootRolling(1,4) = 0; ...
            constraintJacobianSwingFootRolling(1,5) = 0; ...
            constraintJacobianSwingFootRolling(1,6) = -R + c6*(-L + R)*cos(splayAngle);
          constraintJacobianSwingFootRolling(2,1) = 0; ...
            constraintJacobianSwingFootRolling(2,2) = 1; ...
            constraintJacobianSwingFootRolling(2,3) = 0; ...
            constraintJacobianSwingFootRolling(2,4) = (s4*pelvisWidth)/2.; ...
            constraintJacobianSwingFootRolling(2,5) = 0; ...
            constraintJacobianSwingFootRolling(2,6) = 0;
          constraintJacobianSwingFootRolling(3,1) = 0; ...
            constraintJacobianSwingFootRolling(3,2) = 0; ...
            constraintJacobianSwingFootRolling(3,3) = 1; ...
            constraintJacobianSwingFootRolling(3,4) = -(c4*pelvisWidth)/2.; ...
            constraintJacobianSwingFootRolling(3,5) = 0; ...
            constraintJacobianSwingFootRolling(3,6) = -(s6*(-L + R)*cos(splayAngle));
          
          
          constraintJacobianSwingFootRollingDot(1,1) = 0; ...
            constraintJacobianSwingFootRollingDot(1,2) = 0; ...
            constraintJacobianSwingFootRollingDot(1,3) = 0; ...
            constraintJacobianSwingFootRollingDot(1,4) = 0; ...
            constraintJacobianSwingFootRollingDot(1,5) = 0; ...
            constraintJacobianSwingFootRollingDot(1,6) = -(s6*u6*(-L + ...
            R)*cos(splayAngle));
          constraintJacobianSwingFootRollingDot(2,1) = 0; ...
            constraintJacobianSwingFootRollingDot(2,2) = 0; ...
            constraintJacobianSwingFootRollingDot(2,3) = 0; ...
            constraintJacobianSwingFootRollingDot(2,4) = (c4*u4*pelvisWidth)/2.; ...
            constraintJacobianSwingFootRollingDot(2,5) = 0; ...
            constraintJacobianSwingFootRollingDot(2,6) = 0;
          constraintJacobianSwingFootRollingDot(3,1) = 0; ...
            constraintJacobianSwingFootRollingDot(3,2) = 0; ...
            constraintJacobianSwingFootRollingDot(3,3) = 0; ...
            constraintJacobianSwingFootRollingDot(3,4) = (s4*u4*pelvisWidth)/2.; ...
            constraintJacobianSwingFootRollingDot(3,5) = 0; ...
            constraintJacobianSwingFootRollingDot(3,6) = -(c6*u6*(-L + ...
            R)*cos(splayAngle));
          
          C = constraintJacobianSwingFootRolling;
          CDot = constraintJacobianSwingFootRollingDot;
          
        case 'stanceFootRolling'
          
          constraintJacobianStanceFootRolling(3,6) = 0;
          constraintJacobianStanceFootRollingDot(3,6) = 0;
          
          constraintJacobianStanceFootRolling(1,1) = 1; ...
            constraintJacobianStanceFootRolling(1,2) = 0; ...
            constraintJacobianStanceFootRolling(1,3) = 0; ...
            constraintJacobianStanceFootRolling(1,4) = 0; ...
            constraintJacobianStanceFootRolling(1,5) = -R + c5*(-L + R)*cos(splayAngle); ...
            constraintJacobianStanceFootRolling(1,6) = 0;
          constraintJacobianStanceFootRolling(2,1) = 0; ...
            constraintJacobianStanceFootRolling(2,2) = 1; ...
            constraintJacobianStanceFootRolling(2,3) = 0; ...
            constraintJacobianStanceFootRolling(2,4) = -(s4*pelvisWidth)/2.; ...
            constraintJacobianStanceFootRolling(2,5) = 0; ...
            constraintJacobianStanceFootRolling(2,6) = 0;
          constraintJacobianStanceFootRolling(3,1) = 0; ...
            constraintJacobianStanceFootRolling(3,2) = 0; ...
            constraintJacobianStanceFootRolling(3,3) = 1; ...
            constraintJacobianStanceFootRolling(3,4) = (c4*pelvisWidth)/2.; ...
            constraintJacobianStanceFootRolling(3,5) = -(s5*(-L + R)*cos(splayAngle)); ...
            constraintJacobianStanceFootRolling(3,6) = 0;
          
          
          constraintJacobianStanceFootRollingDot(1,1) = 0; ...
            constraintJacobianStanceFootRollingDot(1,2) = 0; ...
            constraintJacobianStanceFootRollingDot(1,3) = 0; ...
            constraintJacobianStanceFootRollingDot(1,4) = 0; ...
            constraintJacobianStanceFootRollingDot(1,5) = -(s5*u5*(-L + ...
            R)*cos(splayAngle)); constraintJacobianStanceFootRollingDot(1,6) = 0;
          constraintJacobianStanceFootRollingDot(2,1) = 0; ...
            constraintJacobianStanceFootRollingDot(2,2) = 0; ...
            constraintJacobianStanceFootRollingDot(2,3) = 0; ...
            constraintJacobianStanceFootRollingDot(2,4) = -(c4*u4*pelvisWidth)/2.; ...
            constraintJacobianStanceFootRollingDot(2,5) = 0; ...
            constraintJacobianStanceFootRollingDot(2,6) = 0;
          constraintJacobianStanceFootRollingDot(3,1) = 0; ...
            constraintJacobianStanceFootRollingDot(3,2) = 0; ...
            constraintJacobianStanceFootRollingDot(3,3) = 0; ...
            constraintJacobianStanceFootRollingDot(3,4) = -(s4*u4*pelvisWidth)/2.; ...
            constraintJacobianStanceFootRollingDot(3,5) = -(c5*u5*(-L + ...
            R)*cos(splayAngle)); constraintJacobianStanceFootRollingDot(3,6) = 0;
          
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
      
      energies.PE = (KSwing*((q5 - q6)*(q5 - q6)) - g*(-2*q3*(2*MLeg + ...
        MPelvis)*cos(groundAngle) + 2*q1*(2*MLeg + MPelvis)*sin(groundAngle) + ...
        MLeg*(legMassVerticalOffset*cos(q5 + groundAngle - splayAngle) + ...
        legMassVerticalOffset*cos(q6 + groundAngle - splayAngle) + ...
        legMassVerticalOffset*cos(q5 + groundAngle + splayAngle) + ...
        legMassVerticalOffset*cos(q6 + groundAngle + splayAngle) + ...
        2*legMassForwardOffset*sin(q5 + groundAngle) + 2*legMassForwardOffset*sin(q6 ...
        + groundAngle))))/2.;
      
      energies.PEGravity = -(g*(-2*q3*(2*MLeg + MPelvis)*cos(groundAngle) + 2*q1*(2*MLeg + ...
        MPelvis)*sin(groundAngle) + MLeg*(legMassVerticalOffset*cos(q5 + groundAngle ...
        - splayAngle) + legMassVerticalOffset*cos(q6 + groundAngle - splayAngle) + ...
        legMassVerticalOffset*cos(q5 + groundAngle + splayAngle) + ...
        legMassVerticalOffset*cos(q6 + groundAngle + splayAngle) + ...
        2*legMassForwardOffset*sin(q5 + groundAngle) + 2*legMassForwardOffset*sin(q6 ...
        + groundAngle))))/2.;
      
      energies.PESpring = (KSwing*((q5 - q6)*(q5 - q6)))/2.;
      
      energies.KE = (IPelvis*(u4*u4) + ILeg*(u5*u5 + ...
        u6*u6)*(cos(splayAngle)*cos(splayAngle)))/2. + (MPelvis*(u1*u1 + u2*u2 + ...
        u3*u3) + MLeg*(-2*s5*u1*u5*legMassForwardOffset - s4*u2*u4*pelvisWidth + ...
        c4*u3*u4*pelvisWidth - 2*c5*u1*u5*legMassVerticalOffset*cos(splayAngle) + ...
        2*s5*u3*u5*legMassVerticalOffset*cos(splayAngle) + ...
        c4*s5*u4*u5*legMassVerticalOffset*pelvisWidth*cos(splayAngle) + u1*u1 + u2*u2 ...
        + u3*u3 + (u4*u4*(pelvisWidth*pelvisWidth))/4. - ...
        2*c5*u3*u5*legMassForwardOffset*(cos(splayAngle)*cos(splayAngle)) + ...
        u5*u5*(legMassForwardOffset*legMassForwardOffset)*(cos(splayAngle)*cos(splayAngle)) + u5*u5*(legMassVerticalOffset*legMassVerticalOffset)*(cos(splayAngle)*cos(splayAngle)) - 2*c5*u3*u5*legMassForwardOffset*(sin(splayAngle)*sin(splayAngle)) + u5*u5*(legMassForwardOffset*legMassForwardOffset)*(sin(splayAngle)*sin(splayAngle)) + u4*u5*legMassForwardOffset*pelvisWidth*sin(splayAngle)*(-(s4*cos(splayAngle)) - c4*c5*sin(splayAngle)) - u4*u5*legMassForwardOffset*pelvisWidth*cos(splayAngle)*(c4*c5*cos(splayAngle) - s4*sin(splayAngle))) + MLeg*(-2*s6*u1*u6*legMassForwardOffset + s4*u2*u4*pelvisWidth - c4*u3*u4*pelvisWidth - 2*c6*u1*u6*legMassVerticalOffset*cos(splayAngle) + 2*s6*u3*u6*legMassVerticalOffset*cos(splayAngle) - c4*s6*u4*u6*legMassVerticalOffset*pelvisWidth*cos(splayAngle) + u1*u1 + u2*u2 + u3*u3 + (u4*u4*(pelvisWidth*pelvisWidth))/4. - 2*c6*u3*u6*legMassForwardOffset*(cos(splayAngle)*cos(splayAngle)) + u6*u6*(legMassForwardOffset*legMassForwardOffset)*(cos(splayAngle)*cos(splayAngle)) + u6*u6*(legMassVerticalOffset*legMassVerticalOffset)*(cos(splayAngle)*cos(splayAngle)) - 2*c6*u3*u6*legMassForwardOffset*(sin(splayAngle)*sin(splayAngle)) + u6*u6*(legMassForwardOffset*legMassForwardOffset)*(sin(splayAngle)*sin(splayAngle)) + u4*u6*legMassForwardOffset*pelvisWidth*sin(splayAngle)*(-(s4*cos(splayAngle)) + c4*c6*sin(splayAngle)) + u4*u6*legMassForwardOffset*pelvisWidth*cos(splayAngle)*(c4*c6*cos(splayAngle) + ...
        s4*sin(splayAngle))))/2.;
      
      
      
      
      energies.total = energies.KE + energies.PE;
    end
    
  end
  
end

