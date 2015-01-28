classdef BasicTestSystem
  
  properties
    
    M = 1;
    inertia = 1;
    L = 1;
    g = 0;
    Lc = 0.5;
    Lp = 0.5;
    contactCircleRadius = 0.4; %5;
    
    dt = 0.01;
    
  end
  
  methods
    
    function [points] = getKinematicPoints(this, state)
      %%
      this.setParamsInCurrentFunction();
      
      q1 = state(1, :); q2 = state(2, :);
      u1 = state(3, :); u2 = state(4, :);
      
      s1 = sin(q1); s2 = sin(q2);
      
      energies.PE = 1.5*g*M - g*M*cos(q1) - g*M*cos(q2);
      
      energies.PEGravity = 1.5*g*M - g*M*cos(q1) - g*M*cos(q2);
      
      energies.KE = (M*(u1*u1))/2. + ((u1*u1) + ...
        (u2*u2))/2. + (M*(u2*u2))/2.;
      
      
      points.pendulumOne.position(1) = -s1;
      points.pendulumOne.position(2) = 0;
      points.pendulumOne.position(3) = -cos(q1);
      
      
      points.pendulumOne.velocity(1) = -(u1*cos(q1));
      points.pendulumOne.velocity(2) = 0;
      points.pendulumOne.velocity(3) = s1*u1;
      
      
      points.pendulumOne.R(1,1) = cos(q1); points.pendulumOne.R(1,2) = 0; ...
        points.pendulumOne.R(1,3) = s1;
      points.pendulumOne.R(2,1) = 0; points.pendulumOne.R(2,2) = 1; ...
        points.pendulumOne.R(2,3) = 0;
      points.pendulumOne.R(3,1) = -s1; points.pendulumOne.R(3,2) = 0; ...
        points.pendulumOne.R(3,3) = cos(q1);
      
      
      points.pendulumTwo.position(1) = 0. - s2;
      points.pendulumTwo.position(2) = 0.;
      points.pendulumTwo.position(3) = 1.5 - cos(q2);
      
      
      points.pendulumTwo.velocity(1) = -(u2*cos(q2));
      points.pendulumTwo.velocity(2) = 0;
      points.pendulumTwo.velocity(3) = s2*u2;
      
      
      points.pendulumTwo.R(1,1) = cos(q2); points.pendulumTwo.R(1,2) = 0; ...
        points.pendulumTwo.R(1,3) = s2;
      points.pendulumTwo.R(2,1) = 0; points.pendulumTwo.R(2,2) = 1; ...
        points.pendulumTwo.R(2,3) = 0;
      points.pendulumTwo.R(3,1) = -s2; points.pendulumTwo.R(3,2) = 0; ...
        points.pendulumTwo.R(3,3) = cos(q2);
      
      
      points.contactPointOnCircle.position(1) = -(s1*contactCircleRadius*power(3.25 ...
        + 3.*cos(q1) - 2*Lc*cos(q1 - q2) - 3.*Lc*cos(q2) + Lc*Lc,-0.5)) + s2*(-Lc + ...
        contactCircleRadius*Lc*power(3.25 + 3.*cos(q1) - 2*Lc*cos(q1 - q2) - ...
        3.*Lc*cos(q2) + Lc*Lc,-0.5));
      points.contactPointOnCircle.position(2) = 0;
      points.contactPointOnCircle.position(3) = 1.5 - ...
        1.5*contactCircleRadius*power(3.25 + 3.*cos(q1) - 2*Lc*cos(q1 - q2) - ...
        3.*Lc*cos(q2) + Lc*Lc,-0.5) - contactCircleRadius*cos(q1)*power(3.25 + ...
        3.*cos(q1) - 2*Lc*cos(q1 - q2) - 3.*Lc*cos(q2) + Lc*Lc,-0.5) + cos(q2)*(-Lc + ...
        contactCircleRadius*Lc*power(3.25 + 3.*cos(q1) - 2*Lc*cos(q1 - q2) - ...
        3.*Lc*cos(q2) + Lc*Lc,-0.5));
      
      
      points.contactPointOnCircle.velocity(1) = ...
        -1.5*u2*contactCircleRadius*power(3.25 + 3.*cos(q1) - 2*Lc*cos(q1 - q2) - ...
        3.*Lc*cos(q2) + Lc*Lc,-0.5) - u2*contactCircleRadius*cos(q1)*power(3.25 + ...
        3.*cos(q1) - 2*Lc*cos(q1 - q2) - 3.*Lc*cos(q2) + Lc*Lc,-0.5) + cos(q2)*(-u2 + ...
        u2*(1 - Lc) + u2*contactCircleRadius*Lc*power(3.25 + 3.*cos(q1) - 2*Lc*cos(q1 ...
        - q2) - 3.*Lc*cos(q2) + Lc*Lc,-0.5));
      points.contactPointOnCircle.velocity(2) = 0.;
      points.contactPointOnCircle.velocity(3) = 0. + ...
        s1*u2*contactCircleRadius*power(3.25 + 3.*cos(q1) - 2*Lc*cos(q1 - q2) - ...
        3.*Lc*cos(q2) + Lc*Lc,-0.5) - s2*(-u2 + u2*(1 - Lc) + ...
        u2*contactCircleRadius*Lc*power(3.25 + 3.*cos(q1) - 2*Lc*cos(q1 - q2) - ...
        3.*Lc*cos(q2) + Lc*Lc,-0.5));
      
      
      points.contactPointOnCircle.R(1,1) = cos(q2); ...
        points.contactPointOnCircle.R(1,2) = 0; points.contactPointOnCircle.R(1,3) = ...
        s2;
      points.contactPointOnCircle.R(2,1) = 0; points.contactPointOnCircle.R(2,2) = ...
        1; points.contactPointOnCircle.R(2,3) = 0;
      points.contactPointOnCircle.R(3,1) = -s2; points.contactPointOnCircle.R(3,2) ...
        = 0; points.contactPointOnCircle.R(3,3) = cos(q2);
      
      
      points.pendulumOneContactPointPosition(1) = -s1;
      points.pendulumOneContactPointPosition(2) = 0;
      points.pendulumOneContactPointPosition(3) = -cos(q1);
      
      
      points.pendulumTwoContactCircleCenterPosition(1) = 0. - s2*Lc;
      points.pendulumTwoContactCircleCenterPosition(2) = 0.;
      points.pendulumTwoContactCircleCenterPosition(3) = 1.5 - Lc*cos(q2);
      
      
      points.circleSurfaceToContactPointDirection(1) = 0. - s1*power(3.25 + ...
        3.*cos(q1) - 2*Lc*cos(q1 - q2) - 3.*Lc*cos(q2) + Lc*Lc,-0.5) + ...
        s2*Lc*power(3.25 + 3.*cos(q1) - 2*Lc*cos(q1 - q2) - 3.*Lc*cos(q2) + ...
        Lc*Lc,-0.5);
      points.circleSurfaceToContactPointDirection(2) = 0.;
      points.circleSurfaceToContactPointDirection(3) = -1.5*power(3.25 + 3.*cos(q1) ...
        - 2*Lc*cos(q1 - q2) - 3.*Lc*cos(q2) + Lc*Lc,-0.5) - cos(q1)*power(3.25 + ...
        3.*cos(q1) - 2*Lc*cos(q1 - q2) - 3.*Lc*cos(q2) + Lc*Lc,-0.5) + ...
        Lc*cos(q2)*power(3.25 + 3.*cos(q1) - 2*Lc*cos(q1 - q2) - 3.*Lc*cos(q2) + ...
        Lc*Lc,-0.5);
      
      
    end
    
    function [qs, us] = getQAndUIndeces(this)
      %%
      qs = 1:2;
      us = 3:4;
    end
    
    function [] = setParamsInCurrentFunction(this)
      %%
      ws = 'caller';
      assignin(ws, 'M', this.M);
      assignin(ws, 'inertia', this.inertia);
      assignin(ws, 'L', this.L);
      assignin(ws, 'g', this.g);
      assignin(ws, 'Lc', this.Lc);
      assignin(ws, 'Lp', this.Lp);
      assignin(ws, 'dt', this.dt);
      assignin(ws, 'contactCircleRadius', this.contactCircleRadius);
    end
    
    function [MM, rhs] = getMassMatrixAndRightHandSide(this, state, inputTorque, contactForce)
      %%
      this.setParamsInCurrentFunction();
      
      q1 = state(1, :); q2 = state(2, :);
      u1 = state(3, :); u2 = state(4, :);
      
      c1 = cos(q1); c2 = cos(q2); s1 = sin(q1); s2 = sin(q2); c1m2 = cos(q1 - q2); s1m2 = sin(q1 - q2);
      
      MM = zeros(2,2, size(state, 2));
      rhs = zeros(2,1, size(state, 2));
      
      % Mass Matrix
      MM(1,1,:) = repmat(inertia + M, size(MM(1,1,:)));
      MM(1,2,:) = 0;
      MM(2,1,:) = MM(1,2,:);
      MM(2,2,:) = repmat(inertia + M, size(MM(1,1,:)));
      
      % righthand side terms
      %       rhs(1, :) = s1*g*M;
      %       rhs(2, :) = - s2*g*M;
      
      % righthand side terms
      rhs(1, :) = inputTorque + s1m2.*contactForce.*Lc.*power(3.25 + 3.*c1 - 2.*c1m2.*Lc - ...
        3.*c2.*Lc + Lc.*Lc,-0.5) + s1.*(-(g.*M) - 1.5.*contactForce.*power(3.25 + 3.*c1 - ...
        2.*c1m2.*Lc - 3.*c2.*Lc + Lc.*Lc,-0.5)) + contactForce.*power(3.25 + 3.*c1 - ...
        2.*c1m2.*Lc - 3.*c2.*Lc + Lc.*Lc,-0.5).*(s1m2.*Lc.*(1 - ...
        contactCircleRadius.*power(3.25 + 3.*c1 - 2.*c1m2.*Lc - 3.*c2.*Lc + Lc.*Lc,-0.5)) ...
        + s1.*(-1.5 + 1.5.*contactCircleRadius.*power(3.25 + 3.*c1 - 2.*c1m2.*Lc - ...
        3.*c2.*Lc + Lc.*Lc,-0.5))) - contactForce.*Lc.*power(3.25 + 3.*c1 - 2.*c1m2.*Lc - ...
        3.*c2.*Lc + Lc.*Lc,-0.5).*(s1m2.*(1 - contactCircleRadius.*power(3.25 + 3.*c1 - ...
        2.*c1m2.*Lc - 3.*c2.*Lc + Lc.*Lc,-0.5)) + s2.*(-1.5 + ...
        1.5.*contactCircleRadius.*power(3.25 + 3.*c1 - 2.*c1m2.*Lc - 3.*c2.*Lc + ...
        Lc.*Lc,-0.5))) + 1.5.*contactForce.*(s1 - s2.*Lc).*power(3.25 + 3.*c1 - 2.*c1m2.*Lc ...
        - 3.*c2.*Lc + Lc.*Lc,-1).*(-contactCircleRadius + power(3.25 + 3.*c1 - 2.*c1m2.*Lc ...
        - 3.*c2.*Lc + Lc.*Lc,0.5));
      rhs(2, :) = 0. + (-1.*s1m2.*contactCircleRadius + ...
        1.5.*s2.*contactCircleRadius).*contactForce.*Lc.*power(3.25 + 3.*c1 - 2.*c1m2.*Lc - ...
        3.*c2.*Lc + Lc.*Lc,-1) - s1m2.*contactForce.*power(3.25 + 3.*c1 - 2.*c1m2.*Lc - ...
        3.*c2.*Lc + Lc.*Lc,-0.5) + s2.*(-(g.*M) + 1.5.*contactForce.*power(3.25 + 3.*c1 - ...
        2.*c1m2.*Lc - 3.*c2.*Lc + Lc.*Lc,-0.5)) - contactForce.*power(3.25 + 3.*c1 - ...
        2.*c1m2.*Lc - 3.*c2.*Lc + Lc.*Lc,-0.5).*(1.5.*s1.*contactCircleRadius.*power(3.25 + ...
        3.*c1 - 2.*c1m2.*Lc - 3.*c2.*Lc + Lc.*Lc,-0.5) + s1m2.*(-1 + Lc - ...
        contactCircleRadius.*Lc.*power(3.25 + 3.*c1 - 2.*c1m2.*Lc - 3.*c2.*Lc + ...
        Lc.*Lc,-0.5))) - 1.5.*contactForce.*power(3.25 + 3.*c1 - 2.*c1m2.*Lc - 3.*c2.*Lc + ...
        Lc.*Lc,-0.5).*(-(s1.*contactCircleRadius.*power(3.25 + 3.*c1 - 2.*c1m2.*Lc - ...
        3.*c2.*Lc + Lc.*Lc,-0.5)) + s2.*(1 + Lc.*(-1 + contactCircleRadius.*power(3.25 + ...
        3.*c1 - 2.*c1m2.*Lc - 3.*c2.*Lc + Lc.*Lc,-0.5))));


    end
    
    function [inputJacobian] = getInputJacobian(this, state)
      %%
      this.setParamsInCurrentFunction();
      inputJacobian = zeros(2, 1, size(state, 2));
      inputJacobian(1,1,:) = 1;
      inputJacobian(2,1,:) = 0;
    end
    
    function [constraintJacobianContact] = getConstraintJacobian(this, state)
      %%
      %       this.setParamsInCurrentFunction();
      %
%             q1 = state(1, :); q2 = state(2, :);
      %       u1 = state(3, :); u2 = state(4, :);
      %
      %       s1 = sin(q1); s2 = sin(q2);
      %
      %       constraintJacobianContact(1,1,:) = 1.5*s1.*power(3.25 + 3.*cos(q1) - 2*Lc*cos(q1 ...
      %         - q2) - 3.*Lc*cos(q2) + Lc*Lc,-1) - Lc*power(3.25 + 3.*cos(q1) - 2*Lc*cos(q1 ...
      %         - q2) - 3.*Lc*cos(q2) + Lc*Lc,-1).*sin(q1 - q2);
      %
      %       constraintJacobianContact(1,2,:) = -1.5*s2*Lc.*power(3.25 + 3.*cos(q1) - ...
      %         2*Lc*cos(q1 - q2) - 3.*Lc*cos(q2) + Lc*Lc,-1) + Lc*power(3.25 + 3.*cos(q1) - ...
      %         2*Lc*cos(q1 - q2) - 3.*Lc*cos(q2) + Lc*Lc,-1).*sin(q1 - q2);
      %
      
      constraintJacobianContact = zeros(1, 2, size(state, 2));
      constraintJacobianContact(1,1,:) = 1;
      constraintJacobianContact(1,2,:) = -1;
      
    end
    
    function [nonPenetrationConstraint] = getNonPenetrationConstraintMatrix(this, state)
      %%
      q1 = state(1, :); q2 = state(2, :);
      u1 = state(3, :); u2 = state(4, :);
      
      s1 = sin(q1); s2 = sin(q2);
      
      this.setParamsInCurrentFunction();
      
      nonPenetrationConstraint = zeros(1, 1, size(state, 2));
      
      %       nonPenetrationConstraint(1,1,:) = (10.5625 - 3.25*contactCircleRadius - ...
      %         19.5*Lc*cos(q2) + 3.*contactCircleRadius*Lc*cos(q2) + Lc*cos(q1 - q2).*(-13. + ...
      %         2.*contactCircleRadius + 12.*Lc*cos(q2) - 4.*(Lc*Lc)) + 6.5*(Lc*Lc) - ...
      %         1.*contactCircleRadius*(Lc*Lc) + cos(q1).*(19.5 - 3.*contactCircleRadius - ...
      %         12.*Lc*cos(q1 - q2) - 18.*Lc*cos(q2) + 6.*(Lc*Lc)) + 9.*(cos(q1).*cos(q1)) + ...
      %         4.*(Lc*Lc)*(cos(q1 - q2).*cos(q1 - q2)) + 9.*(Lc*Lc)*(cos(q2).*cos(q2)) - ...
      %         6.*cos(q2)*power(Lc,3) + power(Lc,4)).*power(3.25 + 3.*cos(q1) - 2.*Lc*cos(q1 ...
      %         - q2) - 3.*Lc*cos(q2) + Lc*Lc,-2);
      
      nonPenetrationConstraint(1,1,:) = (-3.25.*contactCircleRadius + ...
        3.*contactCircleRadius.*Lc.*cos(q2) - 1.*contactCircleRadius.*(Lc.*Lc) + ...
        3.25.*power(3.25 + 3.*cos(q1) - 2.*Lc.*cos(q1 - q2) - 3.*Lc.*cos(q2) + Lc.*Lc,0.5) ...
        - 3.*Lc.*cos(q2).*power(3.25 + 3.*cos(q1) - 2.*Lc.*cos(q1 - q2) - 3.*Lc.*cos(q2) + ...
        Lc.*Lc,0.5) + Lc.*Lc.*power(3.25 + 3.*cos(q1) - 2.*Lc.*cos(q1 - q2) - ...
        3.*Lc.*cos(q2) + Lc.*Lc,0.5) + cos(q1).*(-3.*contactCircleRadius + 3.*power(3.25 ...
        + 3.*cos(q1) - 2.*Lc.*cos(q1 - q2) - 3.*Lc.*cos(q2) + Lc.*Lc,0.5)) + cos(q1 - ...
        q2).*(2.*contactCircleRadius.*Lc - 2.*Lc.*power(3.25 + 3.*cos(q1) - 2.*Lc.*cos(q1 ...
        - q2) - 3.*Lc.*cos(q2) + Lc.*Lc,0.5))).*power(3.25 + 3.*cos(q1) - 2.*Lc.*cos(q1 - ...
        q2) - 3.*Lc.*cos(q2) + Lc.*Lc,-1);


    end
    
    
    function [err] = errorFunction(this, trajectoryState)
      %%
      err = norm(diff(trajectoryState.inputTape)); %0; %norm(trajectoryState.subtickRatios - 0.5);
    end
    
    function [c, ce, allConstraints] = getConstraintMatrices(this, trajectoryState)
      %%
      this.setParamsInCurrentFunction();
      
      [qIndeces, uIndeces] = this.getQAndUIndeces;
      
      qs = trajectoryState.stateTrajectory(qIndeces,:);
      us = trajectoryState.stateTrajectory(uIndeces, :);
      constraintForces = trajectoryState.constraintForceTrajectory;
      inputs = trajectoryState.inputTape;
      
      if (sum(sum(isnan(qs))))
        qs
        error('shouldn''t have nans in the state vector!!');
      end
      
      thisTick = 1 : (trajectoryState.numTicks - 1);
      nextTick = thisTick + 1;
      
      allDts = trajectoryState.getDtsOfTrajectory(dt);
      
      integrationConstraint = qs(:, thisTick) - qs(:, nextTick) + repmat(allDts, [2 1]) .* us(:, nextTick);
      
      [MM, rhs] = this.getMassMatrixAndRightHandSide(trajectoryState.stateTrajectory, trajectoryState.inputTape, trajectoryState.constraintForceTrajectory);
      constraintJacobian = this.getConstraintJacobian(trajectoryState.stateTrajectory);
      inputJacobian = this.getInputJacobian(trajectoryState.stateTrajectory);
      
      %       dynamicsConstraint = [dynamicsConstraint ...
      %         MM * (us(nextTick) - us(thisTick)) - dt * (rhs + inputJacobian * inputs + constraintJacobian' * constraintForces)];
      
      MM = MM(:, :, nextTick);
      constraintJacobian = constraintJacobian(:, :, nextTick);
      constraintJacobianTranspose = permute(constraintJacobian, [2 1 3]);
      inputJacobian = inputJacobian(:, :, nextTick);
      inputsNextTick = inputs(:, nextTick);
      constraintForcesNextTick = constraintForces(:, nextTick);
      rhsNextTick = rhs(:, nextTick);
      
      %%
      accelerations = reshape(us(:, nextTick) - us(:, thisTick), [size(us, 1) 1 length(thisTick)]);
      inputsNextTick = reshape(inputsNextTick, [size(inputsNextTick, 1) 1 length(thisTick)]);
      constraintForcesNextTick = reshape(constraintForcesNextTick, [size(constraintForcesNextTick, 1) 1 length(thisTick)]);
      rhsNextTick = reshape(rhsNextTick, [size(rhsNextTick, 1) 1 length(thisTick)]);
      
      reshapedDts = permute(repmat(allDts, [2, 1]), [1 3 2]);
      
      fMinusMA = sum(MM .* repmat(accelerations, [1 size(MM, 1), 1]), 2) - reshapedDts .* ...
        (rhsNextTick + ...
        sum(inputJacobian .* repmat(inputsNextTick, [size(MM, 1), 1, 1]), 2) + ...
        sum(constraintJacobianTranspose .* repmat(constraintForcesNextTick, [size(MM, 1), 1, 1]), 2));
      
      %%
      [nonPenetrationConstraint] = this.getNonPenetrationConstraintMatrix(trajectoryState.stateTrajectory);
      nonPenetrationConstraintTranpose = permute(nonPenetrationConstraint, [2 1 3]);
      
      constraintForces = reshape(constraintForces, [size(constraintForces, 1) 1 size(nonPenetrationConstraintTranpose, 3)]);
      contactingForceConstraint = ...
        sum(nonPenetrationConstraintTranpose .* repmat(constraintForces, [size(nonPenetrationConstraintTranpose, 1), 1, 1]), 2);
      
      %%
      ce = ...
        [reshape(integrationConstraint, [numel(fMinusMA), 1]);
        reshape(squeeze(fMinusMA), [numel(fMinusMA), 1]); ...
        reshape(squeeze(contactingForceConstraint), [numel(contactingForceConstraint), 1]); ...
        ...trajectoryState.stateTrajectory(2, :)' - pi; ...
        trajectoryState.stateTrajectory(1, end) - pi; ...
        ...trajectoryState.stateTrajectory(3, end); ...
        trajectoryState.stateTrajectory([1:4], 1); ...
        trajectoryState.subtickRatios' - 0.5; ...
        ]; ...trajectoryState.stateTrajectory(:, 1)];
      
        
      allConstraints.integrationConstraint = integrationConstraint;
      allConstraints.fMinusMA = fMinusMA;
      allConstraints.contactingForceConstraint = contactingForceConstraint;
      allConstraints.intialConditions = trajectoryState.stateTrajectory([1:4], 1);
      allConstraints.finalConditions = trajectoryState.stateTrajectory(1, end) - pi;
      
      %%
      %             nonPenetrationConstraint
      c = [ ...
        ...reshape(abs(us), [numel(us), 1]) - 100; ...
        reshape(squeeze(-nonPenetrationConstraint), [numel(nonPenetrationConstraint), 1]); ...
        ...(reshape(squeeze(contactingForceConstraint), [numel(contactingForceConstraint), 1])) .^2 - (0.5)^2; ...
        ...reshape(squeeze(-constraintForces), [numel(constraintForces), 1]); ...
        ];
      
      ...allConstraints.maxInputs = us - 100;
        allConstraints.nonPenetrationConstraint = -nonPenetrationConstraint;
      
%       epsilon = 1e-2;
%       cesToCs = ce.^2 - epsilon^2;
%       
%       ce = [];
%       c = [c; cesToCs];
      
      %       ce(isnan(ce)) = 1;
      %       ce(isinf(ce)) = 1;
      %       c(isnan(c)) = 1;
      %       c(isinf(c)) = 1;
      %
      %       if (sum(sum(isnan(ce))))
      %         fprintf('NaNs, \nintegration constraint: %g\n', sum(sum(isnan(integrationConstraint))));
      %         fprintf('newton''s law constraint: %g\n', sum(sum(isnan(fs))));
      %         fprintf('contacting force constraint: %g\n', sum(sum(isnan(contactingForceConstraint))));
      %         error('shouldn''t have nans in the constraint!!');
      %       end
      
      
      %       integrationConstraint
      %       fs
      %       contactingForceConstraint
      
      %%
      %       this.plotStateTrajectory(trajectoryState);
      
      
    end
    
    function [] = plotConstraints(this, trajectoryState)
      %%
      
      [c, ce, allConstraints] = this.getConstraintMatrices(trajectoryState);
      
      fig = GridFigure(2,3);
      fig = fig.nextAvailableSubplot();
      plot(allConstraints.integrationConstraint');
      ylabel('integrationConstraint');
      
      fig = fig.nextAvailableSubplot();
      plot(reshape(allConstraints.fMinusMA, [2, size(allConstraints.fMinusMA, 3)])');
      ylabel('fMinusMA');

      fig = fig.nextAvailableSubplot();
      plot(reshape(allConstraints.contactingForceConstraint, [1, size(allConstraints.contactingForceConstraint, 3)]));
      ylabel('contactingForceConstraint');
      
      fig = fig.nextAvailableSubplot();
      plot(allConstraints.intialConditions);
      ylabel('intialConditions');
      
      fig = fig.nextAvailableSubplot();
      plot(allConstraints.finalConditions, 'ok');
      ylabel('finalConditions');
      
      %       fig = fig.nextAvailableSubplot();
      %       plot(allConstraints.maxInputs');
      %       ylabel('maxInputs');
      
      fig = fig.nextAvailableSubplot();
      plot(reshape(allConstraints.nonPenetrationConstraint, [1, size(allConstraints.nonPenetrationConstraint, 3)]));
      ylabel('nonPenetrationConstraint');
      
    end
    
    function [] = plotStateTrajectory(this, trajectoryState)
      %%
      for i = 1 : size(trajectoryState.stateTrajectory, 2)
        this.plotState(trajectoryState.stateTrajectory(:, i));
        pause(0.1);
      end
    end
    
    function [] = plotState(this, state)
      %%
      points = this.getKinematicPoints(state);
      plotter = Plotter();
      plotter.drawCOMSymbol(points.pendulumOne);
      plotter.drawCOMSymbol(points.pendulumTwo);
      plotter.plotSegmentBetweenPoints([0 0 0], points.pendulumOne.position);
      plotter.plotSegmentBetweenPoints([0 0 1.5], points.pendulumTwo.position);
      
      plot3(points.pendulumOneContactPointPosition(1), points.pendulumOneContactPointPosition(2), ...
        points.pendulumOneContactPointPosition(3), 'or');
      
      plot3(points.contactPointOnCircle.position(1), points.contactPointOnCircle.position(2), ...
        points.contactPointOnCircle.position(3), 'oc', 'MarkerSize', 14, 'MarkerFaceColor', 'c');
      
      a.position = [0 0 0];
      b.position = points.circleSurfaceToContactPointDirection;
%       plotter.plotVectorFromPoint(a, b);
      
      %      plotter.plotSphere(points.pendulumTwoContactCircleCenterPosition, this.contactCircleRadius);
      %      state
    end
    
  end
  
end

