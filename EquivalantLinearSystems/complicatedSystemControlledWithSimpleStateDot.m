function [xDot] = complicatedSystemControlledWithSimpleStateDot(t, x, complicatedSystem, simpleSystem, desiredComPosition, comPositionJacobian, comVelocityJacobian, varargin)


%% Parameters:
lqrStateCostWeighting = [1 0; 0 0]; % (position and velocity of simple system's mass)
lqrControlCostWeight = 0.0001;
timeToSwitchDesiredPosition = [];
controlGain = [];

for i = 1:2:length(varargin)
  option = varargin{i};
  value = varargin{i + 1};
  switch option
    case 'lqrStateCostWeighting'
      lqrStateCostWeighting = value;
    case 'lqrControlCostWeight'
      lqrControlCostWeight = value;
    case 'timeToSwitchDesiredPosition'
      timeToSwitchDesiredPosition = value;
    case 'controlGain'
      controlGain = value;
  end
end

A = complicatedSystem.a;
B = complicatedSystem.b;

ASimple = simpleSystem.a;
BSimple = simpleSystem.b;


if ~isempty(timeToSwitchDesiredPosition)
  if t > timeToSwitchDesiredPosition
    desiredComPosition = -desiredComPosition;
  end
end


if (isempty(desiredComPosition))
  xDot = A * x;
  return
end


%% (I) first go from the complex state to the simple state
% simple mass state is com state of complex system:

complexToSimpleStateJacobian = [comPositionJacobian; comVelocityJacobian];
xSimple = complexToSimpleStateJacobian * x;


%% (II) go from the simple state to the desired behavior of the simple system
% control to the desired simple system mass position using LQR:

errorSimple = xSimple - [desiredComPosition; 0];
N = [];
KSimple = lqr(ASimple, BSimple, lqrStateCostWeighting, lqrControlCostWeight, N);
if (~isempty(controlGain))
  KSimple = controlGain;
end
uSimpleDesired = -KSimple * errorSimple;
xDotSimpleDesired = ASimple * xSimple + BSimple * uSimpleDesired;


%% (III) go from simple desired behavior to a complex desired behavior 
% the desired acceleration of the simple system's mass is the desired
% acceleration of the complicated system's com:

xDotComDesired = xDotSimpleDesired;


%% (IV) go from desired behavior of the complex system to the control signal for the complex system
% this is tricky, but here we're trying a form of inverse dynamics, I
% guess:

u = pinv(B) * (pinv([comPositionJacobian; comVelocityJacobian]) * xDotComDesired - A * x);


%% this is the stateDot we need to calculate to integerate the complex system:

xDot = A * x + B * u;


%% Now let's get a measure for how well the simple model does at approximating the complex:

expectedComDot = [comPositionJacobian; comVelocityJacobian] * xDot;
% comDotDesiredMinusExpected = xDotSimpleDesired - expectedComDot;
err = xDotSimpleDesired - expectedComDot;



end

