function [xDot, uSimpleDesired] = simpleStateDotWithLQRControl(t, x, simpleSystem, desiredComPosition, varargin)



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

ASimple = simpleSystem.a;
BSimple = simpleSystem.b;

if (isempty(desiredComPosition))
  uSimpleDesired = 0.0;
  xDot = ASimple * x;
  return
end


if ~isempty(timeToSwitchDesiredPosition)
  if t > timeToSwitchDesiredPosition
    desiredComPosition = -desiredComPosition;
  end
end


%%
xSimple = x;

errorSimple = xSimple - [desiredComPosition; 0];
N = [];
KSimple = lqr(ASimple, BSimple, lqrStateCostWeighting, lqrControlCostWeight, N);
if (~isempty(controlGain))
  KSimple = controlGain;
end
uSimpleDesired = -KSimple * errorSimple;
xDotSimpleDesired = ASimple * xSimple + BSimple * uSimpleDesired;

xDot = xDotSimpleDesired;



end

