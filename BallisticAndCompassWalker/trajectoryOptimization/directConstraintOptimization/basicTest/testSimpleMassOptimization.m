function [] = testSimpleMassOptimization()
%%
numStates = 25;
dt = 0.1;

optimizedState = createOptimizedState(numStates);

x0 = optimizedStateToVector(optimizedState);
A = [];
b = [];
Aeq = [];
beq = [];
lb = [];
ub = [];

tic
optimizedVector = fmincon(@(x) errorFunction(x, ''), x0, A, b, Aeq, beq, lb, ub, @(x) allConstraints(x, dt, ''));
toc
optimizedState = createOptimizedState(optimizedVector);
plotOptimizedState(optimizedState);

%% now add a wall

optimizedState = createOptimizedState(numStates, 'contact');
x0 = optimizedStateToVector(optimizedState);

% tic
% optimizedVector = fmincon(@(x) errorFunction(x, 'contact'), x0, A, b, Aeq, beq, lb, ub, @(x) allConstraints(x, dt, 'contact'));
% toc
tic
options = optimset('algorithm', 'sqp');
optimizedVector = fmincon(@(x) errorFunction(x, 'contact'), x0, A, b, Aeq, beq, lb, ub, @(x) allConstraints(x, dt, 'contact'), ...
  options);
toc

% tic
% options = optimset('algorithm', 'interior-point');
% optimizedVector = fmincon(@(x) errorFunction(x, 'contact'), x0, A, b, Aeq, beq, lb, ub, @(x) allConstraints(x, dt, 'contact'), ...
%   options);
% toc

optimizedState = createOptimizedState(optimizedVector, 'contact');
plotOptimizedState(optimizedState);


end

function [c, ce] = allConstraints(vector, dt, type)
%%
[state] = createOptimizedState(vector, type);

[c, ce] = dynamicsConstraints(vector, dt, type);
ce = [ce state.states(:, 1)']; % initial position and velocity are 0
ce = [ce state.states(:, end)' - [1 0]]; % final position and velocity are [1 0]

end

function [c, ce] = dynamicsConstraints(vector, dt, type)
%%
[state] = createOptimizedState(vector, type);

numStates = size(state.states, 2);
thisTicks = 1 : (numStates - 1);
nextTicks = thisTicks + 1;

positions = state.states(1, :);
velocities = state.states(2, :);

integrationConstraints = (positions(thisTicks) - positions(nextTicks)) + dt * velocities(nextTicks);

massTimesAcceleration = (velocities(nextTicks) - velocities(thisTicks));
sumOfForces = state.forces(nextTicks);
if (isfield(state, 'contactForces'))
  sumOfForces = sumOfForces + state.contactForces(nextTicks);
end

newtonsLaw = massTimesAcceleration - dt * sumOfForces;

ce = [integrationConstraints newtonsLaw];
c = [];
if (isfield(state, 'contactForces'))
  distanceBetweenMassAndWall = 1 - positions;
  ce = [ce distanceBetweenMassAndWall .* state.contactForces];
  c = [-distanceBetweenMassAndWall state.contactForces]; %
end

end

function [errorValue] = errorFunction(vector, type)
%%
[state] = createOptimizedState(vector, type);
errorValue = sum(state.forces.^2);
end

function [vect] = optimizedStateToVector(state)
%%
numStates = size(state.states, 2);
i = 0;
vect(i + (1 : numStates)) = state.states(1, :);
i = i + numStates;
vect(i + (1 : numStates)) = state.states(2, :);
i = i + numStates;
vect(i + (1 : numStates)) = state.forces;
i = i + numStates;

if (isfield(state, 'contactForces'))
  vect(i + (1 : numStates)) = state.contactForces;
end

end

function [ret] = createOptimizedState(numStatesOrVector, type)
%%
contact = 0;
if (nargin == 1)
  type = '';
end
if (strcmp('contact', type))
  contact = 1;
end

if (length(numStatesOrVector) == 1)
  ret.states = zeros(2, numStatesOrVector);
  ret.forces = zeros(1, numStatesOrVector);
  if (contact)
    ret.contactForces = zeros(1, numStatesOrVector);
  end
else
  numStates = length(numStatesOrVector) / 3;
  if (contact)
    numStates = length(numStatesOrVector) / 4;
  end
  
  vect = numStatesOrVector;
  i = 0;
  ret.states(1, :) = vect(i + (1 : numStates));
  i = i + numStates;
  ret.states(2, :) = vect(i + (1 : numStates));
  i = i + numStates;
  ret.forces = vect(i + (1 : numStates));
  if (contact)
    i = i + numStates;
    ret.contactForces = vect(i + (1 : numStates));
  end
end
end

function [] = plotOptimizedState(state)
%%
figure('color', 'w');
xs = 1 : length(state.states);
forcesToPlot = [state.forces];
contact = (isfield(state, 'contactForces'));
if contact
  forcesToPlot = [forcesToPlot; state.contactForces];
end
[axs, h1, h2] = plotyy(xs, state.states, xs, forcesToPlot);
set(h1, 'linewidth', 3);
set(h2(1), 'linewidth', 2, 'color', ones(3,1) * 0.5, 'lineStyle', '--');
if contact
  set(h2(2), 'linewidth', 2, 'color', [1 0 0], 'lineStyle', '--');
end
set(axs(2), 'ycolor', ones(3,1) * 0.5);
box off;
end
