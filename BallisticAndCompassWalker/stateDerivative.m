function [xdot, lambda] = stateDerivative(t, x, mode)
%%
% mode is the string passed to getJc, it determines the current constraint
% configuration of the machine (e.g. 'doubleSupport', 'singleSupport', etc.)
%
% There are two equations integrated here:
% MM xDDot - CTranspose lambda = RHS
% C * xDDot = -C xDot
%
% where C is the constraint jacobian, of the form C*xDot = 0. The second
% equation above follows directly from the definition of the constraint
% jacobian, simply by taking the derivative (X*xDDot + CDot*xDot = 0)
%

% global step

global stateDerivativeTime stateDerivativeMode startOnestepClock
stateDerivativeTime = t;
stateDerivativeMode = mode;

% simulationTimeLimit = 5; %1.5; %0.01; %
% if (toc(startOnestepClock) > simulationTimeLimit)
%   fprintf('onestep simulation took longer than %g seconds\n', simulationTimeLimit);
%   error('onestep simulation took longer than %g seconds', simulationTimeLimit);
% end

shouldCheckConstraint = 1;

qIndeces = 1:8;
uIndeces = 9:16;

% Mass Matrix
MM = getMM(x);

% righthand side terms
rhs = getrhs(x);

% Constraint matrices
[Jc, Jcdot] = getJc(x, mode);

[n1, n2] = size(Jc);
% Solving the equation of motion
MMnew = [MM -Jc'; Jc zeros(n1,n1)];
if(numel(Jcdot) > 0)
  rhsnew = [rhs; -Jcdot * x(uIndeces)];
else
  rhsnew = rhs;
end
temp = MMnew\rhsnew;
udot = temp(qIndeces,1);
% if(nargout > 1)
lambda = temp(uIndeces(1):(uIndeces(1) - 1 + n1), 1);
% end
xdot = [x(uIndeces); udot];

% Call a function to calculate and store reaction forces
% if(shouldCalculateReactionForces)
%   getreactionforces(Jc, lambda); %, t, 1);
% end

if(shouldCheckConstraint)
  % Check if constraints are satisfied initially
  if(numel(Jcdot) > 0)
    constraint = Jc * udot + Jcdot * x(uIndeces);
  else
    constraint = 0;
  end
  
  %   constraintNorm(step, 1) = norm(constraint);
  %   constraintNorm(step, 2) = t;
  if norm(constraint)>1e-2
    %     fprintf('Constraints not satisfied during simulation at t = %g, norm = %g\n', t, norm(constraint));
    %     display(constraint);
    error('constraint failed at t = %g, norm = %g\n', t, norm(constraint));
    return;
  end
end

% step=step+1;

end



