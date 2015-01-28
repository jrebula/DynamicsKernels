classdef Constraint
  % Constraint for a HybridDynamicSystem
  %
  % maybe constraints need to have preconditions and post conditions...
  % 
  % so for elastically bouncing a ball,
  % precondition would be 
  % h == 0 
  % post condition would be (hDotNext - hDotPrev) == 0
  % and this is only an approximation that gets corrector as dt -> 0
  %
  % 
  % so for inelastically throwing a ball at the ground,
  % precondition would be 
  % h == 0 
  % post condition would be (hDotNext - hDotPrev) == 0
  %
  %

  
  properties
  end
  
  methods
    
    function [constraintFeasibility] = getFeasibility(this, q, u, uDot)
      %% returns various constraint feasibility metrics, where non zero entries
      % mean the constraint is infeasible and therefore can not be exerted.

      % for a normal inelastic constraint, 
      % returns position, velocity, and acceleration constraint feasibilities
      
      % how do we handle, for example, elastic bouncing? I guess that would
      % be done with the continuous equations of motion?
      
      % constrainError_{j} = (C(x) - Pc) * lambda_i
      % constrainError_{j+1} = (Jc * xDot - PcDot) * lambda_i
      % constrainError_{j+2} = (Jc * xDDot + JcDot xDot - PcDDot) * lambda_i
      
      % there are other requirements introduced by this constraint, for
      % example, you have to tell the optimization the ball can't just
      % drop through the floor with no forces at all. Maybe these requirements can be
      % implemented in general with inequality constraints? 
    end
    
  end
  
end

