classdef ContinuousTrunkController
  % ContinuousTrunkController
  
  properties
    p;
    d;
    desiredTrunkAngleSpline = [];
    desiredTrunkVelocitySpline = [];
  end
  
  methods
    
    function [this] = ContinuousTrunkController(desiredTrunkAngleSpline, desiredTrunkVelocitySpline)
      %%
      this.desiredTrunkAngleSpline = desiredTrunkAngleSpline;
      this.desiredTrunkVelocitySpline = desiredTrunkVelocitySpline;
      this.p = 1;
      this.d = 1;
    end
    
    function [this] = resetToDefault(this)
      %%
    end

    function [torsoRollTorque] = calculateControlValues(this, walker, time, state)
      %%
      desiredRoll = ppval(this.desiredTrunkAngleSpline, time);
      desiredRollDot = ppval(this.desiredTrunkVelocitySpline, time);
      
      state = walker.getWalkerStateObjectFromVector(state);
      torsoRollTorque = this.p * (desiredRoll - state.torso.roll) + ...
        this.d * (desiredRollDot - state.torso.rollDot);
    end
    
    function [] = calculateControlAndSetInCurrentFunction(this, walker, time, state)
      %%
      torsoRollTorque = this.calculateControlValues(walker, time, state);
      assignin('caller', 'torsoRollTorque', torsoRollTorque);
    end
    
  end
  
  
end


