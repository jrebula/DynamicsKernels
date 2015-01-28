classdef TrajectoryState
  
  properties
    
    numTicks;
    stateTrajectory;
    constraintForceTrajectory;
    inputTape;
    subtickRatios;
    
  end
  
  methods
    
    function [this] = TrajectoryState(numTicks)
      if (mod(numTicks, 2) == 0)
        error('can only have trajectories with odd numbers of states!!');
      end
      this.numTicks = numTicks;
      this.stateTrajectory = zeros(4, numTicks);
      this.constraintForceTrajectory = zeros(1, numTicks);
      this.inputTape = zeros(1, numTicks);
      this.subtickRatios = ones(1, (numTicks - 1) / 2) * 0.5;
      % 
    end
    
    function [vector] = getVector(this)
      vector = reshape([this.stateTrajectory; this.constraintForceTrajectory; this.inputTape], ...
        [1, 6 * this.numTicks]);
      vector = [vector this.subtickRatios];
    end
    
    function [this] = setFromVector(this, vector)
      %%
      this.stateTrajectory = reshape(vector(1 : (4 * this.numTicks)), [4, this.numTicks]);
      i = (4 * this.numTicks);
      this.constraintForceTrajectory = reshape(vector(i + (1 : this.numTicks)), [1, this.numTicks]);
      i = i + this.numTicks;
      this.inputTape = reshape(vector(i + (1 : this.numTicks)), [1, this.numTicks]);
      i = i + this.numTicks;
      this.subtickRatios = reshape(vector(i + (1 : length(this.subtickRatios))), [1, length(this.subtickRatios)]);
    end
    
    function [dts] = getDtsOfTrajectory(this, baseDt)
      %%
      dt = baseDt;
      majorTimes = dt * (1 : (1 + length(this.subtickRatios)));
      majorTimes = majorTimes - dt;
      subTimes = majorTimes(1 : (end-1)) + this.subtickRatios * dt;
      dts = diff(sort([majorTimes subTimes]));
    end
    
  end
  
end

