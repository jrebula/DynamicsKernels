classdef InterpolatedNoiseInTime
  % InterpolatedNoiseInTime precalculates a trajectory of noise in time,
  % suitable for use in disturbing dynamic systems integrated by variable
  % time step methods
  
  properties
    standardDeviationOfNoise = 1;
    meanOfNoise = 0;
  end
  
  properties (Access = protected)
    
    timeBetweenRandomControlPoints = 0.01;
    durationOfStreamToGenerateAtOnce = 10;
    
    randomDataPoints = [];
    
    randomStream;
    
  end
  
  methods (Static)
    
    function [this] = createWithNewRandomNoise()
      %%
      this = InterpolatedNoiseInTime();
      this = this.resetWithSeed('shuffle');
    end
    
  end
  
  methods
    
    function [this] = InterpolatedNoiseInTime()
      %%
      this = this.resetWithSeed(0);
    end
    
    function [this] = resetToDefault(this)
      %%
      [this] = this.resetWithSeed(0);
    end

    function [this] = resetWithSeed(this, seed)
      %%
      this.randomDataPoints = [];
      this.randomStream = RandStream('mt19937ar', 'Seed', seed);
      this = this.addASetOfPoints();
    end
    
    function [value] = calculateRandomValueAtTime(this, time)
      %%
      while (time > this.getMaxTimeGenerated())
        this = this.addASetOfPoints();
      end
      value = interp1(this.getTimesGenerated(), this.randomDataPoints, time);
      value = value * this.standardDeviationOfNoise;
      value = value + this.meanOfNoise;
    end
    
  end
  
  methods (Access = protected)
    
    function [maxTime] = getMaxTimeGenerated(this)
      maxTime = (length(this.randomDataPoints) - 1) * this.timeBetweenRandomControlPoints;
    end
    
    function [timesGenerated] = getTimesGenerated(this)
      timesGenerated = (0:(length(this.randomDataPoints) - 1)) * this.timeBetweenRandomControlPoints;
    end
    
    function [this] = addASetOfPoints(this)
      numPointsToGenerate = floor(this.durationOfStreamToGenerateAtOnce / this.timeBetweenRandomControlPoints);
      this.randomDataPoints = [this.randomDataPoints this.randomStream.randn(numPointsToGenerate, 1)];
    end
    
  end
  
end

