classdef LateralNoisePerturber < InterpolatedNoiseInTime
  % LateralNoisePerturber 
  
  methods (Static)
    
    function [this] = createWithNewRandomNoise()
      %%
      this = LateralNoisePerturber();
      this = this.resetWithSeed('shuffle');
    end
    
  end
  
  methods
    
    function [this] = LateralNoisePerturber()
      %%
      this = this.resetWithSeed(0);
    end
    
    function [this] = resetToDefault(this)
      %%
      [this] = this.resetWithSeed(0);
    end

    function [lateralPelvisForce] = calculateControlValues(this, walker, time, state)
      %%
      lateralPelvisForce = this.calculateRandomValueAtTime(time);
    end
    
    function [] = calculateControlAndSetInCurrentFunction(this, walker, time, state)
      %%
      lateralPelvisForce = this.calculateControlValues(walker, time, state);
      assignin('caller', 'lateralPelvisForce', lateralPelvisForce);
    end
    
  end
  
  
end

