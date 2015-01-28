classdef ModelMechanismsResults
  
  properties
    outputs;
  end
  
  methods
    
    function [this] = ModelMechanismsResults(outputs)
      %%
      this.outputs = outputs;
    end
    
    function [] = plot(this)
      %%
      controls = this.outputs.additionalOutputs;

      
      for i = 1 : length(controls)
        subplot(3, 1, 1);
        plot(controls{i}.torsoRollAcceleration)
        hold on;
        
        subplot(3, 1, 2);
        plot(controls{i}.centerOfPressureX, controls{i}.centerOfPressureY);
        axis equal;
        hold on;

        
      end
      
      
    end
    
  end
  
end

