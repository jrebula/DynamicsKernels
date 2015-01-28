classdef TrajectoryOptimizationState
  % TrajectoryOptimizationState 
  
  properties
    states;
    inputs;
    parameters;
    reactionForces;
    
    numberOfStates;
    numberOfInputs;
    numberOfParameters;
    numberOfReactionForces;
    
    numberOfControlPoints;
  end
  
  methods
    
    function [this] = TrajectoryOptimizationState(numberOfStates, ...
        numberOfInputs, ...
        numberOfParameters, ...
        numberOfReactionForces, ...
        numberOfControlPoints)
      %%
      this.numberOfStates = numberOfStates;
      this.numberOfInputs = numberOfInputs;
      this.numberOfParameters = numberOfParameters;
      this.numberOfReactionForces = numberOfReactionForces;
      
      this.numberOfControlPoints = numberOfControlPoints;
      
      this.states = zeros(numberOfStates, numberOfControlPoints);
      this.inputs = zeros(numberOfInputs, numberOfControlPoints);
      this.parameters = zeros(numberOfParameters, 1);
      this.reactionForces = zeros(numberOfReactionForces, numberOfControlPoints);
    end
    
    function [state, input, reactionForces] = getStateControlAndReactionsAtIndex(this, index)
      %%
      state = this.states(:, index);
      input = this.inputs(:, index);
      reactionForces = this.reactionForces(:, index);
    end
    
    function [this] = setFromVector(this, vector)
      %%
      
      indeces = 1 : (this.numberOfStates * this.numberOfControlPoints);
      this.states = vector(indeces);
      vector(indeces) = [];
      
      indeces = 1 : (this.numberOfInputs * this.numberOfControlPoints);
      this.inputs = vector(indeces);
      vector(indeces) = [];
      
      indeces = 1 : (this.numberOfReactionForces * this.numberOfControlPoints);
      this.reactionForces = vector(indeces);
      vector(indeces) = [];
      
      this.parameters = vector;
      
    end
    
    function [vector] = getVector(this)
      %%
      vector = reshape(this.states, [this.numberOfStates * this.numberOfControlPoints, 1]);
      vector = [vector reshape(this.inputs, [this.numberOfInputs * this.numberOfControlPoints, 1])];
      vector = [vector reshape(this.reactionForces, [this.numberOfReactionForces * this.numberOfControlPoints, 1])];
      vector = [vector this.parameters];
    end
    
    
    
  end
  
end

