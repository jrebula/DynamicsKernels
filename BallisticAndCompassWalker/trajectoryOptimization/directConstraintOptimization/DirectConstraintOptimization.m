classdef DirectConstraintOptimization
  
  properties
    timeStep;
    mechanicalSystem;
    costFunction;
  end
  
  methods
    
    function [this] = DirectConstraintOptimization(mechanicalSystem, ...
        costFunction, timeStep)
      %%
      this.timeStep = timeStep;
      this.mechanicalSystem = mechanicalSystem;
      this.costFunction = costFunction;
    end
    
    function [constraintVectorEquality, constraintVectorGreaterThanZero] = ...
        calculateOptimizationConstraints(this, ...
        currentState, nextState, currentConstraintForces, nextConstraintForces, nextTime)
      %%
      
      [qIndeces, uIndeces] = this.mechanicalSystem.getQAndUIndeces();
      [MM, rhs] = this.mechanicalSystem.getMassMatrixAndRightHandSide(nextTime, nextState);
      [constraintJacobian] = this.mechanicalSystem.getConstraintJacobian(nextState);
      [nonPenetrationConstraint] = this.mechanicalSystem.getNonPenetrationConstraint(currentState);
      [additionalConstraintConstraints] = ...
        this.mechanicalSystem.getAdditionalConstrainedPositive(currentState, currentConstraintForces);
      
      qK = currentState(qIndeces);
      qKPlus1 = nextState(qIndeces);
      qDotK = currentState(uIndeces);
      qDotKPlus1 = nextState(uIndeces);
      
      % see Posa, 2013ish, "A Direct Method for Trajectory Optimization of Rigid Bodies Through Contact"

      % eq 7
      constraintVectorEquality = [ ...
        qK - qKPlus1 + this.h * qDotKPlus1 ...
        MM * (qDotKPlus1 - qDotK) - this.h * (rhs + constraintJacobian' * nextConstraintForces)];

      % eq 17
      constraintVectorGreaterThanZero = [constraintVectorGreaterThanZero ...
        nonPenetrationConstraint ...
        nonPenetrationConstraint' * currentConstraintForces];
      
    end
    
  end
  
end

