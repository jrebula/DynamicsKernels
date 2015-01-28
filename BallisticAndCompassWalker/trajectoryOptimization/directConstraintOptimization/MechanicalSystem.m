classdef MechanicalSystem
  
  properties
    
  end
  
  methods (Abstract)
    
    [qs, us] = getQAndUIndeces(this)
    [MM, rhs] = getMassMatrixAndRightHandSide(this, time, state)
    [constraints] = getConstraints(this)
    
  end
  
  
  
  
end

