classdef MechanicalContact
  
  properties
  end
  
  methods (Abstract)
    
    [numContactForces] = getNumContactForces(this)
    [contactForceJacobians] = getContactForceJacobian(this, state)
    
    [nonPenetrationConstraint] = getNonPenetrationConstraint(this, state)
    [additionalConstraintConstraints] = getAdditionalConstraintConstraints(this, state, constraintForces)
    
  end
  
end

