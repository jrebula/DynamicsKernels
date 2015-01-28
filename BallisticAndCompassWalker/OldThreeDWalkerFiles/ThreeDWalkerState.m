classdef ThreeDWalkerState
  % ThreeDWalkerState wrapper around the state of a 3d walker
  
  properties
    pelvis = struct('x', 0, 'y', 0, 'z', 1, 'roll', 0, 'xDot', 0, 'yDot', 0, 'zDot', 0, 'rollDot', 0);
    stanceLeg = struct('pitch', 0, 'pitchDot', 0);
    swingLeg = struct('pitch', 0, 'pitchDot', 0);
  end
  
  methods
    
    function [this] = ThreeDWalkerState(input)
      %%
      if (nargin == 0)
        
        %         this.pelvis
        
        %         this.pelvis.x = 0;
        %         this.pelvis.y = 0;
        %         this.pelvis.z = 1;
        %         this.pelvis.roll = 0;
        %         this.stanceLeg.pitch = 0;
        %         this.swingLeg.pitch = 0;
        
        %         this.pelvis.xDot = 0;
        %         this.pelvis.yDot = 0;
        %         this.pelvis.zDot = 0;
        %         this.pelvis.rollDot = 0;
        %         this.stanceLeg.pitchDot = 0;
        %         this.swingLeg.pitchDot = 0;
        return;
      end
      
      if (isnumeric(input))
        this = ThreeDWalkerState();
        this = this.setFromVector(input);
        return
      end
      
      this = input;
    end
    
    
    function [new] = switchLegs(this, walker)
      %% go from walking on old stance foot to walking on old swing foot
      
      new = ThreeDWalkerState();
      
      % this kinda made it look like the walker was switching from left to
      % right:
      %       new.pelvis.x = this.pelvis.x;
      %       new.pelvis.y = this.pelvis.y;
      %       new.pelvis.z = this.pelvis.z;
      %       new.pelvis.roll = pi + this.pelvis.roll;
      %       new.stanceLeg.pitch = this.swingLeg.pitch;
      %       new.swingLeg.pitch = this.stanceLeg.pitch;
      %
      %       new.pelvis.xDot = this.pelvis.xDot;
      %       new.pelvis.yDot = this.pelvis.yDot;
      %       new.pelvis.zDot = this.pelvis.zDot;
      %       new.pelvis.rollDot = this.pelvis.rollDot;
      %       new.stanceLeg.pitchDot = this.swingLeg.pitchDot;
      %       new.swingLeg.pitchDot = this.stanceLeg.pitchDot;
      
      % this switches the names and sides of the walker:
      points = walker.getKinematicPoints(this);
      
      new.pelvis.x = this.pelvis.x;
      new.pelvis.y = this.pelvis.y;
      %       new.pelvis.y = points.swingFootContactPoint(2) - this.pelvis.y;
      new.pelvis.z = this.pelvis.z;
      new.pelvis.roll = -this.pelvis.roll;
      new.stanceLeg.pitch = this.swingLeg.pitch;
      new.swingLeg.pitch = this.stanceLeg.pitch;
      
      new.pelvis.xDot = this.pelvis.xDot;
      new.pelvis.yDot = -this.pelvis.yDot;
      new.pelvis.zDot = this.pelvis.zDot;
      new.pelvis.rollDot = -this.pelvis.rollDot;
      new.stanceLeg.pitchDot = this.swingLeg.pitchDot;
      new.swingLeg.pitchDot = this.stanceLeg.pitchDot;
      
      %       % this just switches the names of the states:
      %       new.pelvis.x = this.pelvis.x;
      %       new.pelvis.y = this.pelvis.y;
      %       new.pelvis.z = this.pelvis.z;
      %       new.pelvis.roll = this.pelvis.roll;
      %       new.stanceLeg.pitch = this.swingLeg.pitch;
      %       new.swingLeg.pitch = this.stanceLeg.pitch;
      %
      %       new.pelvis.xDot = this.pelvis.xDot;
      %       new.pelvis.yDot = this.pelvis.yDot;
      %       new.pelvis.zDot = this.pelvis.zDot;
      %       new.pelvis.rollDot = this.pelvis.rollDot;
      %       new.stanceLeg.pitchDot = this.swingLeg.pitchDot;
      %       new.swingLeg.pitchDot = this.stanceLeg.pitchDot;
      
      
    end
    
    function [vect] = getVector(this)
      %%
      vect = [ ...
        this.pelvis.x, ...
        this.pelvis.y, ...
        this.pelvis.z, ...
        this.pelvis.roll, ...
        this.stanceLeg.pitch, ...
        this.swingLeg.pitch, ...
        this.pelvis.xDot, ...
        this.pelvis.yDot, ...
        this.pelvis.zDot, ...
        this.pelvis.rollDot, ...
        this.stanceLeg.pitchDot, ...
        this.swingLeg.pitchDot]';
    end
    
    function [this] = setFromVector(this, vect)
      %%
      i = 1;
      this.pelvis.x = vect(i); i = i + 1;
      this.pelvis.y = vect(i); i = i + 1;
      this.pelvis.z = vect(i); i = i + 1;
      this.pelvis.roll = vect(i); i = i + 1;
      this.stanceLeg.pitch = vect(i); i = i + 1;
      this.swingLeg.pitch = vect(i); i = i + 1;
      this.pelvis.xDot = vect(i); i = i + 1;
      this.pelvis.yDot = vect(i); i = i + 1;
      this.pelvis.zDot = vect(i); i = i + 1;
      this.pelvis.rollDot = vect(i); i = i + 1;
      this.stanceLeg.pitchDot = vect(i); i = i + 1;
      this.swingLeg.pitchDot = vect(i);
    end
    
    
    function [] = setQsUsAndTrigInCurrentFunction(this)
      
      x = this.getVector();
      
      ws = 'caller';
      assignin(ws, 'q1', x(1));
      assignin(ws, 'q2', x(2));
      assignin(ws, 'q3', x(3));
      assignin(ws, 'q4', x(4));
      assignin(ws, 'q5', x(5));
      assignin(ws, 'q6', x(6));
      assignin(ws, 'u1', x(7));
      assignin(ws, 'u2', x(8));
      assignin(ws, 'u3', x(9));
      assignin(ws, 'u4', x(10));
      assignin(ws, 'u5', x(11));
      assignin(ws, 'u6', x(12));
      
      assignin(ws, 'c4', cos(x(4)));
      assignin(ws, 'c5', cos(x(5)));
      assignin(ws, 'c6', cos(x(6)));
      
      assignin(ws, 's4', sin(x(4)));
      assignin(ws, 's5', sin(x(5)));
      assignin(ws, 's6', sin(x(6)));
    end
    
  end
  
end

