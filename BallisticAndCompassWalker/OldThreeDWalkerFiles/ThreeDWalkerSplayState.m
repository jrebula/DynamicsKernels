classdef ThreeDWalkerSplayState < ThreeDWalkerState
  % ThreeDWalkerSplayState wrapper around the state of a ThreeDWalkerSplay
  
  methods (Static)
    
    
    function [] = setQsUsAndTrigInCurrentFunctionFromState(state)
      x = state;
      ws = 'caller';
      i = 1;
      
      assignin(ws, 'q1', x(i)); i = i + 1;
      assignin(ws, 'q2', x(i)); i = i + 1;
      assignin(ws, 'q3', x(i)); i = i + 1;
      assignin(ws, 'q4', x(i)); i = i + 1;
      assignin(ws, 'q5', x(i)); i = i + 1;
      assignin(ws, 'q6', x(i)); i = i + 1;
      assignin(ws, 'q7', x(i)); i = i + 1;
      assignin(ws, 'q8', x(i)); i = i + 1;

      assignin(ws, 'u1', x(i)); i = i + 1;
      assignin(ws, 'u2', x(i)); i = i + 1;
      assignin(ws, 'u3', x(i)); i = i + 1;
      assignin(ws, 'u4', x(i)); i = i + 1;
      assignin(ws, 'u5', x(i)); i = i + 1;
      assignin(ws, 'u6', x(i)); i = i + 1;
      assignin(ws, 'u7', x(i)); i = i + 1;
      assignin(ws, 'u8', x(i));
      
      %       assignin(ws, 'c4m6', cos(x(4) - x(6)));
      %       assignin(ws, 'c4m8', cos(x(4) - x(8)));
      %       assignin(ws, 'c5m6', cos(x(5) - x(6)));
      %       assignin(ws, 'c5p6', cos(x(5) + x(6)));
      %       assignin(ws, 'c7m8', cos(x(7) - x(8)));
      %       assignin(ws, 'c7p8', cos(x(7) + x(8)));
      %
      %       assignin(ws, 's4m6', sin(x(4) - x(6)));
      %       assignin(ws, 's4m8', sin(x(4) - x(8)));
      %
      %       assignin(ws, 's5m6',  sin(x(5) - x(6)));
      %       assignin(ws, 's5p6', sin(x(5) + x(6)));
      %       assignin(ws, 's7m8', sin(x(7) - x(8)));
      %       assignin(ws, 's7p8', sin(x(7) + x(8)));
      
      %       assignin(ws, 'c4m6p5', cos(x(4) + x(5) - x(6)));
      %       assignin(ws, 'c4m8p7',  cos(x(4) + x(7) - x(8)));
      
      assignin(ws, 'c4p6', cos(x(4) + x(6)));
      assignin(ws, 's4p6', sin(x(4) + x(6)));
      
      assignin(ws, 'c4p8', cos(x(4) + x(8)));
      assignin(ws, 's4p8', sin(x(4) + x(8)));
      assignin(ws, 'c4m5p6', cos(x(4) - x(5) + x(6)));
      
      assignin(ws, 'c4', cos(x(4)));
      assignin(ws, 'c5', cos(x(5)));
      assignin(ws, 'c6', cos(x(6)));
      assignin(ws, 'c7', cos(x(7)));
      assignin(ws, 'c8', cos(x(8)));
      
      assignin(ws, 's4', sin(x(4)));
      assignin(ws, 's5', sin(x(5)));
      assignin(ws, 's6', sin(x(6)));
      assignin(ws, 's7', sin(x(7)));
      assignin(ws, 's8', sin(x(8)));
      
    end
    
  end
  
  
  methods
    
    function [this] = ThreeDWalkerSplayState(input)
      %%
      this = this@ThreeDWalkerState();

      if (nargin == 0)
        this.stanceLeg.roll = 0;
        this.swingLeg.roll = 0;
        
        this.stanceLeg.rollDot = 0;
        this.swingLeg.rollDot = 0;
        return;
      end
            
      if (isa(input, 'ThreeDWalkerSplayState'))
        this.pelvis = input.pelvis;
        this.stanceLeg = input.stanceLeg;
        this.swingLeg = input.swingLeg;
        return;
      end
      
      if (isa(input, 'ThreeDWalkerState'))
        %         this = ThreeDWalkerState();
        
        this.pelvis = input.pelvis;
        this.stanceLeg = input.stanceLeg;
        this.swingLeg = input.swingLeg;
        
        this.stanceLeg.roll = 0;
        this.swingLeg.roll = 0;
        
        this.stanceLeg.rollDot = 0;
        this.swingLeg.rollDot = 0;
        return;
      end
      
      if (isnumeric(input))
        %                 if (length(input) == 12)
        %                   this = setFromVector@ThreeDWalkerState(this, input);
        %                 else
        this = ThreeDWalkerSplayState();
        this = this.setFromVector(input);
        %         end
        return
      end
      
      this = input;
    end
    
    
    function [new] = switchLegsSignsNotNames(this, walker, initialPelvisY)
      %% go from walking on old stance foot to walking on old swing foot
      
      %       new = switchLegs@ThreeDWalkerState(this, walker);
      new = ThreeDWalkerSplayState(this);
      %       %       change the names and signs of the angles
      new.pelvis.x = this.pelvis.x;
      new.pelvis.y = initialPelvisY - this.pelvis.y;
      new.pelvis.z = this.pelvis.z;
      new.pelvis.roll = -this.pelvis.roll;
      new.stanceLeg.pitch = this.stanceLeg.pitch;
      new.swingLeg.pitch = this.swingLeg.pitch;
      
      new.pelvis.xDot = this.pelvis.xDot;
      new.pelvis.yDot = -this.pelvis.yDot;
      new.pelvis.zDot = this.pelvis.zDot;
      new.pelvis.rollDot = -this.pelvis.rollDot;
      new.stanceLeg.pitchDot = this.stanceLeg.pitchDot;
      new.swingLeg.pitchDot = this.swingLeg.pitchDot;
      
      new.stanceLeg.roll = -this.stanceLeg.roll;
      new.swingLeg.roll = -this.swingLeg.roll;
      
      new.stanceLeg.rollDot = -this.stanceLeg.rollDot;
      new.swingLeg.rollDot = -this.swingLeg.rollDot;
    end
    
    function [new] = switchLegsNamesNotSigns(this, walker)
      %% go from walking on old stance foot to walking on old swing foot
      
      %       new = switchLegs@ThreeDWalkerState(this, walker);
      new = ThreeDWalkerSplayState(this);
      %       %       change the names and signs of the angles
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
      
      new.stanceLeg.roll = -this.swingLeg.roll;
      new.swingLeg.roll = -this.stanceLeg.roll;
      
      new.stanceLeg.rollDot = -this.swingLeg.rollDot;
      new.swingLeg.rollDot = -this.stanceLeg.rollDot;
      
      
      % just change the names of the angles
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
      %
      %       new.stanceLeg.roll = this.swingLeg.roll;
      %       new.swingLeg.roll = this.stanceLeg.roll;
      %
      %       new.stanceLeg.rollDot = this.swingLeg.rollDot;
      %       new.swingLeg.rollDot = this.stanceLeg.rollDot;
    end
    
    function [new] = switchLegs(this, walker)
      %% go from walking on old stance foot to walking on old swing foot
      
      %       new = switchLegs@ThreeDWalkerState(this, walker);
      new = ThreeDWalkerSplayState(this);
      
      
      %       change the names and signs of the angles
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
      
      new.stanceLeg.roll = -this.swingLeg.roll;
      new.swingLeg.roll = -this.stanceLeg.roll;
      
      new.stanceLeg.rollDot = -this.swingLeg.rollDot;
      new.swingLeg.rollDot = -this.stanceLeg.rollDot;
      
      
      %       % just change the names of the angles
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
      %
      %       new.stanceLeg.roll = this.swingLeg.roll;
      %       new.swingLeg.roll = this.stanceLeg.roll;
      %
      %       new.stanceLeg.rollDot = this.swingLeg.rollDot;
      %       new.swingLeg.rollDot = this.stanceLeg.rollDot;
    end
    
    function [vect] = getVector(this)
      %%
      vect = [ ...
        this.pelvis.x, ...
        this.pelvis.y, ...
        this.pelvis.z, ...
        this.pelvis.roll, ...
        this.stanceLeg.pitch, ...
        this.stanceLeg.roll, ...
        this.swingLeg.pitch, ...
        this.swingLeg.roll, ...
        ...
        this.pelvis.xDot, ...
        this.pelvis.yDot, ...
        this.pelvis.zDot, ...
        this.pelvis.rollDot, ...
        this.stanceLeg.pitchDot, ...
        this.stanceLeg.rollDot, ...
        this.swingLeg.pitchDot, ...
        this.swingLeg.rollDot]';
    end
    
    function [this] = setFromVector(this, vect)
      %%
      if (length(vect) == 12)
        this = setFromVector@ThreeDWalkerState(this, vect);
        return;
      end
      
      i = 1;
      this.pelvis.x = vect(i); i = i + 1;
      this.pelvis.y = vect(i); i = i + 1;
      this.pelvis.z = vect(i); i = i + 1;
      this.pelvis.roll = vect(i); i = i + 1;
      this.stanceLeg.pitch = vect(i); i = i + 1;
      this.stanceLeg.roll = vect(i); i = i + 1;
      this.swingLeg.pitch = vect(i); i = i + 1;
      this.swingLeg.roll = vect(i); i = i + 1;
      
      this.pelvis.xDot = vect(i); i = i + 1;
      this.pelvis.yDot = vect(i); i = i + 1;
      this.pelvis.zDot = vect(i); i = i + 1;
      this.pelvis.rollDot = vect(i); i = i + 1;
      this.stanceLeg.pitchDot = vect(i); i = i + 1;
      this.stanceLeg.rollDot = vect(i); i = i + 1;
      this.swingLeg.pitchDot = vect(i); i = i + 1;
      this.swingLeg.rollDot = vect(i);
    end
    
    function [] = setQsUsAndTrigInCurrentFunction(this)
      %%
      x = this.getVector();
      ws = 'caller';
      i = 1;
      
      assignin(ws, 'q1', x(i)); i = i + 1;
      assignin(ws, 'q2', x(i)); i = i + 1;
      assignin(ws, 'q3', x(i)); i = i + 1;
      assignin(ws, 'q4', x(i)); i = i + 1;
      assignin(ws, 'q5', x(i)); i = i + 1;
      assignin(ws, 'q6', x(i)); i = i + 1;
      assignin(ws, 'q7', x(i)); i = i + 1;
      assignin(ws, 'q8', x(i)); i = i + 1;

      assignin(ws, 'u1', x(i)); i = i + 1;
      assignin(ws, 'u2', x(i)); i = i + 1;
      assignin(ws, 'u3', x(i)); i = i + 1;
      assignin(ws, 'u4', x(i)); i = i + 1;
      assignin(ws, 'u5', x(i)); i = i + 1;
      assignin(ws, 'u6', x(i)); i = i + 1;
      assignin(ws, 'u7', x(i)); i = i + 1;
      assignin(ws, 'u8', x(i));
      
      %       assignin(ws, 'c4m6', cos(x(4) - x(6)));
      %       assignin(ws, 'c4m8', cos(x(4) - x(8)));
      %       assignin(ws, 'c5m6', cos(x(5) - x(6)));
      %       assignin(ws, 'c5p6', cos(x(5) + x(6)));
      %       assignin(ws, 'c7m8', cos(x(7) - x(8)));
      %       assignin(ws, 'c7p8', cos(x(7) + x(8)));
      %
      %       assignin(ws, 's4m6', sin(x(4) - x(6)));
      %       assignin(ws, 's4m8', sin(x(4) - x(8)));
      %
      %       assignin(ws, 's5m6',  sin(x(5) - x(6)));
      %       assignin(ws, 's5p6', sin(x(5) + x(6)));
      %       assignin(ws, 's7m8', sin(x(7) - x(8)));
      %       assignin(ws, 's7p8', sin(x(7) + x(8)));
      
      %       assignin(ws, 'c4m6p5', cos(x(4) + x(5) - x(6)));
      %       assignin(ws, 'c4m8p7',  cos(x(4) + x(7) - x(8)));
      
      assignin(ws, 'c4p6', cos(x(4) + x(6)));
      assignin(ws, 's4p6', sin(x(4) + x(6)));
      
      assignin(ws, 'c4p8', cos(x(4) + x(8)));
      assignin(ws, 's4p8', sin(x(4) + x(8)));
      assignin(ws, 'c4m5p6', cos(x(4) - x(5) + x(6)));
      
      assignin(ws, 'c4', cos(x(4)));
      assignin(ws, 'c5', cos(x(5)));
      assignin(ws, 'c6', cos(x(6)));
      assignin(ws, 'c7', cos(x(7)));
      assignin(ws, 'c8', cos(x(8)));
      
      assignin(ws, 's4', sin(x(4)));
      assignin(ws, 's5', sin(x(5)));
      assignin(ws, 's6', sin(x(6)));
      assignin(ws, 's7', sin(x(7)));
      assignin(ws, 's8', sin(x(8)));
    end
    
  end
  
end

