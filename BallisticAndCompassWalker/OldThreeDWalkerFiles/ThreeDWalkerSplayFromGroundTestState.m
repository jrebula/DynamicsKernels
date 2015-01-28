classdef ThreeDWalkerSplayFromGroundTestState < ThreeDWalkerState
  % ThreeDWalkerSplayFromGroundTestState wrapper around the state of a ThreeDWalkerSplayFromGroundTest
  
  properties
    
    stanceAnkle = [];
    
  end
  
  methods
    
    function [this] = ThreeDWalkerSplayFromGroundTestState(input)
      %%
      this = this@ThreeDWalkerState();

      if (nargin == 0)
        
        this.stanceLeg.roll = 0;
        %         this.swingLeg.roll = 0;
        
        this.stanceLeg.rollDot = 0;
        %         this.swingLeg.rollDot = 0;
        
        this.stanceAnkle.x = 0;
        this.stanceAnkle.y = 0;
        this.stanceAnkle.z = 0;
        
        this.stanceAnkle.xDot = 0;
        this.stanceAnkle.yDot = 0;
        this.stanceAnkle.zDot = 0;
        
        this.stanceAnkle.pitch = 0;
        this.stanceAnkle.roll = 0;
        
        this.stanceAnkle.pitchDot = 0;
        this.stanceAnkle.rollDot = 0;
        
        return;
      end
            
      if (isa(input, 'ThreeDWalkerSplayFromGroundState'))
        this.pelvis = input.pelvis;
        this.stanceAnkle = input.stanceAnkle;
        this.stanceLeg = input.stanceLeg;
        %         this.swingLeg = input.swingLeg;
        return;
      end
      
      if (isa(input, 'ThreeDWalkerState'))
        %         this = ThreeDWalkerState();
        
        this.pelvis = input.pelvis;
        this.stanceLeg = input.stanceLeg;
        %         this.swingLeg = input.swingLeg;
        
        this.stanceLeg.roll = 0;
        %         this.swingLeg.roll = 0;
        
        this.stanceLeg.rollDot = 0;
        %         this.swingLeg.rollDot = 0;
        
        this.stanceAnkle.x = 0;
        this.stanceAnkle.y = 0;
        this.stanceAnkle.z = 1;
        
        this.stanceAnkle.xDot = 0;
        this.stanceAnkle.yDot = 0;
        this.stanceAnkle.zDot = 0;
        
        this.stanceAnkle.pitch = 0;
        this.stanceAnkle.roll = 0;
        
        this.stanceAnkle.pitchDot = 0;
        this.stanceAnkle.rollDot = 0;
        
        return;
      end
      
      if (isnumeric(input))
        %                 if (length(input) == 12)
        %                   this = setFromVector@ThreeDWalkerState(this, input);
        %                 else
        this = ThreeDWalkerSplayFromGroundTestState();
        this = this.setFromVector(input);
        %         end
        return
      end
      
      this = input;
    end
    
    function [new] = switchLegs(this)
      %% go from walking on old stance foot to walking on old swing foot
      
      %       new = switchLegs@ThreeDWalkerState(this);
      %       new = ThreeDWalkerSplayState(new);
      %
      %       new.stanceLeg.roll = -this.swingLeg.roll;
      %       new.swingLeg.roll = -this.stanceLeg.roll;
      %       new.stanceLeg.rollDot = -this.swingLeg.rollDot;
      %       new.swingLeg.rollDot = -this.stanceLeg.rollDot;
      
      new = ThreeDWalkerSplayFromGroundTestState(this);
      return;
      
      this.setQsUsAndTrigInCurrentFunction();
      
      rollAngleOfSwingAnkle = acos(-(s4*(s4*(-(s5*(-(c7*c8*s6) - c6*s8)) + c4*(1 ...
        - c5)*(c6*c7*c8 - s6*s8)) + c8*s7*(c5 + (1 - c5)*(c4*c4)))) + c4*(c4*((1 - ...
        c5)*c8*s4*s7 + s5*(-(c7*c8*s6) - c6*s8)) + (c6*c7*c8 - s6*s8)*(c5 + (1 - ...
        c5)*(s4*s4))));
      
      pitchAngleOfSwingAnkle = acos(-(s4*(s4*(-(s5*(-(c7*c8*s6) - c6*s8)) + c4*(1 ...
        - c5)*(c6*c7*c8 - s6*s8)) + c8*s7*(c5 + (1 - c5)*(c4*c4)))) + c4*(c4*((1 - ...
        c5)*c8*s4*s7 + s5*(-(c7*c8*s6) - c6*s8)) + (c6*c7*c8 - s6*s8)*(c5 + (1 - ...
        c5)*(s4*s4))));
      
      %%
      new.stanceAnkle.x = this.stanceAnkle.x;
      new.stanceAnkle.y = this.stanceAnkle.y;
      new.stanceAnkle.z = this.stanceAnkle.z;

      new.stanceAnkle.roll = rollAngleOfSwingAnkle;
      new.stanceAnkle.pitch = pitchAngleOfSwingAnkle;
      
      %       new.stanceLeg.roll = -this.swingLeg.roll;
      
      %       new.swingLeg.pitch = this.stanceLeg.pitch;
      %       new.swingLeg.roll = -this.stanceLeg.roll;
      
      %%
      new.stanceAnkle.xDot = this.stanceAnkle.xDot;
      new.stanceAnkle.yDot = -this.stanceAnkle.yDot;
      new.stanceAnkle.zDot = this.stanceAnkle.zDot;
      
      new.stanceAnkle.rollDot = 0; %-this.swingAnkle.rollDot;
      new.stanceAnkle.pitchDot = 0; %this.swingAnkle.pitchDot;
      
      %       new.stanceLeg.rollDot = -this.swingLeg.rollDot;
      
      %       new.swingLeg.pitchDot = this.stanceLeg.pitchDot;
      %       new.swingLeg.rollDot = -this.stanceLeg.rollDot;
      
    end
    
    function [vect] = getVector(this)
      %%
      vect = [ ...
        this.stanceAnkle.x, ...
        this.stanceAnkle.y, ...
        this.stanceAnkle.z, ...
        this.stanceAnkle.roll, ...
        this.stanceAnkle.pitch, ...
        ...this.stanceLeg.roll, ...
        ...this.swingLeg.pitch, ...
        ...this.swingLeg.roll, ...
        ...
        this.stanceAnkle.xDot, ...
        this.stanceAnkle.yDot, ...
        this.stanceAnkle.zDot, ...
        this.stanceAnkle.rollDot, ...
        this.stanceAnkle.pitchDot, ...
        ...this.stanceLeg.rollDot, ...
        ...this.swingLeg.pitchDot, ...
        ...this.swingLeg.rollDot
        ]';
    end
    
    function [this] = setFromVector(this, vect)
      %%
      if (length(vect) == 12)
        this = setFromVector@ThreeDWalkerState(this, vect);
        return;
      end
      
      i = 1;
      this.stanceAnkle.x = vect(i); i = i + 1;
      this.stanceAnkle.y = vect(i); i = i + 1;
      this.stanceAnkle.z = vect(i); i = i + 1;
      this.stanceAnkle.roll = vect(i); i = i + 1;
      this.stanceAnkle.pitch = vect(i); i = i + 1;
      %       this.stanceLeg.roll = vect(i); i = i + 1;
      %       this.swingLeg.pitch = vect(i); i = i + 1;
      %       this.swingLeg.roll = vect(i); i = i + 1;
      
      this.stanceAnkle.xDot = vect(i); i = i + 1;
      this.stanceAnkle.yDot = vect(i); i = i + 1;
      this.stanceAnkle.zDot = vect(i); i = i + 1;
      this.stanceAnkle.rollDot = vect(i); i = i + 1;
      this.stanceAnkle.pitchDot = vect(i); i = i + 1;
      %       this.stanceLeg.rollDot = vect(i); i = i + 1;
      %       this.swingLeg.pitchDot = vect(i); i = i + 1;
      %       this.swingLeg.rollDot = vect(i);
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
      %       assignin(ws, 'q6', x(i)); i = i + 1;
      %       assignin(ws, 'q7', x(i)); i = i + 1;
      %       assignin(ws, 'q8', x(i)); i = i + 1;
      
      assignin(ws, 'u1', x(i)); i = i + 1;
      assignin(ws, 'u2', x(i)); i = i + 1;
      assignin(ws, 'u3', x(i)); i = i + 1;
      assignin(ws, 'u4', x(i)); i = i + 1;
      assignin(ws, 'u5', x(i)); i = i + 1;
      %       assignin(ws, 'u6', x(i)); i = i + 1;
      %       assignin(ws, 'u7', x(i)); i = i + 1;
      %       assignin(ws, 'u8', x(i));
      
      
      
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
      
      
      %       assignin(ws, 'csc7', csc(x(7)));
      %       assignin(ws, 'csc8', csc(x(8)));
      %
      %       assignin(ws, 's6m8p7', cos(x(6) + x(7) - x(8)));
      %       assignin(ws, 's4m8p5', cos(x(4) + x(5) - x(8)));
      %       assignin(ws, 'c6m8p7', cos(x(6) + x(7) - x(8)));
      %       assignin(ws, 'c4m8p5', cos(x(4) - x(8) + x(5)));
      %
      %       assignin(ws, 'c6p7', cos(x(6) + x(7)));
      %       assignin(ws, 's6p7', sin(x(6) + x(7)));
      
      assignin(ws, 'c4p5', cos(x(4) + x(5)));
      assignin(ws, 's4p5', sin(x(4) + x(5)));

%       assignin(ws, 'c4p6', cos(x(4) + x(6)));
%       assignin(ws, 's4p6', sin(x(4) + x(6)));

%       assignin(ws, 'c4p8', cos(x(4) + x(8)));
%       assignin(ws, 's4p8', sin(x(4) + x(8)));
%       assignin(ws, 'c4m5p6', cos(x(4) - x(5) + x(6)));
      
      assignin(ws, 'c4', cos(x(4)));
      assignin(ws, 'c5', cos(x(5)));
%       assignin(ws, 'c6', cos(x(6)));
%       assignin(ws, 'c7', cos(x(7)));
%       assignin(ws, 'c8', cos(x(8)));
%       assignin(ws, 'c9', cos(x(9)));
      
      assignin(ws, 's4', sin(x(4)));
      assignin(ws, 's5', sin(x(5)));
%       assignin(ws, 's6', sin(x(6)));
%       assignin(ws, 's7', sin(x(7)));
%       assignin(ws, 's8', sin(x(8)));
%       assignin(ws, 's9', sin(x(9)));
    end
    
  end
  
end

