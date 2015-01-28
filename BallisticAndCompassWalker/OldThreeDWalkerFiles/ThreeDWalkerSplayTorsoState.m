classdef ThreeDWalkerSplayTorsoState < ThreeDWalkerSplayState
  % ThreeDWalkerSplayTorsoState wrapper around the state of a ThreeDWalkerSplayTorso
  
  properties
    
    torso = struct('roll', 0, 'rollDot', 0);
    %     torso = struct('roll', 0, 'rollDot', 0, 'pitch', 0, 'pitchDot', 0);
    
  end
  
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
      assignin(ws, 'q9', x(i)); i = i + 1;
      %       assignin(ws, 'q10', x(i)); i = i + 1;
      
      assignin(ws, 'u1', x(i)); i = i + 1;
      assignin(ws, 'u2', x(i)); i = i + 1;
      assignin(ws, 'u3', x(i)); i = i + 1;
      assignin(ws, 'u4', x(i)); i = i + 1;
      assignin(ws, 'u5', x(i)); i = i + 1;
      assignin(ws, 'u6', x(i)); i = i + 1;
      assignin(ws, 'u7', x(i)); i = i + 1;
      assignin(ws, 'u8', x(i)); i = i + 1;
      assignin(ws, 'u9', x(i)); i = i + 1;
      %       assignin(ws, 'u10', x(i));
      
      
      %       assignin(ws, 'c4p6', cos(x(4) + x(6)));
      %       assignin(ws, 's4p6', sin(x(4) + x(6)));
      %
      %       assignin(ws, 'c4p8', cos(x(4) + x(8)));
      %       assignin(ws, 's4p8', sin(x(4) + x(8)));
      %       assignin(ws, 'c4m5p6', cos(x(4) - x(5) + x(6)));
      %
      assignin(ws, 'c4p9', cos(x(4) + x(9)));
      assignin(ws, 's4p9', sin(x(4) + x(9)));
      
      
      assignin(ws, 'c5m7', cos(x(5) - x(7)));
      assignin(ws, 's5m7', sin(x(5) - x(7)));


      
      assignin(ws, 'c4', cos(x(4)));
      assignin(ws, 'c5', cos(x(5)));
      assignin(ws, 'c6', cos(x(6)));
      assignin(ws, 'c7', cos(x(7)));
      assignin(ws, 'c8', cos(x(8)));
      assignin(ws, 'c9', cos(x(9)));
      %       assignin(ws, 'c10', cos(x(10)));
      
      assignin(ws, 's4', sin(x(4)));
      assignin(ws, 's5', sin(x(5)));
      assignin(ws, 's6', sin(x(6)));
      assignin(ws, 's7', sin(x(7)));
      assignin(ws, 's8', sin(x(8)));
      assignin(ws, 's9', sin(x(9)));
      %       assignin(ws, 's10', sin(x(10)));
      
    end
    
  end
  
  
  methods
    
    function [this] = ThreeDWalkerSplayTorsoState(input)
      %%
      this = this@ThreeDWalkerSplayState();
      
      if (nargin == 0)
        this.torso.roll = 0;
        this.torso.rollDot = 0;
        %         this.torso.pitch = 0;
        %         this.torso.pitchDot = 0;
        return;
      end
      
      if (isa(input, 'ThreeDWalkerSplayTorsoState'))
        this.torso = input.torso;
        this.pelvis = input.pelvis;
        this.stanceLeg = input.stanceLeg;
        this.swingLeg = input.swingLeg;
        return;
      end
      
      
      if (isa(input, 'ThreeDWalkerSplayState'))
        this.pelvis = input.pelvis;
        this.stanceLeg = input.stanceLeg;
        this.swingLeg = input.swingLeg;
        return;
      end
      
      if (isa(input, 'ThreeDWalkerState'))
        error('maybe try turning the input state into a ThreeDWalkerSplayState first?');
      end
      
      if (isnumeric(input))
        %                 if (length(input) == 12)
        %                   this = setFromVector@ThreeDWalkerState(this, input);
        %                 else
        this = ThreeDWalkerSplayTorsoState();
        this = this.setFromVector(input);
        %         end
        return
      end
      
      this = input;
    end
    
    function [new] = switchLegsSignsNotNames(this, walker, initialPelvisY)
      %% go from walking on old stance foot to walking on old swing foot
      new = ThreeDWalkerSplayTorsoState(switchLegsSignsNotNames@ThreeDWalkerSplayState(this, walker, initialPelvisY));
      new.torso.roll = -this.torso.roll;
      new.torso.rollDot = -this.torso.rollDot;
      
      %       new.torso.pitch = this.torso.pitch;
      %       new.torso.pitchDot = this.torso.pitchDot;
    end
    
    function [new] = switchLegsNamesNotSigns(this, walker)
      %% go from walking on old stance foot to walking on old swing foot
      %       new = switchLegs@ThreeDWalkerState(this, walker);
      new = ThreeDWalkerSplayTorsoState(switchLegsNamesNotSigns@ThreeDWalkerSplayState(this, walker));
      new.torso.roll = -this.torso.roll;
      new.torso.rollDot = -this.torso.rollDot;
      
      %       new.torso.pitch = this.torso.pitch;
      %       new.torso.pitchDot = this.torso.pitchDot;
      error('pretty sure this doesn''t do what it says, also pretty sure we don''t need it');
    end
    
    function [new] = switchLegs(this, walker)
      %% go from walking on old stance foot to walking on old swing foot
      new = ThreeDWalkerSplayTorsoState(switchLegs@ThreeDWalkerSplayState(this, walker));
      new.torso.roll = -this.torso.roll;
      new.torso.rollDot = -this.torso.rollDot;
      
      %       new.torso.pitch = this.torso.pitch;
      %       new.torso.pitchDot = this.torso.pitchDot;
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
        ...this.torso.pitch, ...
        this.torso.roll, ...
        ...
        this.pelvis.xDot, ...
        this.pelvis.yDot, ...
        this.pelvis.zDot, ...
        this.pelvis.rollDot, ...
        this.stanceLeg.pitchDot, ...
        this.stanceLeg.rollDot, ...
        this.swingLeg.pitchDot, ...
        this.swingLeg.rollDot, ...
        ...this.torso.pitchDot ...
        this.torso.rollDot ...
        ]';
    end
    
    function [this] = setFromVector(this, vect)
      %%
      if (length(vect) == 16)
        this = setFromVector@ThreeDWalkerSplayState(this, vect);
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
      % this.torso.pitch = vect(i); i = i + 1;
      this.torso.roll = vect(i); i = i + 1;
      
      this.pelvis.xDot = vect(i); i = i + 1;
      this.pelvis.yDot = vect(i); i = i + 1;
      this.pelvis.zDot = vect(i); i = i + 1;
      this.pelvis.rollDot = vect(i); i = i + 1;
      this.stanceLeg.pitchDot = vect(i); i = i + 1;
      this.stanceLeg.rollDot = vect(i); i = i + 1;
      this.swingLeg.pitchDot = vect(i); i = i + 1;
      this.swingLeg.rollDot = vect(i); i = i + 1;
      % this.torso.pitchDot = vect(i); i = i + 1;
      this.torso.rollDot = vect(i); i = i + 1;
      
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
      assignin(ws, 'q9', x(i)); i = i + 1;
      %       assignin(ws, 'q10', x(i)); i = i + 1;
      
      assignin(ws, 'u1', x(i)); i = i + 1;
      assignin(ws, 'u2', x(i)); i = i + 1;
      assignin(ws, 'u3', x(i)); i = i + 1;
      assignin(ws, 'u4', x(i)); i = i + 1;
      assignin(ws, 'u5', x(i)); i = i + 1;
      assignin(ws, 'u6', x(i)); i = i + 1;
      assignin(ws, 'u7', x(i)); i = i + 1;
      assignin(ws, 'u8', x(i)); i = i + 1;
      assignin(ws, 'u9', x(i)); i = i + 1;
      %       assignin(ws, 'u10', x(i));
      
      
      %       assignin(ws, 'c4p6', cos(x(4) + x(6)));
      %       assignin(ws, 's4p6', sin(x(4) + x(6)));
      %
      %       assignin(ws, 'c4p8', cos(x(4) + x(8)));
      %       assignin(ws, 's4p8', sin(x(4) + x(8)));
      %       assignin(ws, 'c4m5p6', cos(x(4) - x(5) + x(6)));
      %
      assignin(ws, 'c4p9', cos(x(4) + x(9)));
      assignin(ws, 's4p9', sin(x(4) + x(9)));
      
      assignin(ws, 'c4', cos(x(4)));
      assignin(ws, 'c5', cos(x(5)));
      assignin(ws, 'c6', cos(x(6)));
      assignin(ws, 'c7', cos(x(7)));
      assignin(ws, 'c8', cos(x(8)));
      assignin(ws, 'c9', cos(x(9)));
      %       assignin(ws, 'c10', cos(x(10)));
      
      assignin(ws, 's4', sin(x(4)));
      assignin(ws, 's5', sin(x(5)));
      assignin(ws, 's6', sin(x(6)));
      assignin(ws, 's7', sin(x(7)));
      assignin(ws, 's8', sin(x(8)));
      assignin(ws, 's9', sin(x(9)));
%       assignin(ws, 's10', sin(x(10)));
    end
    
  end
  
end

