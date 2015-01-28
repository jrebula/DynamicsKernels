classdef WalkerPlotter
  %WalkerPlotter functions for drawing walkers (segments, masses, etc.s)
  
  properties
    
    vectorColor = 'b';
    lineWidth = 2;
    footColor = [1 1 1] .* 0.7;
    linkColor = 'b';
    
  end
  
  methods
    
    
    function [] = plotVectorFromPoint(this, point, vector, varargin)
      
      scaleFactor = 1.0;
      for i = 1 : 2 : length(varargin)
        if(strcmp(varargin{i}, 'scaleFactor'))
          scaleFactor = varargin{i+1};
        end
      end
      
      line([point.position(1) point.position(1) + scaleFactor * vector.position(1)], ...
        [point.position(2), point.position(2) + scaleFactor * vector.position(2)], ...
        [point.position(3), point.position(3) + scaleFactor * vector.position(3)], ...
        'Color', this.vectorColor, 'LineWidth', this.lineWidth);
      hold on;
      
      dotColor = this.vectorColor; %'k';
      plot3(point.position(2), point.position(2), point.position(3), ...
        'o', 'MarkerFaceColor', dotColor, 'MarkerEdgeColor', dotColor, 'MarkerSize', 6);
      axis equal;
      
    end
    
    
    function [] = plotFoot(this, pose, radius)
      %%
      thetas = pi : 0.01 : 2*pi;
      footWedge = [ ...
        [0 0 0]' ...
        [radius * cos(thetas); zeros(size(thetas)); radius * sin(thetas)] ...
        [0 0 0]'];
      
      
      R2 = [ ...
        pose.R(1, 1), 0, pose.R(1, 3)
              0,      1,     0
        pose.R(3, 1), 0, pose.R(3, 3)
        ];

      footWedge = R2 * footWedge;
      
      footWedge = footWedge + repmat([pose.position(1) pose.position(2) pose.position(3)], [length(footWedge) 1])';
      
      fill3(footWedge(1,:), footWedge(2,:), footWedge(3,:), this.footColor)
      hold on;
      
      %       line( ...
      %         [pose.position(1) - radius - 0.1 pose.position(1) + radius + 0.1], ...
      %         [pose.position(2) pose.position(2)], ...
      %         [pose.position(3) - radius, pose.position(3) - radius] ...
      %         );
    end
    
    function plotSegmentBetweenPoints(this, from, to)
      
      line( ...
        [from(1) to(1)], ...
        [from(2), to(2)], ...
        [from(3), to(3)], ...
        'Color', this.linkColor, 'LineWidth', this.lineWidth);
      hold on;
      
      dotColor = this.linkColor; %'k';
      plot3(from(1), from(2), from(3), ...
        'o', 'MarkerFaceColor', dotColor, 'MarkerEdgeColor', dotColor, 'MarkerSize', 10);
      hold on;
      plot3(to(1), to(2), to(3), ...
        'o', 'MarkerFaceColor', dotColor, 'MarkerEdgeColor', dotColor, 'MarkerSize', 10);
      axis equal;
    end
    
    function drawCOMSymbol(this, pose, varargin)
      
      plot3(pose.position(1), pose.position(2), pose.position(3), 'o', ...
        'MarkerFaceColor', 'k', ...
        'MarkerFaceColor', 0.8 * ones(3, 1), ...
        'MarkerSize', 12);
      hold on;
      return
      
            radius = 0.02; %0.1; %
      for i=1:2:length(varargin)
        if(strcmp(varargin{i}, 'radius'))
          radius = varargin{i+1};
        end
      end
      
      rectangle('Position',[pose.position(1) - radius, pose.position(2) - radius, 2*radius, 2*radius], ...
        'Curvature',[1,1],...
        'FaceColor','w', 'LineWidth', this.lineWidth);
      hold on;
      
      thetas = 0:0.01:pi/2;
      topRightWedge = [[0 0]' radius*[cos(thetas); sin(thetas)] [0 0]'];
      
      thetas = pi:0.01:3*pi/2;
      bottomLeftWedge = [[0 0]' radius*[cos(thetas); sin(thetas)] [0 0]'];
      
      topRightWedge = pose.R*topRightWedge;
      bottomLeftWedge = pose.R*bottomLeftWedge;
      
      topRightWedge = topRightWedge + repmat([pose.position(1) pose.position(2)], [length(topRightWedge) 1])';
      bottomLeftWedge = bottomLeftWedge + repmat([pose.position(1) pose.position(2)], [length(bottomLeftWedge) 1])';

      fill(topRightWedge(1,:), topRightWedge(2,:), 'k')
      fill(bottomLeftWedge  (1,:), bottomLeftWedge  (2,:), 'k')
    end
    
    
  end
  
end

