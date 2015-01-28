classdef GridFigure
  %% GridFigure wraps a figure with subplots. Somewhat more pleasant to use than subplot().
  % e.g.
  % myFigure = GridFigure(3, 3);
  % myFigure = myFigure.nextAvailableSubplot(); % activates the next unused plot, in a left/right, top/down fashion
  % peaks(5);
  % myFigure = myFigure.nextAvailableSubplot();
  % peaks(7);
  % myFigure = myFigure.subplot(2, 2); % explicitly place the next one at row,col
  % peaks(9);
  % myFigure = myFigure.subplotFromIndex(9); % expliclity place with subplot index
  % peaks(11);
  % myFigure = myFigure.subplotFromIndex(7);
  % peaks(13);
  %
  
  properties
    
    rows;
    cols;
    figureHandle;
    
    subplotHandles = [];
  end
  
  methods
    
    function [this] = GridFigure(rows, cols)
      this.figureHandle = figure('color', 'w', 'position', [100 100 800 600]);
      this.rows = rows;
      this.cols = cols;
      this.subplotHandles = NaN * ones(rows, cols);
    end
    
    function [] = activate(this)
      figure(this.figureHandle);
    end
    
    function [this, thisHandle] = subplot(this, row, col)
      thisHandle = this.subplotHandles(row, col);
      if (isnan(thisHandle))
        %         thisHandle = subplot(this.rows, this.cols, col + this.cols * (row - 1));
        thisHandle = subplot(this.rows, this.cols, sub2ind([this.rows, this.cols], row, col));
        this.subplotHandles(row, col) = thisHandle;
      else
        axes(thisHandle);
      end
    end
    
    function [this, thisHandle] = subplotFromIndex(this, index)
      [row, col] = ind2sub([this.rows, this.cols], index);
      thisHandle = this.subplotHandles(row, col);
      if (isnan(thisHandle))
        thisHandle = subplot(this.rows, this.cols, index);
        this.subplotHandles(row, col) = thisHandle;
      else
        axes(thisHandle);
      end
    end
    
    function [this, thisHandle] = nextAvailableSubplot(this)
      for i = 1:(this.rows * this.cols)
        [row, col] = ind2sub([this.rows, this.cols], i);
        if (isnan(this.subplotHandles(row, col)))
          [this, thisHandle] = subplotFromIndex(this, i);
          return;
        end
      end
      error('Sorry, no more subplots available! Try adding a row or column to this GridFigure when it is created.');
    end
    
    function [] = prettyUpAllSubfigs(this)
      for i = 1:this.rows
        for j = 1:this.cols
          thisHandle = this.subplotHandles(i, j);
          if (~isnan(thisHandle))
            axes(this.subplotHandles(i, j));
            box off;
            set(gca, 'xtick', []);
            set(gca, 'ytick', []);
          end
        end
      end
    end
    
  end
  
end

