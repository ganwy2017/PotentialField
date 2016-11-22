classdef Cell
    %CELL Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        xmin;
        xmax;
        ymin;
        ymax;
        id;
        centerx;
        centery;
    end
    
    methods
        function obj = Cell(id, xmin, xmax, ymin, ymax)
           if nargin > 0 
               obj.id = id;
               obj.xmin  = xmin;
               obj.xmax  = xmax;
               obj.ymin  = ymin;
               obj.ymax  = ymax;
           end
        end
        
        function [rect] = asRectangle(obj)
           width  = obj.xmax - obj.xmin;
           height = obj.ymax - obj.ymin;
           x = obj.xmin;
           y = obj.ymin;
           rect = [x, y, width, height];
        end
        
        function isinside = inside(obj, x, y)
            isinside = x > obj.xmin ...
                    && x < obj.xmax ...
                    && y > obj.ymin ...
                    && y < obj.ymax;
        end
        
        function centerx = getCenterX(obj)
            centerx = obj.xmin + (obj.xmax - obj.xmin) / 2;
        end
        
        function centery = getCenterY(obj)
            centery = obj.ymin + (obj.ymax - obj.ymin) / 2;
        end
    end
    
end

