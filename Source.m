classdef Source
    %SOURCE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        type;
        x;
        y;
        customk = 0;
        force = 0;
    end
    
    methods
        function obj = Source(x, y, type)
            if nargin > 0
                obj.x = x;
                obj.y = y;
                obj.type = type;
            end
        end
    end
end

