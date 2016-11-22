classdef Relation
    %RELATION Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        direction;
        from;
        to;
        id;
    end
    
    methods
        
        function obj = Relation(id, from, to, direction)
            if nargin
                obj.id = id;
                obj.from = from;
                obj.to = to;
                obj.direction = direction;
            end
        end
        
    end
    
end

