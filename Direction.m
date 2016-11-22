classdef Direction
    %DIRECTION Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
    end
    
    methods
        function f = force(obj)
            switch obj
                case Direction.NS
                    f = [0;-1];
                case Direction.SN
                    f = [0;1];
                case Direction.EW
                    f = [-1;0];
                case Direction.WE
                    f = [1;0];
            end
        end
        function [opp] = getopposite(obj)
            switch obj
                case Direction.NS
                    opp = Direction.SN;
                case Direction.SN
                    opp = Direction.NS;
                case Direction.EW
                    opp = Direction.WE;
                case Direction.WE
                    opp = Direction.EW;
            end
        end
    end
    
    enumeration
       EW, WE, NS, SN
    end
end

