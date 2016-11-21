classdef SourceType
    
    properties
        K
    end
    
    methods
        function obj = SourceType(K)
            if nargin > 0
                obj.K = K;
            end
        end
    end
    
    
    enumeration
        REPULSIVE(-1), ATTRACTIVE(+1000), VOID(0)
    end    
end

