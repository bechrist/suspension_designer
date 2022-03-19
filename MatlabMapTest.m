classdef MatlabMapTest < matlab.mixin.indexing.RedefinesParen   
    properties (Access = private)
        Keys (:,1) string
        Values (:,1) cell
    end
    
    methods (Static, Access = public)
        function obj = empty(varargin)
            if nargin == 0
                obj = MyMap(string.empty(0,1),cell.empty(0,1));
                return;
            end
            
            keys = string.empty(varargin{:});
            if ~all(size(keys) == [0, 1])
                error("MyMap:MustBeEmptyColumnVector",...
                    "The only supported empty size is 0x1.");
            end
            
            obj = MyMap(keys,cell.empty(varargin{:}));
        end
    end
    
    methods (Access = public)
        function obj = MyMap(keys_in,values_in)
            if nargin == 0
                obj = MyMap.empty(0,1);
                return;
            end
            
            narginchk(2,2);
            
            if ~all(size(keys_in) == size(values_in))
                error("MyMap:InputSizesDoNotMatch",...
                    "The sizes of the input keys and values must match.");
            end
            
            obj.Keys = keys_in;
            obj.Values = values_in;
        end
        
        function keys = getKeys(obj)
            keys = obj.Keys;
        end
    
        function values = getValues(obj)
            values = obj.Values;
        end
        
        function varargout = size(obj,varargin)
            [varargout{1:nargout}] = size(obj.Keys,varargin{:});
        end

        function C = cat(~,varargin)
            error("MyMap:ConcatenationNotSupported",...
                "Concatenation is not supported.");
        end

        function lastKey = end(~,~,~)
            error("MyMap:EndNotSupported",...
                "Using end with MyMap objects is not supported.");
        end
    end

    methods (Access = private)
        function [keyExists,idx] = convertKeyToIndex(obj,keyCellArray)
            arguments
                obj
                keyCellArray cell {validateKeys}
            end

            requestedKey = keyCellArray{1};
            idx = find(contains(obj.Keys,requestedKey));
            keyExists = ~isempty(idx);
        end
    end
    
    methods (Access = protected)
        function varargout = parenReference(obj,indexOp)      
            [keyExists,idx] = convertKeyToIndex(obj,indexOp(1).Indices);

            if ~keyExists
                error("MyMap:KeyDoesNotExist",...
                    "The requested key does not exist.");
            end

            if numel(indexOp) == 1
                nargoutchk(0,1);
                varargout{1} = obj.Values{idx};
            else
                [varargout{1:nargout}] = obj.Values{idx}.(indexOp(2:end));
            end
        end
        
        function obj = parenAssign(obj,indexOp,varargin)
            indicesCell = indexOp(1).Indices;
            [keyExists,idx] = convertKeyToIndex(obj,indicesCell);

            if numel(indexOp) == 1
                value = varargin{1};
                if keyExists
                    obj.Values{idx} = value;
                else
                    obj.Keys(end+1) = indicesCell{1};
                    obj.Values{end+1} = value;
                end
                return;
            end

            if ~keyExists
                error("MyMap:MultiLevelAssignKeyDoesNotExist", ...
                    "Assignment failed because key %s does not exist",...
                indicesCell{1});
            end

            [obj.Values{idx}.(indexOp(2:end))] = varargin{:};
        end

        function obj = parenDelete(obj,indexOp)
            [keyExists,idx] = convertKeyToIndex(obj,indexOp(1).Indices);
            if keyExists
                obj.Keys(idx) = [];
                obj.Values(idx) = [];
            else
                error("MyMap:DeleteNonExistentKey",...
                    "Unable to perform deletion. The key %s does not exist.",...
                indexOp(1).Indices{1});
            end
       end
    
       function n = parenListLength(obj,indexOp,indexingContext)
            [keyExists,idx] = convertKeyToIndex(obj,indexOp(1).Indices);
            if ~keyExists
                if indexingContext == matlab.indexing.IndexingContext.Assignment
                    error("MyMap:MultiLevelAssignKeyDoesNotExist", ...
                        "Unable to perform assignment. Key %s does not exist",...
                    indexOp(1).Indices{1});
                end
                error("MyMap:KeyDoesNotExist",...
                    "The requested key does not exist.");
            end   
            n = listLength(obj.Values{idx},indexOp(2:end),indexingContext);
        end
    end
end