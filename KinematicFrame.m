classdef KinematicFrame
    properties
        Name     (1,1) string  = ""         % Frame Name
        Key      (1,1) string  = ""         % Frame Keys
        Graph    (1,1) digraph = digraph(); % System Graph
        Base     (:,1) string  = ""         % Base Frame
        Origin   (3,1) double  = zeros(3,1) % Origin in Base Frame
        Rotation (3,1) double  = zeros(3,1) % Orientation in Base Frame
        Follower (:,1) string  = ""         % Follower Frame(s)
        PoI      (:,1) struct  = ...        % Point(s) of Interest
            struct('Name'    , {"Origin", "x-Axis", "y-Axis", "z-Axis"}, ...
                   'Key'     , {"O"     , "E1"    , "E2"    , "E3"    }, ...
                   'Position', {[0;0;0] , [25;0;0], [0;25;0], [0;0;25]})
    end
    
    methods
        function obj = KinematicFrame()
            % All Values Set by Default
        end
        
        function obj = AddFrame(obj, Name, Key, Base, Origin, Rotation)
            obj(end+1).Name   = Name;
            obj(end).Key      = Key;
            obj(end).Base     = Base;
            obj(end).Origin   = Origin;
            obj(end).Rotation = Rotation;
            obj(end).Follower = char.empty;
            
            BaseIdx = find(strcmp([obj.Key], Base), 1);
            if strcmp(obj(BaseIdx).Follower,"")
               obj(BaseIdx).Follower = Key;
            else
                obj(BaseIdx).Follower(end+1) = Key;
            end
        end
        
        function obj = AddPoI(obj, FoI, Name, Key, Position)
            FoI = strcmp([obj.Name],FoI);
            
            Point.Name     = Name;
            Point.Key      = Key; 
            Point.Position = Position;
            
            obj(FoI).PoI = [obj(FoI).PoI; Point];
        end
        
        function [Nodes, Weights] = GenerateGraph(obj, FoI, Nodes, Weights)
            FoI = find(strcmp([obj.Key],FoI));
            
            if ~all(strcmp(obj(FoI).Follower,""))
                for j = 1:numel(obj(FoI).Follower)
                    FollowerIdx = find(strcmp([obj.Key],obj(FoI).Follower(j)));

                    Nodes(end+1,:) = [FoI, FollowerIdx];
                    Weights(end+1) = 1;

                    Nodes(end+1,:) = Nodes(end,2:-1:1);
                    Weights(end+1) = -1;

                    [Nodes, Weights] = obj.GenerateGraph( ...
                        obj(FollowerIdx).Key, Nodes, Weights);
                end
            end
        end
        
        function obj = UpdateFrame(obj, FoI, Origin, Rotation)
            FoI = strcmp([obj.Key],FoI);
            
            if ~isempty(Origin)
                obj(FoI).Origin   = Origin;
            end
            
            if ~isempty(Rotation)
                obj(FoI).Rotation = Rotation;
            end
        end
        
        function obj = UpdatePoI(obj, FoI, PoI, Position)
            FoI = strcmp([obj.Key],FoI);
            PoI = strcmp([obj(FoI).PoI.Key], PoI);
            
            obj(FoI).PoI(PoI).Position = Position;
        end
        
        function P = EvaluatePoI(obj, PoI, FoI, FoE)
            FoI = obj(strcmp([obj.Key],FoI));
            
            if isstring(PoI) 
                P0 = FoI.PoI(strcmp([FoI.PoI.Key],PoI)).Position;   
            elseif isnumeric(PoI)
                P0 = PoI; 
            else
                error('PoI Argument of Invalid Type')
            end
            
            if strcmp(FoI.Key, FoE)
                P = P0;
            elseif ismember(FoE, FoI.Follower)
                error('Not Complete')
            elseif strcmp(FoE, FoI.Base) 
                P = InverseTransform(P0, FoI.Origin, FoI.Rotation);
            else
                P0 = InverseTransform(P0, FoI.Origin, FoI.Rotation);
                
                P = obj.EvaluatePoI(P0, FoI.Base, FoE);
            end
            
            function x = ForwardTransform(x0, p, w)
                x = [ cos(w(2))*cos(w(3)), cos(w(2))*sin(w(3)), -sin(w(2)); ...
                     -cos(w(1))*sin(w(3)) + cos(w(3))*sin(w(1))*sin(w(2)), ...
                      cos(w(1))*cos(w(3)) + sin(w(1))*sin(w(2))*sin(w(3)), ...
                      cos(w(2))*sin(w(1)); ...
                      sin(w(1))*sin(w(3)) + cos(w(1))*cos(w(3))*sin(w(2)), ...
                     -cos(w(3))*sin(w(1)) + cos(w(1))*sin(w(2))*sin(w(3)), ...
                      cos(w(1))*cos(w(2)) ] * (x0 - p);
            end

            function x = InverseTransform(x0, p, w)
                x = [ cos(w(2))*cos(w(3)), ...
                     -cos(w(1))*sin(w(3)) + cos(w(3))*sin(w(1))*sin(w(2)), ...
                      sin(w(1))*sin(w(3)) + cos(w(1))*cos(w(3))*sin(w(2)); ...
                      cos(w(2))*sin(w(3)), ...
                      cos(w(1))*cos(w(3)) + sin(w(1))*sin(w(2))*sin(w(3)), ...
                     -cos(w(3))*sin(w(1)) + cos(w(1))*sin(w(2))*sin(w(3)); ...
                     -sin(w(2)), cos(w(2))*sin(w(1)), cos(w(1))*cos(w(2))] * x0 + p;
            end
        end
        
        function PlotFrame(obj, FoI, Color)
            O = obj.EvaluatePoI("O", FoI, "I");
            E1 = obj.EvaluatePoI("E1", FoI, "I");
            E2 = obj.EvaluatePoI("E2", FoI, "I");
            E3 = obj.EvaluatePoI("E3", FoI, "I");
            
            quiver3( O(1)*ones(3,1), O(2)*ones(3,1), O(3)*ones(3,1), ...
                [E1(1) E2(1) E3(1)]'-O(1), [E1(2) E2(2) E3(2)]'-O(2), ...
                [E1(3) E2(3) E3(3)]'-O(3), 'Color', Color);
            
            hold on;
            axis equal;
        end
        
        function PlotPoI( obj, FoI, PoI, Style )
            P = obj.EvaluatePoI(PoI, FoI, "I");
            
            plot3( P(1), P(2), P(3), Style );
        end
    end
end