classdef KinematicSystem
    properties
        Name     (1,1) string    = ""       % System Name
        Date     (1,1) datetime  = datetime % Date Creation
        Graph    (1,1) graph     = graph()  % System Graph
    end
    
    methods
        function obj = KinematicSystem(Type)
            % Generate Kinematic System by default type
            if nargin == 0
                return
            else
                switch Type
                    case "Double Wishbone"
                        obj.Graph = DoubleWishboneInit() 
                
        end
        
        function obj = AppendSystem(obj,Type)
            
        end
        
        function obj = AddPoI(obj, FoI, Name, Key, Position)
            FoI = strcmp([obj.Name],FoI);
            
            Point.Name     = Name;
            Point.Key      = Key; 
            Point.Position = Position;
            
            obj(FoI).PoI = [obj(FoI).PoI; Point];
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
        
        function P = EvaluatePoI(obj, P0, FoI, FoE)
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