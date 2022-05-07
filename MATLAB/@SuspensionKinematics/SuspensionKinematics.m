classdef SuspensionKinematics
    properties
        Name     (1,1) string    = string()  % System Name
        Date     (1,1) datetime  = datetime  % Creation Date & time
        Target   (1,1) struct    = struct()  % Design Targets
        Bound    (1,1) struct    = struct()  % Design Bounds
        Sample   (1,1) struct    = struct()  % Design Sample
        Frame    (1,1) graph     = graph()   % Frame Graph
        Path           uint8     = [];       % Shortest Path Tree
    end
    
    methods(Static)
        %% Transformation Operations
        function T = Translation(x0)
            T = eye(4); T(1:3,4) = x0;
        end
        
        function R = Rotation(Theta, Seq)
            R = eye(4);
            for k = 1:numel(Seq)
                switch Seq(k)
                    case {1,'x',"x"}
                        R = [1 0              0             0; ...
                             0 cos(Theta(k)) -sin(Theta(k)) 0; ...
                             0 sin(Theta(k))  cos(Theta(k)) 0; ...
                             0 0              0             1] * R;
                    case {2,'y',"y"}
                        R = [ cos(Theta(k)) 0 sin(Theta(k)) 0; ...
                              0         1   0               0; ...
                             -sin(Theta(k)) 0 cos(Theta(k)) 0; ...
                              0         0   0               1] * R;
                    case {3,'z',"z"}
                        R = [cos(Theta(k)) -sin(Theta(k)) 0 0; ...
                             sin(Theta(k))  cos(Theta(k)) 0 0; ...
                             0              0             1 0; ...
                             0              0             0 1] * R;
                end
            end
        end
        
        function pFollower = ForwardTransform(pBase, Theta, x0)
            pFollower = ...
                SuspensionKinematics.Rotation(-Theta(3:-1:1),'zyx') * ...
                SuspensionKinematics.Translation(-x0) * ...
                [pBase; ones(1,size(pBase,2))];
            
            pFollower = pFollower(1:3,:);
        end
        
        function pBase = ReverseTransform(pFollower, Theta, x0) 
            pBase = SuspensionKinematics.Translation(x0) * ...
                SuspensionKinematics.Rotation(Theta,'xyz') * ...
                [pFollower; ones(1,size(pFollower,2))];
                
            pBase = pBase(1:3,:);
        end
    end
    
    methods
        %% Constructor / Initialization
        function obj = SuspensionKinematics(Name, Target, Bound)
            % Input Validation
            if nargin == 0
                return
            end
            
            % Generate Suspension System
            obj.Name   = Name;
            obj.Target = Target;
            obj.Bound  = Bound;
            
            switch Target.Type.Linkage
                case "Double Wishbone"
                    obj = obj.DoubleWishboneInit();
                case "Multilink"
                    error("Multilink is Not Yet Implemented")
                otherwise
                    error("Linkage Type Not Recognized")
            end 
        end
        
        %% Design Generation
        function obj = GenerateLinkage(obj)
            switch obj.Target.Type.Linkage
                case "Double Wishbone"
                    obj = DoubleWishboneDesign(obj);
                case "Multilink"
                    error("Multilink is Not Yet Implemented")
                otherwise
                    error("Linkage Type Not Recognized")
            end
        end
        
        %% System Extension
        %{
        function obj = AppendSystem(obj,Type)
            
        end
        %}
        
        %{
        function obj = AddPoI(obj, FoI, Name, Key, Position)
            FoI = strcmp([obj.Name],FoI);
            
            Point.Name     = Name;
            Point.Key      = Key; 
            Point.Position = Position;
            
            obj(FoI).PoI = [obj(FoI).PoI; Point];
        end
        %}
        
        %% Point of Interest
        function P = EvaluatePoint(obj, P0, F0, F)
            obj.Frame.Nodes;
            
            % Input Parsing
            F0 = FrameInputValidation(obj, F0);
            F  = FrameInputValidation(obj, F);
            
            if isstring(P0)
                P0 = obj.Frame.Nodes{F0,"PoI"}{:}{P0,"Position"}{:};
            elseif isnumeric(P0) 
                if isscalar(P0)
                    P0 = obj.Frame.Nodes{F0,"PoI"}{:}{P0,"Position"}{:};
                end
            else
                error("P0 Must be a Valid String or Numeric")
            end
                
            % Graph Traversal & Successive Transformations
            P  = P0;
            Fo = F0;
            while F0 ~= F
                Fn = obj.Path(F,Fo);
                
                if Fn == 0
                    break
                elseif Fn > Fo
                    Theta = obj.Frame.Nodes{Fn,"Rotation"}{:};
                    x0    = obj.Frame.Nodes{Fn,"Position"}{:};
                    
                    P = SuspensionKinematics.ForwardTransform(P,Theta,x0);
                else 
                    Theta = obj.Frame.Nodes{Fo,"Rotation"}{:};
                    x0    = obj.Frame.Nodes{Fo,"Position"}{:};
                    
                    P = SuspensionKinematics.ReverseTransform(P,Theta,x0);
                end
                
                Fo = Fn;
            end
            
            %%% Local Functions
            function F = FrameInputValidation(obj, F)
                if isnumeric(F)
                    F = floor(F);

                    if (F < 1) || (F > height(obj.Frame.Nodes))
                        error("Numeric Frame Index Out of Range")
                    end
                elseif isstring(F)
                    F = find(F == string(obj.Frame.Nodes.Name));

                    if isempty(F)
                        error("Frame String Key Does Not Exist")
                    end
                else
                    error("Frame Must be a Valid String or Numeric")
                end
            end
        end
        
        function obj = SetPoint(obj, P0, F0, P, F)
            if nargin < 5
                F = F0;
            end
            
            P = obj.EvaluatePoint(P,F,F0);
            
            P0 = char(P0);
            if ismember(P0, fieldnames(obj.Bound))
                str1 = '';
                
                switch P0(2)
                    case 'A'
                        str1 = [str1, 'Inner '];
                    case 'B'
                        str1 = [str1, 'Outer '];
                end
                
                switch P0(1)
                    case 'L'
                        str1 = [str1, 'Lower A-Arm '];
                    case 'U'
                        str1 = [str1, 'Upper A-Arm '];
                    case 'T'
                        str1 = [str1, 'Tie Rod '];
                end
                
                if length(P0) == 3 
                    switch P0(3)
                        case 'F'
                            str1 = [str1, 'Front '];
                        case 'R'
                            str1 = [str1, 'Rear '];
                    end
                end
                
                str1 = [str1, 'Pickup (', P0, ') '];
                
                str2 = {'Longitudinal'; 'Lateral'; 'Vertical'};
                for i = 1:3
                    if P(i)<obj.Bound.(P0)(i,1) || obj.Bound.(P0)(i,2)<P(i)
                        warning('SuspensionKinematics:ValueOutofRange', ...
                            [str1, 'Exceeds ', str2{i},' Bounds\n', ...
                                'Min: ', num2str(obj.Bound.(P0)(i,1),3), ...
                              ', Max: ', num2str(obj.Bound.(P0)(i,2),3), ...
                          ', Current: ', num2str(P(i),3)]);   
                    end
                end
            end
            
            obj.Frame.Nodes{F0,"PoI"}{:}{P0,"Position"}{:} = P;
        end
        
        %% Plotting
        function PlotPoint(obj, PoI, FoI, Style)
            if isstring("PoI")
                Style = obj.Frame.Nodes{FoI,"PoI"}{:}{PoI,"Style"}{:};
            elseif nargin < 4
                Style = 'kx';
            end
            
            P = obj.EvaluatePoint(PoI, FoI, "I");
           
            plot3(P(1,:), P(2,:), P(3,:), Style); hold on;
        end
        
        function PlotFrame(obj, FoI, Color)
            if nargin < 3
                Color = 'k';
            end
            % Evaluate Frame Axes
            O  = obj.EvaluatePoint("O" , FoI, "I");
            E1 = obj.EvaluatePoint("E1", FoI, "I");
            E2 = obj.EvaluatePoint("E2", FoI, "I");
            E3 = obj.EvaluatePoint("E3", FoI, "I");
            
            quiver3(O(1)*ones(3,1), O(2)*ones(3,1), O(3)*ones(3,1), ...
                [E1(1) E2(1) E3(1)]'-O(1), [E1(2) E2(2) E3(2)]'-O(2), ...
                [E1(3) E2(3) E3(3)]'-O(3), 'Color', Color);
            
            hold on;
            xlabel('x')
            ylabel('y')
            zlabel('z')
            axis equal;
        end
        
        function PlotSystem(obj)
            for k = 1:height(obj.Frame.Nodes)
                obj.PlotFrame(k)
                
                if height(obj.Frame.Nodes{k,"PoI"}{:}) > 4
                    for p = 5:height(obj.Frame.Nodes{k,"PoI"}{:})
                        obj.PlotPoint(p,k)
                    end
                end
            end
            
            xlim(obj.Target.Wheelbase.*[-1 1])
            ylim(obj.Target.Track.*[-1 1])
            zlim([0, 500])
        end
    end
end