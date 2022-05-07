function obj = DoubleWishboneDesign(obj)    
    %% DoubleWishboneDesign
    % Populates a multibody frame graph with the static frame
    % transforms and point of interest locations based on the design
    % targets and sample of the design space described by the bounds.
    
    %% Allocate Known Parameters   
    % Body Frame
    obj.Frame.Nodes{"B","Position"}{:}(3) = obj.Target.CG(3);
    obj.Frame.Nodes{"B","Rotation"}{:}(2) = obj.Target.Rake;

    % Tire Frame
    obj.Frame.Nodes{"T","Position"}{:} = ...
        [obj.Target.CG(1); obj.Target.Track/2; 0];
    
    obj.Frame.Nodes{"T","Rotation"}{:} = ...
        [-obj.Target.Camber; 0; obj.Target.Toe];
    
    % Wheel Frame
    obj.Frame.Nodes{"W","Position"}{:}(3) = ...
        obj.Target.LoadedRadius / sin(pi/2 - obj.Target.Camber);
    
    obj.Frame.Nodes{"W","Rotation"}{:}(2) = -obj.Target.Caster;
    
    % Axle Frame
    obj.Frame.Nodes{"X","Position"}{:} = ...
        [obj.Target.CG(1); 0; obj.Target.Ride - obj.Target.CG(3)];
    
    %% Design Space Sampling
    for PoI = {'LAF', 'LAR', 'UAF', 'UAR', 'TA', 'LB', 'UB', 'TB'}
        if strcmp(PoI{:}(2),'A')
            obj.Frame.Nodes{"X","PoI"}{:}{PoI{:},"Position"}{:} = ...
                obj.Bound.(PoI{:})(:,1) + obj.Sample.(PoI{:}) .* ...
                diff(obj.Bound.(PoI{:}),1,2);
        elseif strcmp(PoI{:}(2),'B')
            obj.Frame.Nodes{"W","PoI"}{:}{PoI{:},"Position"}{:} = ...
                obj.Bound.(PoI{:})(:,1) + obj.Sample.(PoI{:}) .* ...
                diff(obj.Bound.(PoI{:}),1,2);
        else
            error(['Point Label not Recognized: ', PoI{:}])
        end
        
        if strcmp(PoI{:}, 'LAR')
            if all(obj.Bound.(PoI{:})(2,:) == 0)
                obj.Frame.Nodes{"X","PoI"}{:}{"LAR","Position"}{:}(2) = ...
                    obj.Frame.Nodes{"X","PoI"}{:}{"LAF","Position"}{:}(2);
                
                obj.Bound.('LAR')(2,:) = obj.Bound.('LAF')(2,:);
            end
        elseif strcmp(PoI{:}, 'UAF')
            if all(obj.Bound.(PoI{:})(1,:) == 0)
                obj.Frame.Nodes{"X","PoI"}{:}{"UAF","Position"}{:}(1) = ...
                    obj.Frame.Nodes{"X","PoI"}{:}{"LAF","Position"}{:}(1);
                
                obj.Bound.('UAF')(1,:) = obj.Bound.('LAF')(1,:);
            end
        elseif strcmp(PoI{:}, 'UAR')
            if all(obj.Bound.(PoI{:})(1,:) == 0)
                obj.Frame.Nodes{"X","PoI"}{:}{"UAR","Position"}{:}(1) = ...
                    obj.Frame.Nodes{"X","PoI"}{:}{"LAR","Position"}{:}(1);
                
                obj.Bound.('UAR')(1,:) = obj.Bound.('LAR')(1,:);
            end

            if all(obj.Bound.(PoI{:})(2,:) == 0)
                obj.Frame.Nodes{"X","PoI"}{:}{"UAR","Position"}{:}(2) = ...
                    obj.Frame.Nodes{"X","PoI"}{:}{"UAF","Position"}{:}(2);
                
                obj.Bound.('UAR')(2,:) = obj.Bound.('UAF')(2,:);
            end
        end
    end
    
    %% Tire Kinematics Design
    % % 1. Instant Centers % %
    % Front View Swing Arm Length (FVSA)
    obj.Target.FVSA = 1 ./ atan(abs(obj.Target.CamberGain)); 
        
    % Roll Center PoI
    obj.Frame.Nodes{"I","PoI"}{:}{"RC","Position"}{:} = ...
        [obj.Frame.Nodes{"T","Position"}{:}(1); 0; ...
         obj.Target.CG(3).*obj.Target.RollCenter/100]; 
    
    % Static Roll Instant Center
    obj.Frame.Nodes{"I","PoI"}{:}{"FC","Position"}{:}(1) = ...
        obj.Frame.Nodes{"T","Position"}{:}(1);

    obj.Frame.Nodes{"I","PoI"}{:}{"FC","Position"}{:}(2:3) = ...
        InstantCenterCalc(obj.Frame.Nodes{"T","Position"}{:}(2), ...
        obj.Target.RollCenter, obj.Target.FVSA, obj.Target.LoadedRadius);
       
    % Side View Swing Arm Length (SVSA)
    obj.Target.SVSA = 1 ./ atan(abs(obj.Target.CasterGain)); 
    
    % Pitch Center PoI
    obj.Frame.Nodes{"I","PoI"}{:}{"PC","Position"}{:} = ...
        [0; obj.Frame.Nodes{"T","Position"}{:}(2); ...
         obj.Target.CG(3).*obj.Target.PitchCenter/100]; 
    
    % Static Pitch Instant Center
    obj.Frame.Nodes{"I","PoI"}{:}{"SC","Position"}{:}(2) = ...
        obj.Frame.Nodes{"T","Position"}{:}(2);

    obj.Frame.Nodes{"I","PoI"}{:}{"SC","Position"}{:}([1 3]) = ...
        InstantCenterCalc(obj.Frame.Nodes{"T","Position"}{:}(1), ...
        obj.Target.PitchCenter, obj.Target.SVSA, obj.Target.LoadedRadius);
    
    % % 2. Outboard Upper A-Arm and Tie Rod Pickups % %
    obj = OutboardPickupCalcs(obj);
    
    % % 3. Inboard Lower & Upper A-Arm Pickups % % 
    obj = InboardPickupCalcs(obj);

    %% Local Calculation Functions
    function IC = InstantCenterCalc(x0,hc,L,rl)
        % Finds intersection between jacking line and swing arm circle
        if L == Inf
            L = 1e6; % Swing Arm Approximated to be 1km (Very Long)
        end
        
        if hc == 0
            IC(2) = 0;
        else
            aa = 1 + (x0./hc).^2;
            bb = -2.*rl;
            cc = rl.^2 - L.^2;

            IC(2) = (-bb + sign(hc).*sqrt(bb.^2 - 4.*aa.*cc))./(2.*aa);
        end
        
        IC(1) = x0 - sign(x0).*sqrt(L.^2 - (IC(2)-rl).^2);
    end

    function obj = OutboardPickupCalcs(obj)
        %%% Upper Ball Joint Lateral Position
        if obj.Frame.Nodes{"X","Position"}{:}(1) > 0
            % Front Axle - Place Via KPI Target
            %
            % Note: this is a simplified definition of KPI. The official
            % definition of KPI would require a pure shift of UB in the
            % intermediate frame. This in turn would shift UB
            % longitudinally within the wheel frame due to static toe and 
            % camber. This was deemed as unnecessary complication.
            
            LB = obj.EvaluatePoint("LB", "W", "W"); % Lower Ball Joint
            UB = obj.EvaluatePoint("UB", "W", "W"); % Upper Ball Joint
            
            UB(2) = LB(2) - (UB(3)-LB(3)).*tan(obj.Target.KPI);

            obj = obj.SetPoint("UB","W",UB);
        else
            % Rear Axle - Place Via Mechanical Scrub Target
            error("Review Scrub Implmentation")
            %{
            LB = obj.EvaluatePoint("LB", "W", "I"); % Lower Ball Joint
            UB = obj.EvaluatePoint("UB", "W", "I"); % Upper Ball Joint

            CP = obj.EvaluatePoint("O" , "T", "I"); % Contact Patch
        
            sm = obj.Rotation(obj.Frame.Nodes{"T","Rotation"}{:}(3), 'z') * ...
                [0; -obj.Target.Scrub; 0; 1];
        
            sm = sm(1:3);
        
            % Kingpin Axis Ground Intersection 
            UB(2) = interp1([0, LB(3)], [CP(2)-sm(2), LB(2)], UB(3));
            %}
        end
    end

    function obj = InboardPickupCalcs(obj) 
        % Evaluate Instant Centers in Axle Coordinate
        FC  = obj.EvaluatePoint("FC", "I", "X");
        SC  = obj.EvaluatePoint("SC", "I", "X");
        
        % Tie Rod Inboard Vertical Position
        TB = obj.EvaluatePoint("TB", "W", "X");
        
        TA = obj.EvaluatePoint("TA", "X", "X");
        
        z_TR  = ThreePointPlane(TB, FC, SC, "z");
        TA(3) = z_TR(TA(1), TA(2));
        
        obj = obj.SetPoint("TA", "X", TA);
        
        % Lower A-Arm Inboard Vertical Position
        LB  = obj.EvaluatePoint("LB", "W", "X");
        
        LAF = obj.EvaluatePoint("LAF", "X", "X");
        LAR = obj.EvaluatePoint("LAR", "X", "X");

        z_LA   = ThreePointPlane(LB, FC, SC, "z");
        n_LA   = ThreePointPlane(LB, FC, SC, "n");
        LAF(3) = z_LA(LAF(1), LAF(2));
        LAR(3) = z_LA(LAR(1), LAR(2));
        
        obj = obj.SetPoint("LAF", "X", LAF);
        obj = obj.SetPoint("LAR", "X", LAR);
        
        % Inboard Upper A-Arm Pickups
        UB = obj.EvaluatePoint("UB", "W", "X");
        
        UAF = obj.EvaluatePoint("UAF", "X", "X");
        UAR = obj.EvaluatePoint("UAR", "X", "X");
        
        n_UA = ThreePointPlane(UB, FC , SC , "n");
        n_IP = ThreePointPlane(TA, LAF, LAR, "n");
        
        L_UA = PlanarIntersection(TA, n_IP, UB, n_UA);
        
        obj = obj.SetPoint("UAF", "X", L_UA(UAF(1)));
        obj = obj.SetPoint("UAR", "X", L_UA(UAR(1)));
        
        % Place A-Arm Frames (Via Revolute Joints)
        SetAArmFrame("L");
        SetAArmFrame("U");

        % Place Tie Rod Frame
        obj.Frame.Nodes{"TR","Position"}{:} = ...
            obj.EvaluatePoint("TA", "X", "X");
        
        TB = obj.EvaluatePoint("TB", "W", "TR");
        obj.Frame.Nodes{"TR","Rotation"}{:}(3) = -atan2(TB(1), TB(2));
        
        TB = obj.EvaluatePoint("TB", "W", "TR");
        obj.Frame.Nodes{"TR","Rotation"}{:}(1) = atan2(TB(3), TB(2));
        
        TB = obj.EvaluatePoint("TB", "W", "TR");
        obj.Frame.Nodes{"TR","PoI"}{:}{"TB","Position"}{:}(2) = TB(2);
        
        % Static Member Planes
        function P = ThreePointPlane(A,B,C,Form)    
            % 
            % Create Normal Vector
            n = cross(A-B,A-C);
            n = n / norm(n);
            
            switch Form
                case "z"
                    P = @(x,y) -(n(1).*(x-A(1))+n(2).*(y-A(2)))/n(3)+A(3);
                case "n"
                    P= n;
                otherwise
                    
            end
        end
        
        % Planar Intersection Line
        function L = PlanarIntersection(p1, n1, p2, n2)
            d1 = dot(n1,p1);
            d2 = dot(n2,p2);
            
            n12 = dot(n1,n2);
            
            s = cross(n1,n2);
            s = s ./ s(1);
            
            p0 = (n1*(d1-d2*n12) + n2*(d2-d1*n12)) / (1 - n12^2);
            p0 = p0 - p0(1)*s;
            
            L = @(x) p0 + x*s; 
        end
        
        % Project Point onto Line
        function P0 = LineProjection(L, P)
            A = L(0);
            B = L(1);
            
            s = B-A;
            v = P-A;
            
            P0 = A + dot(s,v)/dot(s,s)*s;
        end
        
        % A-Arm Frame Population
        function SetAArmFrame(M)
            B = obj.EvaluatePoint(strcat(M,"B"), "W", "X");
            n = ThreePointPlane(B, FC , SC , "n");
            
            L = PlanarIntersection(TA, n_IP, B, n);
        
            obj.Frame.Nodes{strcat(M,"A"),"Position"}{:} = ...
                LineProjection(L, B);

            AF = obj.EvaluatePoint(strcat(M,"AF"), "X", strcat(M,"A"));
            obj.Frame.Nodes{strcat(M,"A"),"Rotation"}{:}(3) = ...
                atan2(AF(2), AF(1));

            AF = obj.EvaluatePoint(strcat(M,"AF"), "X", strcat(M,"A"));
            obj.Frame.Nodes{strcat(M,"A"),"Rotation"}{:}(2) = ...
                -atan2(AF(3), AF(1)); 

            B  = obj.EvaluatePoint(strcat(M,"B"), "W", strcat(M,"A"));
            obj.Frame.Nodes{strcat(M,"A"),"Rotation"}{:}(1) = ...
                atan2(B(3),B(2));

            AF = obj.EvaluatePoint(strcat(M,"AF"), "X", strcat(M,"A"));
            AR = obj.EvaluatePoint(strcat(M,"AR"), "X", strcat(M,"A"));
            B  = obj.EvaluatePoint(strcat(M,"B") , "W", strcat(M,"A"));
            
            obj.Frame.Nodes{strcat(M,"A"),"PoI"}{:} ...
                {strcat(M,"AF"),"Position"}{:}(1) = AF(1);
            
            obj.Frame.Nodes{strcat(M,"A"),"PoI"}{:} ...
                {strcat(M,"AR"),"Position"}{:}(1) = AR(1);
            
            obj.Frame.Nodes{strcat(M,"A"),"PoI"}{:} ...
                {strcat(M,"B"),"Position"}{:}(2) = B(2); 
        end
    end
end
