function Frame = DoubleWishboneInit(Target, Bound, p0)    
    %% DoubleWishboneInit
    % Generates a multibody frame graph and populates the static frame
    % transforms and point of interest locations based on the design
    % targets and sample (p0) of the design space described by the bounds.
    
    %% Generate Multibody System Graph
    % Default Point of Interest Table
    PoIDef = table( ["Origin"; "x-Axis"; "y-Axis"; "z-Axis"], ...
                    {[0;0;0] ; [25;0;0]; [0;25;0]; [0;0;25]});
    PoIDef.Properties.VariableNames = {'Title', 'Position'};
    PoIDef.Properties.RowNames = ["O", "E1", "E2", "E3"];
    
    % Intermediate Frame (1)
    I = {"Intermediate", string.empty, ...
         zeros(3,1), zeros(3,1), ...
         false(6,1), PoIDef};
    
    % Body Frame (2)
    B = {"Body", "I", ...
         [0; 0; Target.Ride], [0; Target.Rake; 0], ...
         false(6,1), PoIDef};

    % Tire and Wheel Frames (3,4)
    T = {"Tire", "I", ...
         [Target.CG(1); Target.Track/2; 0], [-Target.Camber; 0; Target.Toe], ...
         logical([1 1 0 1 0 1]'), PoIDef};
     
    W = {"Wheel", "T", ...
        [0; 0; Target.Rl], [0; -Target.Caster; 0], ...
        logical([0 0 0 0 1 0]'), PoIDef};
    
    W{6} = [W{6}; {"Lower Pickup", zeros(3,1)}];
    W{6} = [W{6}; {"Upper Pickup", zeros(3,1)}];
    W{6} = [W{6}; {"Tie Rod Pickup", zeros(3,1)}];
    W{6}.Properties.RowNames = [PoIDef.Properties.RowNames; "LB"; "UB"; "TB"];
    
    % Axle Frame (5)
    X = {"Axle", "B", ...
        [Target.CG(1); 0; 0], zeros(3,1), ...
        false(6,1), PoIDef};
    
    % Suspension Element Frames
    LA = {"Lower A-Arm", "X", ...
          zeros(3,1), zeros(3,1), ...
          logical([0 0 0 1 0 0]'), PoIDef};
      
    UA = {"Upper A-Arm", "X", ...
          zeros(3,1), zeros(3,1), ...
          logical([0 0 0 1 0 0]'), PoIDef};
      
    TR = {"Tie Rod", "X", ...
          zeros(3,1), zeros(3,1), ...
          logical([0 0 0 1 0 1]'), PoIDef};
    
    LA{6} = [LA{6}; {"Apex", zeros(3,1)}];
    LA{6} = [LA{6}; {"Front Pickup", zeros(3,1)}];
    LA{6} = [LA{6}; {"Rear Pickup", zeros(3,1)}];
    LA{6}.Properties.RowNames = [PoIDef.Properties.RowNames; "LB"; "LAF"; "LAR"];
    
    UA{6} = [UA{6}; {"Apex", zeros(3,1)}];
    UA{6} = [UA{6}; {"Front Pickup", zeros(3,1)}];
    UA{6} = [UA{6}; {"Rear Pickup", zeros(3,1)}];
    UA{6}.Properties.RowNames = [PoIDef.Properties.RowNames; "UB"; "UAF"; "UAR"];
    
    % Generate Node Table
    NodeTbl = cell2table([I; B; T; W; X; LA; UA; TR]);
    NodeTbl.Properties.VariableNames = {'Title', ...
        'Base', 'Position', 'Rotation', 'DoF', 'PoI'};
    NodeTbl.Properties.RowNames = ["I", "B", "T", "W", "X", "LA", "UA", "TR"]';
    
    % Generate Graph & Index Table
    A = string(NodeTbl.Properties.RowNames) == ["O", NodeTbl.Base{:}];
    [s,t] = ind2sub(size(A), find(A));
    
    Frame = graph(s, t, ones(size(s)), NodeTbl);
    
    %% Design Space Sampling
    % p0(1) - Inboard Lower A-Arm y-Coord (XCS)    
    Frame.Nodes{"LA","Position"}{:}(2) = ...
        Bound.LA(2,1) + p0(1).*diff(Bound.LA(2,:),1,2);
    
    % p0(2:4) - Inboard Tie Rod (x,y,z)-Coord (XCS)
    Frame.Nodes{"TR","Position"}{:} = ...
        Bound.TA(:,1) + p0(2:4).*diff(Bound.TA,1,2);
    
    % p0(5:7) - Outboard Lower A-Arm (x,y,z)-Coord (WCS)
    Frame.Nodes{"W","PoI"}{:}{"LB","Position"} = ...
        Bound.LB(:,1) + p0(5:7).*diff(Bound.LB,1,2);
    
    % p0(8:9) - Outboard Upper A-Arm (x,z)-Coord (WCS)
    Frame.Nodes{"W","PoI"}{:}{"UB","Position"}([1,3]) = ...
        Bound.UB([1,3],1) + p0(8:9).*diff(Bound.UB([1,3],:),1,2);
        
    % p0(10:11) - Outboard Tie Rod (x,y)-Coord (WCS)
    Frame.Nodes{"W","PoI"}{:}{"TB","Position"}(1:2) = ...
        Bound.TB(1:2,1) + p0(10:11).*diff(Bound.TB(1:2,:),1,2);
    
    %% Roll Geometry
    % 1a. Anti-Roll / Camber Gain -> Instant Roll Center 
    Target.FVSA = 1 ./ ( atan( abs( Target.CamberGain ) ) ); 
        % Front View Swing Arm Length (FVSA) [mm]
    
    Target.RollCenter = Target.CG(3) .* Target.RollCenter/100; 
        % Roll Center Height [mm]
    
    % Swing Arm Circle, Jacking Line Intersection Coefficients
    a = 1 + (2*Target.RollCenter ./ Target.Track).^2;
    b = (4.*Target.RollCenter .* (Target.Rl - Target.RollCenter) ./ ...
        Target.Track - Target.Track);
    c = (Target.Track/2).^2 + (Target.RollCenter-Target.Rl).^2 - Target.FVSA.^2;
    
    % Static Instant Centers [mm,mm]
    Target.RollIC(1) = (-b - sqrt(b.^2 - 4.*a.*c)) ./ (2*a);
    Target.RollIC(2) = -2.*Target.RollCenter.*Target.RollIC(1) ./ ...
        Target.Track + Target.RollCenter;
    
    % 1b. Anti-Pitch / Caster Gain -> Instant Pitch Center
    Target.SVSA = 1 ./ ( atan( abs( Target.CasterGain ) ) ); 
        % Side View Swing Arm Length (SVSA) [mm]
    
    Target.PitchCenter = Target.CG(3) .* Target.PitchCenter/100; 
        % Pitch Center Height [mm]
    
    % Swing Arm Circle, Jacking Line Intersection Coefficients
    a = 1 + (2*Target.PitchCenter ./ Target.Wheelbase).^2;
    b = (4.*Target.PitchCenter .* (Target.Rl - Target.PitchCenter) ./ ...
        Target.Wheelbase - Target.Wheelbase);
    c = (Target.Track/2).^2 + (Target.PitchCenter-Target.Rl).^2 - Target.SVSA.^2;
    
    % Static Instant Centers [mm,mm]
    Target.RollIC(1) = (-b - sqrt(b.^2 - 4.*a.*c)) ./ (2*a);
    Target.RollIC(2) = -2.*Target.RollCenter.*Target.RollIC(1) ./ ...
        Target.Track + Target.RollCenter;
    

end
