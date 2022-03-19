%% Suspension Solver Test

%% Targets & Bounds
%%% Vehicle Targets
Target.Wheelbase  = 1525;             % Nominal Wheelbase [mm]
Target.WeightDist = 0.5;              % Static Front Weight Distribution []
Target.SprungMass = 225;              % Sprung Mass [kg]
Target.CG(3)      = 8.5  .* (25.4);   % Nominal CG Height [in -> mm]
Target.Ride       = 2    .* (25.4);   % Nominal Ride Height [in -> mm]
Target.Rake       = 0    .* (pi/180); % Nominal Rake Angle [deg -> rad]
Target.Rl         = 7.85 .* (25.4);   % Nominal Loaded Radius [in -> mm]

Target.CG(1) = Target.Wheelbase * (1-Target.WeightDist); % C.G. to Front Axle (a) [mm]

%%% Suspension Objectives
Target(1).Track       =  1220;                    % Nominal Front Track Width [mm]

Target(1).Toe         =  0.50 .* (pi/180);        % Static Toe (Positive Out) [deg -> rad]

Target(1).AntiDive    =  25;                      % Normalized FBPC Height [%]
Target(1).Caster      =  3.00 .* (pi/180);        % Static Caster [deg -> rad]
Target(1).CasterGain  =  0.05 .* (pi/(180*25.4)); % Caster Gain [deg/in -> rad/mm]

Target(1).AntiRoll    =  25;                      % Normalized FBRC Height [%]
Target(1).Camber      = -1.60 .* (pi/180);        % Static Camber [deg -> rad]
Target(1).CamberGain  = -0.50 .* (pi/(180*25.4)); % Camber Gain [deg/in -> rad/mm]

Target(1).Scrub       =  0.50 .* (25.4);          % Maximum Scrub [in -> mm]
Target(1).KPI         =  5.00 .* (pi/180);        % Target KPI [deg -> rad]

Target(1).RideRatio   =  0.80;                    % Ride Motion Ratio Target [] 
Target(1).ARBRatio    =  0.80;                    % ARB Motion Ratio Target [] 

% Inboard  Pickup: Longitudinal |   Lateral   |   Vertical  | 
Bound(1).LA =     [  3.00,  5.00;  8.00,  8.70;  0.50,  1.50] .* (25.4); % FLA Bounds (XCS) [in -> mm]
Bound(1).UA =     [  0   ,  0   ;  8.70, 10.00;  6.00,  8.00] .* (25.4); % FUA Bounds (XCS) [in -> mm]
Bound(1).TA =     [  2.00,  3.00;  8.70,  8.70;  2.50,  2.75] .* (25.4); % FTA Bounds (XCS) [in -> mm]
Bound(1).RA =     [- 3.00,  2.00;  6.00,  9.50;  8.00, 10.00] .* (25.4); % FRA Bounds (XCS) [in -> mm]
Bound(1).PA =     [  0   ,  0   ;  2.00,  4.00;  0   ,  0   ] .* (25.4); % FPA Bounds (RCS) [in -> mm]
Bound(1).SA =     [- 3.00,  0.00;  8.00, 12.00; 10.00, 18.00] .* (25.4); % FSA Bounds (XCS) [in -> mm]

% Outboard Pickup: Longitudinal |   Lateral   |   Vertical  | 
Bound(1).LB =     [  0.00,  0.00;- 0.88,- 0.88;- 2.70,- 3.25] .* (25.4); % FLB Bounds (TCS) [in -> mm] 
Bound(1).UB =     [  0.00,  0.00;- 0.88,- 1.75;  3.00,  3.50] .* (25.4); % FUB Bounds (TCS) [in -> mm] 
Bound(1).TB =     [  2.50,  2.81;- 0.88,- 1.25;- 1.50,  0.50] .* (25.4); % FTB Bounds (TCS) [in -> mm]
Bound(1).PB =     [  2.25,  3.75;- 1.75,- 0.85;- 2.50,  2.50] .* (25.4); % FPB Bounds (ACS) [in -> mm] 
Bound(1).SB =     [  2.25,  3.75;- 1.75,- 0.85;- 2.50,  2.50] .* (25.4); % FSB Bounds (RCS) [in -> mm]

%% Initialize Frames 
Frame = InitializeFrame( Target );

%% Design Generation
x0 = ones(13,1)/2;

Frame = DesignGeneration(x0, Target, Bound, Frame);

%% Local Function
function Frame = InitializeFrame( Target )
    % Inermediate Frame (1)
    Frame = KinematicFrame();
    Frame.Name = "Intermediate";
    Frame.Tag = "I";
    
    % Body Frame (2)
    Frame = Frame.AddFrame("Body", "B", "I", ...
        [0; 0; Target.Ride], [0; Target.Rake; 0]);
    
    % Tire and Wheel Frames (3,4)
    Frame = Frame.AddFrame("Tire", "T", "I", ...
        [Target.CG(1); Target.Track/2; 0], [-Target.Camber; 0; Target.Toe]);
    Frame = Frame.AddFrame("Wheel", "W", "T", ...
        [0; 0; Target.Rl], [0; -Target.Caster; 0]);
    
    Frame = Frame.AddPoI("Wheel", "Lower Pickup", "LB", zeros(3,1));
    Frame = Frame.AddPoI("Wheel", "Upper Pickup", "UB", zeros(3,1));
    Frame = Frame.AddPoI("Wheel", "Tie Rod Pickup", "TB", zeros(3,1));
    
    % Axle Frame (5)
    Frame = Frame.AddFrame("Axle", "X", "B", [Target.CG(1); 0; 0], zeros(3,1));
    
    % Suspension Element Frames
    Frame = Frame.AddFrame("Lower A-Arm", "LA", "X", zeros(3,1), zeros(3,1));
    Frame = Frame.AddFrame("Upper A-Arm", "UA", "X", zeros(3,1), zeros(3,1));
    Frame = Frame.AddFrame("Tie Rod", "TR", "X", zeros(3,1), zeros(3,1));
    
    Frame = Frame.AddPoI("Lower A-Arm", "Apex", "LB", zeros(3,1));
    Frame = Frame.AddPoI("Lower A-Arm", "Front Pickup", "LAF", zeros(3,1));
    Frame = Frame.AddPoI("Lower A-Arm", "Rear Pickup", "LAR", zeros(3,1));
    
    Frame = Frame.AddPoI("Upper A-Arm", "Apex", "UB", zeros(3,1));
    Frame = Frame.AddPoI("Upper A-Arm", "Front Pickup", "UAF", zeros(3,1));
    Frame = Frame.AddPoI("Upper A-Arm", "Rear Pickup", "UAR", zeros(3,1));
    
    % Compliance Mechanism Frames
    Frame = Frame.AddFrame("Rocker", "RK", "X", zeros(3,1), zeros(3,1));
    Frame = Frame.AddFrame("Shock", "RS", "X", zeros(3,1), zeros(3,1));
end

function Frame = DesignGeneration( x0, Target, Bound, Frame )
    %% Indexing Maps and Dependency Graph
    FMap = containers.Map();
    for i = 1:numel(Frame)
        FMap(Frame(i).Tag)=i;
    end
    
    PMap = containers.Map();
    for i = 1:numel(Frame)
        for j = 1:numel(Frame(i).PoI)
            PMap( strcat(Frame(i).Tag, "-", Frame(i).PoI(j).Tag) ) = j;
        end
    end
    
    [Nodes, Weights] = Frame.GenerateGraph("I", [], []);
    Frame(FMap("I")).Graph = digraph(Nodes(:,1), Nodes(:,2), Weights);
    
    %% Design Space Sampling
    % x0(1)  - Inboard Lower A-Arm y-Coord (X)    
    Frame(FMap("LA")).Origin(2) = Bound.LA(2,1) + x0(1).*diff(Bound.LA(2,:),1,2);
    
    % x0(2)  - Inboard Tie Rod x-Coord (X)
    % x0(3)  - Inboard Tie Rod y-Coord (X)
    % x0(4)  - Inboard Tie Rod z-Coord (X)
    Frame(FMap("TR")).Origin = Bound.TA(:,1) + x0(2:4).*diff(Bound.TA,1,2);
    
    % x0(5)  - Outboard Lower A-Arm x-Coord (W)
    % x0(6)  - Outboard Lower A-Arm y-Coord (W)
    % x0(7)  - Outboard Lower A-Arm z-Coord (W)
    Frame(FMap("W")).PoI(PMap("W-LB")).Position = Bound.LB(:,1) + ...
        x0(5:7).*diff(Bound.LB,1,2);
    
    % x0(8)  - Outboard Upper A-Arm x-Coord (W)
    % x0(9)  - Outboard Upper A-Arm z-Coord (W)
    Frame(FMap("W")).PoI(PMap("W-UB")).Position([1,3]) = Bound.UB([1,3],1) + ...
        x0(8:9).*diff(Bound.UB([1,3],:),1,2);
        
    % x0(10) - Outboard Tie Rod x-Coord (W)
    % x0(11) - Outboard Tie Rod y-Coord (W)
    Frame(FMap("W")).PoI(PMap("W-TB")).Position(1:2) = Bound.TB(1:2,1) + ...
        x0(10:11).*diff(Bound.TB(1:2,:),1,2);
    
    % x0(12) - Lower A-Arm Swing Angle (X)
    Frame(FMap("LA")).Rotation(3) = x0(12);
    
    % x0(13) - Upper A-Arm Swing Angle (X)
    Frame(FMap("UA")).Rotation(3) = x0(13);
    
    %% Roll Design
    % 1a. Anti-Roll / Camber Gain -> Instant Roll Center 
    % 1b. Instant Pitch Center    -> LA (z') 
    % 2.  Kingpin Geometry        -> UB (y)
    % 3.  Bump Steer Colinearity  -> UA (y',z')
    % 4.  Tie Rod Bump Alignment  -> TB (z)
    % 5a. Anti-Dive / Caster Gain -> Instant Pitch Center
    % 5b. Instant Pitch Center    -> LA (alpha_y), UA (alpha_y)
    % 5c. Hinge Consistency       -> LA (x,y,z), UA (x,y,z)
   
    %%% Notes
    % [CG], [AR%], [dgam/dj], [rl], [tw] -> IC_y, IC_z 
    % {LB_y}, {LB_z}, {LA_y} + (IC_y, IC_z) -> LA_z' 
    % [KPI | S_m], {UB_z} + (LB_y, LB_z) -> UB_y
    % {TA_y}, {TA_z} + (LA_y, LA_z, UB_y, UB_z, IC_y, IC_z) -> UA_y', UA_z'
    % {TB_y} + (TA_y, TA_z, IC_y, IC_z) -> TB_z
   
    %%% 1a. Instant Roll Center 
    Target.FVSA = 1 ./ ( atan( abs( Target.CamberGain ) ) ); 
        % Front View Swing Arm Length (FVSA) [mm]
    
    Target.RollCenter = Target.CG(3) .* Target.AntiRoll/100; % Roll Center Height [mm]
    
    % Swing Arm Circle, Jacking Line Intersection Coefficients
    a = 1 + (2*Target.RollCenter ./ Target.Track).^2;
    b = (4.*Target.RollCenter .* (Target.Rl - Target.RollCenter) ./ ...
        Target.Track - Target.Track);
    c = (Target.Track/2).^2 + (Target.RollCenter-Target.Rl).^2 - Target.FVSA.^2;
    
    % Static Instant Centers [mm,mm]
    Target.RollIC(1) = (-b - sqrt(b.^2 - 4.*a.*c)) ./ (2*a);
    Target.RollIC(2) = -2.*Target.RollCenter.*Target.RollIC(1) ./ ...
        Target.Track + Target.RollCenter;
        
    %%% 1b. Inboard Lower A-Arm Pickup (z')
    LA = Frame.EvaluatePoI("O", "LA", "I");
    LB = Frame.EvaluatePoI("LB", "W", "I");
    
    LA(3) = (Target.RollIC(2)-LB(3))./(Target.RollIC(1)-LB(2)) .* ...
        (LA(2) - LB(2)) + LB(3);
    
    error("Need to implement frame dependency tree")
    Frame(FMap("LA")).Origin = Frame.EvaluatePoI(LA, "I", "X");
    
    %%% 2. Outboard Upper A-Arm Pickup (y)
    
    %%% 3. Inboard Upper A-Arm Pickup (y',z')
    
    %%% 4. Outboard Tie Rod Pickup (z)
    
    %%% 5a. Instant Pitch Center
    
    %%% 5b. A-Arm Dive Angles 
    
    %%% 5c. Inboard A-Arm Pickups
    Frame.PlotFrame("I",'k')
    Frame.PlotFrame("X",'r')
    Frame.PlotFrame("T",'k')
    Frame.PlotFrame("W",'b')
    
    scatter3( Frame(FMap("X")).Origin(1), Target.RollIC(1), Target.RollIC(2), 'kx' )

    scatter3( LA(1), LA(2), LA(3), 'rs')
        
    Frame.PlotPoI("W", "LB", 'bs')
    Frame.PlotPoI("W", "TB", 'bs')
    Frame.PlotPoI("W", "UB", 'bs')
    
    plot3( Frame(FMap("X")).Origin(1).*ones(2,1), ...
           [LB(2), Target.RollIC(1)], ...
           [LB(3), Target.RollIC(2)], 'k--' )
   
    plot3( Frame(FMap("X")).Origin(1).*ones(2,1), ...
          [Frame(FMap("T")).Origin(2), Target.RollIC(1)], ...
          [Frame(FMap("T")).Origin(3), Target.RollIC(2)], 'k--' )
end