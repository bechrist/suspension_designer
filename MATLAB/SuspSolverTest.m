%% Suspension Solver Test
% This script tests a multibody suspension kinematic design object

%% Environment Steup
% Clear Environment
clc; clear; close all;

% Stylization
set(groot,'defaulttextinterpreter','latex');
set(groot,'defaultAxesTickLabelInterpreter','latex');
set(groot,'defaultLegendInterpreter','latex');

%% Targets & Bounds
%%% Vehicle Targets
Target.Wheelbase    = 1525;             % Nominal Wheelbase [mm]
Target.WeightDist   = 0.5;              % Static Front Weight Distribution []
Target.SprungMass   = 225;              % Sprung Mass [kg]
Target.CG(3)        = 8.5  .* (25.4);   % Nominal CG Height [in -> mm]
Target.Ride         = 2    .* (25.4);   % Nominal Ride Height [in -> mm]
Target.Rake         = 0    .* (pi/180); % Nominal Rake Angle [deg -> rad]
Target.LoadedRadius = 7.85 .* (25.4);   % Nominal Loaded Radius [in -> mm]

Target.CG(1) = Target.Wheelbase * (1-Target.WeightDist); % C.G. to Front Axle [mm]

%%% Suspension Targets
Target.Type.Linkage = "Double Wishbone";       % Suspension Linkage Type

Target.Track       =  1220;                    % Nominal Front Track Width [mm]

Target.Toe         =  0.50 .* (pi/180);        % Static Toe (Positive Out) [deg -> rad]

Target.PitchCenter =  10;                      % Normalized FBPC Height [%]
Target.Caster      =  3.00 .* (pi/180);        % Static Caster          [deg -> rad]
Target.CasterGain  =  0.25 .* (pi/(180*25.4)); % Caster Gain            [deg/in -> rad/mm]

Target.RollCenter  =  15;                      % Normalized FBRC Height [%]
Target.Camber      = -1.60 .* (pi/180);        % Static Camber          [deg -> rad]
Target.CamberGain  = -1.00 .* (pi/(180*25.4)); % Camber Gain            [deg/in -> rad/mm]

Target.Scrub       =  0.50 .* (25.4);          % Maximum Scrub [in -> mm]
Target.KPI         =  3.00 .* (pi/180);        % Target KPI    [deg -> rad]

Target.RideRatio   =  0.80;                    % Ride Motion Ratio Target [] 
Target.ARBRatio    =  0.80;                    % ARB Motion Ratio Target  [] 

%%% Double Wishbone Design Space Bounds
% Notes:
% 1. Several bounds will be inherited if left to zero
%    - UAF inherits longitudinal bounds from LAF
%    - UAR inherits longitudinal bounds from LAR
%    - LAR inherits lateral bounds from LAF
%    - UAR inherits lateral bounds from UAF
% 2. Bounds set as NaN are fixed design parameters
%    - PA & SB should be within the y-z rocker plane
%    - PA sits on the y-axis of the rocker plane

% Inboard  Pickups: 
%           |Longitudinal |   Lateral   |   Vertical  | 
Bound.LAF = [  5.00,  5.00;  8.00,  8.70;  0.50,  1.50] .* (25.4); % (X) [in -> mm]
Bound.LAR = [- 5.00,- 5.00;  0   ,  0   ;  0.50,  1.50] .* (25.4); % (X) [in -> mm]
Bound.UAF = [  0   ,  0   ;  8.70, 10.00;  6.00,  8.00] .* (25.4); % (X) [in -> mm]
Bound.UAR = [  0   ,  0   ;  0   ,  0   ;  6.00,  8.00] .* (25.4); % (X) [in -> mm]
Bound.TA  = [  2.00,  3.00;  8.70,  8.70;  2.50,  2.75] .* (25.4); % (X) [in -> mm]

Bound.RA = [- 3.00,  2.00;  6.00,  9.50;  8.00, 10.00] .* (25.4); % (X) [in -> mm]
Bound.PA = [  NaN ,  NaN ;  2.00,  4.00;  NaN ,  NaN ] .* (25.4); % (R) [in -> mm]
Bound.SA = [- 3.00,  0.00;  8.00, 12.00; 10.00, 18.00] .* (25.4); % (X) [in -> mm]

% Outboard Pickups: 
%          |Longitudinal |   Lateral   |   Vertical  | 
Bound.LB = [  0.00,  0.00;- 0.88,- 0.88;- 3.25,- 2.70] .* (25.4); % (W) [in -> mm] 
Bound.UB = [  0.00,  0.00;- 1.75,- 0.88;  3.00,  3.50] .* (25.4); % (W) [in -> mm] 
Bound.TB = [  2.50,  2.85;- 1.25,- 0.88;- 1.50,  0.50] .* (25.4); % (W) [in -> mm]

Bound.PB = [  2.25,  3.75;- 3.00,- 2.00;  1.50,  2.50] .* (25.4); % (A) [in -> mm] 
Bound.SB = [  NaN ,  NaN ;  2.25,  3.75;  1.50,  2.50] .* (25.4); % (R) [in -> mm]

%% Roll-Steer Linkage Design
%%% Initialize Suspension
Susp = SuspensionKinematics("Test", Target, Bound); 

%%% Design Sampling
for P = string(fieldnames(Susp.Sample))'
    Susp.Sample.(P)(Susp.Sample.(P)==true) = 1/2;
end

%%% Design Generation (Static Configuration)
Susp = Susp.GenerateLinkage;

%%% Jounce-Steer Sweep


%% Plotting
Susp.PlotSystem

CP = Susp.EvaluatePoint( "O", "T", "I");
FC = Susp.EvaluatePoint("FC", "I", "I");
SC = Susp.EvaluatePoint("SC", "I", "I");

plot3([CP(1),FC(1)], [CP(2),FC(2)], [CP(3),FC(3)], 'k:')
plot3([CP(1),SC(1)], [CP(2),SC(2)], [CP(3),SC(3)], 'k:')

for M = {'LA','UA','TR'}
    A = Susp.EvaluatePoint("O", string(M), "I");
    B = Susp.EvaluatePoint(string([M{:}(1),'B']), string(M), "I");
    
    if strcmp(M,"TR")
        plot3([A(1),B(1)], [A(2),B(2)], [A(3),B(3)], 'k')
    else
        AF = Susp.EvaluatePoint(string([M{:}(1),'AF']), string(M), "I");
        AR = Susp.EvaluatePoint(string([M{:}(1),'AR']), string(M), "I");
        
        plot3([ A(1), B(1)], [ A(2), B(2)], [ A(3), B(3)], 'k--')
        plot3([AF(1),AR(1)], [AF(2),AR(2)], [AF(3),AR(3)], 'k--')
        
        plot3([AF(1), B(1)], [AF(2), B(2)], [AF(3), B(3)], 'k') 
        plot3([AR(1), B(1)], [AR(2), B(2)], [AR(3), B(3)], 'k') 
        
        plot3([AF(1),SC(1)], [AF(2),SC(2)], [AF(3),SC(3)], 'k:')
    end
    
    plot3([B(1),FC(1)], [B(2),FC(2)], [B(3),FC(3)], 'k:')
end

load('OZShell.mat');
Rim = {"Rim", OZShell, 'k-'};
Susp.Frame.Nodes{"W","PoI"}{:} = [Susp.Frame.Nodes{"W","PoI"}{:}; Rim];
Susp.Frame.Nodes{"W","PoI"}{:}.Properties.RowNames{end} = 'Rim';

Susp.PlotPoint("Rim", "W", "I");