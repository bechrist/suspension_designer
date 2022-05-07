function obj = DoubleWishboneInit(obj)    
    %% DoubleWishboneInit
    % Initializes a multibody frame graph of a double wishbone linkage

    %% Generate Multibody System Graph
    % Default Point of Interest Table
    DefaultPoI = table( ["Origin"; "x-Axis"; "y-Axis"; "z-Axis"], ...
                    {[0;0;0] ; [25;0;0]; [0;25;0]; [0;0;25]}, ...
                    {'k.'; 'k.'; 'k.'; 'k.'});

    DefaultPoI.Properties.VariableNames = {'Title', 'Position','Style'};
    DefaultPoI.Properties.RowNames = ["O", "E1", "E2", "E3"];

    % Intermediate Frame
    I = {"I", "Intermediate", "", zeros(3,1), zeros(3,1), ...
         false(6,1), DefaultPoI};
    
    I{end} = [I{end}; {"Roll Center"         , zeros(3,1), 'kx'}];
    I{end} = [I{end}; {"Front Instant Center", zeros(3,1), 'k*'}];
    
    I{end} = [I{end}; {"Pitch Center"        , zeros(3,1), 'kx'}];
    I{end} = [I{end}; {"Side Instant Center" , zeros(3,1), 'k*'}];
    
    I{end}.Properties.RowNames = ...
        [DefaultPoI.Properties.RowNames; "RC"; "FC"; "PC"; "SC"];
    
    % Body Frame
    B = {"B", "Body", "I", zeros(3,1), zeros(3,1), ...
        logical([0 0 1 1 1 0]'), DefaultPoI};

    % Tire Frame
    T = {"T", "Tire", "I", zeros(3,1), zeros(3,1), ...
         logical([1 1 0 1 0 1]'), DefaultPoI};

    % Wheel Frame
    W = {"W", "Wheel", "T", zeros(3,1), zeros(3,1), ...
        logical([0 0 0 0 1 0]'), DefaultPoI};

    W{end} = [W{end}; {"Lower Pickup"  , zeros(3,1), 'ks'}];
    W{end} = [W{end}; {"Upper Pickup"  , zeros(3,1), 'ks'}];
    W{end} = [W{end}; {"Tie Rod Pickup", zeros(3,1), 'ks'}];

    W{end}.Properties.RowNames = ...
        [DefaultPoI.Properties.RowNames; "LB"; "UB"; "TB"];

    % Axle Frame
    X = {"X", "Axle", "B", zeros(3,1), zeros(3,1), ...
        false(6,1), DefaultPoI};
    
    X{end} = [X{end}; {"Lower A-Arm Front Pickup", zeros(3,1), 'ks'}];
    X{end} = [X{end}; {"Lower A-Arm Rear Pickup" , zeros(3,1), 'ks'}];
    X{end} = [X{end}; {"Upper A-Arm Front Pickup", zeros(3,1), 'ks'}];
    X{end} = [X{end}; {"Upper A-Arm Rear Pickup" , zeros(3,1), 'ks'}];
    X{end} = [X{end}; {"Tie Rod Pickup"          , zeros(3,1), 'ks'}];
    
    X{end}.Properties.RowNames = ...
        [DefaultPoI.Properties.RowNames; "LAF"; "LAR"; "UAF"; "UAR"; "TA"];
    
    % Suspension Element Frames
    LA = {"LA", "Lower A-Arm", "X", zeros(3,1), zeros(3,1), ...
          logical([0 0 0 1 0 0]'), DefaultPoI};

    LA{end} = [LA{end}; {"Apex"        , zeros(3,1), 'ko'}];
    LA{end} = [LA{end}; {"Front Pickup", zeros(3,1), 'ko'}];
    LA{end} = [LA{end}; {"Rear Pickup" , zeros(3,1), 'ko'}];

    LA{end}.Properties.RowNames = ...
        [DefaultPoI.Properties.RowNames; "LB"; "LAF"; "LAR"];
    
    UA = {"UA", "Upper A-Arm", "X", zeros(3,1), zeros(3,1), ...
          logical([0 0 0 1 0 0]'), DefaultPoI};

    UA{end} = [UA{end}; {"Apex"        , zeros(3,1), 'ko'}];
    UA{end} = [UA{end}; {"Front Pickup", zeros(3,1), 'ko'}];
    UA{end} = [UA{end}; {"Rear Pickup" , zeros(3,1), 'ko'}];

    UA{end}.Properties.RowNames = ...
        [DefaultPoI.Properties.RowNames; "UB"; "UAF"; "UAR"];
    
    TR = {"TR", "Tie Rod", "X", zeros(3,1), zeros(3,1), ...
          logical([0 0 0 1 0 1]'), DefaultPoI};

    TR{end} = [TR{end}; {"Outer Pickup", zeros(3,1), 'ko'}];
    TR{end}.Properties.RowNames = [DefaultPoI.Properties.RowNames; "TB"];
    
    % Generate Node Table
    NodeTable = cell2table([I; B; T; W; X; LA; UA; TR]);

    NodeTable.Properties.VariableNames = {'Name', 'Title', ...
        'Base', 'Position', 'Rotation', 'DoF', 'PoI'};

    NodeTable.Properties.RowNames = ...
        ["I", "B", "T", "W", "X", "LA", "UA", "TR"]';

    % Generate Adjacency Matrix & Graph
    Adjacency = string(NodeTable.Properties.RowNames)==NodeTable.Base(:)';

    [Source, Target] = ind2sub(size(Adjacency), find(Adjacency));

    obj.Frame = graph(Source, Target, [], NodeTable);

    % Generate Shortest Path Tree Vectors
    for k = 1:height(NodeTable)
        obj.Path = [obj.Path; ...
            shortestpathtree(obj.Frame, k, 'OutputForm', 'vector')];
    end

    %% Generate Design Sample Structure
    % This is extremely dependent on the choice of design
    % rules applied during the design generation process. Please
    % refer to DoubleWishboneDesign.m for more details.
    %
    % Key:
    % 1: Value is Sampled
    % 0: Value is Auto-Calculated or Fixed
    
    % Double Wishbone Default Setup
    obj.Sample.LAF = [1 1 0]';
    obj.Sample.LAR = [1 1 0]';
    obj.Sample.UAF = [1 0 0]';
    obj.Sample.UAR = [1 1 0]';
    obj.Sample.TA  = [1 1 1]';

    obj.Sample.LB  = [1 1 1]';
    obj.Sample.UB  = [1 0 1]';
    obj.Sample.TB  = [1 1 0]';

    % Determining if Bounds Restrict Sampled Values
    for PoI = ["LAF", "LAR", "UAF", "UAR", "TA", "LB", "UB", "TB"]
        for i = find(obj.Sample.(PoI)==1)
            if diff(obj.Bound.(PoI)(i,:)) == 0
                obj.Sample.(PoI)(i) = 0;
            end
        end
    end
end
