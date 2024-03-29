% Define an empty scenario.
scenario = drivingScenario;
scenario.SampleTime = 0.01;

% define parameter
situation_num = 1;
wait_end = 0;
scenes = {scene1, scene2, scene3, scene4, scene5, scene6, scene7, scene8, scene9, scene10};
collision_mats = [];
max_S_vals = [];
global collision_mat;
collision_mat = zeros(10);
global max_S_val;
max_S_val = zeros(10);

for scene_n = 8:10
    for cur_speed = 7:2:25
        for cur_distance = 15:5:60
            fprintf('Situation %d: vehicle speed: %d m/s, distance: %d m\n', situation_num, cur_speed, cur_distance);
            displaySceneAndVehicle(situation_num, cur_speed, cur_distance, wait_end, scenes{1, scene_n});
            situation_num = situation_num + 1;
        end
    end

    filename = "./results/scene" + scene_n + ".csv";
    writematrix(collision_mat, filename);
    filename = "./results/scene" + scene_n + "_maxS_val.csv";
    writematrix(max_S_val, filename);
    collision_mats = cat(3, collision_mats, collision_mat);
    max_S_vals = cat(3, max_S_vals, max_S_val);
    collision_mat = zeros(10);
    max_S_val = zeros(10);
end

collision_mats


function displaySceneAndVehicle(situation_num, vehicle_speed, distance, wait_end, scene)
    % Create the drivingScenario object and ego car
    [scenario, egoVehicle, speed, waypoints] = scene.create_scene(vehicle_speed, distance, wait_end);
    
    % Create all the sensors
    [sensors, numSensors] = createSensors(scenario);
    global collision_mat;
    global max_S_val;
    
    % Register actor profiles with the sensors.
    profiles = actorProfiles(scenario);
    for m = 1:numel(sensors)
        if isa(sensors{m},'drivingRadarDataGenerator')
            sensors{m}.Profiles = profiles;
        else
            sensors{m}.ActorProfiles = profiles;
        end
    end

    tracker = multiObjectTracker('FilterInitializationFcn', @initSimDemoFilter, ...
        'AssignmentThreshold', 30, 'ConfirmationThreshold', [4 5]);
    positionSelector = [1 0 0 0; 0 0 1 0]; % Position selector
    velocitySelector = [0 1 0 0; 0 0 0 1]; % Velocity selector

    % Create the display and return a handle to the bird's-eye plot
    BEP = createDemoDisplay(egoVehicle, sensors, situation_num, vehicle_speed, distance);
    toSnap = true;

    count = 0;
    collision = 0;

    velocity_list = [];
    position_list = [];
    yaw_list = [];

    positions = [];
    velocities = [];



    t_end = 50;

    % control vehicle
    while advance(scenario) && ishghandle(BEP.Parent) && collision == 0
        % Get the scenario time
        time = scenario.SimulationTime;
    
        % Get the position of the other vehicle in ego vehicle coordinates
        ta = targetPoses(egoVehicle);
    
        % Simulate the sensors
        detectionClusters = {};
        isValidTime = false(1,8);
        for i = 1:2
            [sensorDets,numValidDets,isValidTime(i)] = sensors{i}(ta, time);
            if numValidDets
                for j = 1:numValidDets
                    % Vision detections do not report SNR. The tracker requires
                    % that they have the same object attributes as the radar
                    % detections. This adds the SNR object attribute to vision
                    % detections and sets it to a NaN.
                    if ~isfield(sensorDets{j}.ObjectAttributes{1}, 'SNR')
                        sensorDets{j}.ObjectAttributes{1}.SNR = NaN;
                    end
    
                    % Remove the Z-component of measured position and velocity
                    % from the Measurement and MeasurementNoise fields
                    sensorDets{j}.Measurement = sensorDets{j}.Measurement([1 2 4 5]);
                    sensorDets{j}.MeasurementNoise = sensorDets{j}.MeasurementNoise([1 2 4 5],[1 2 4 5]);
                end
                detectionClusters = [detectionClusters; sensorDets]; %#ok<AGROW>
            end
        end

        velocity_list = [velocity_list; egoVehicle.Velocity];
        position_list = [position_list; egoVehicle.Position];
        yaw_list = [yaw_list; egoVehicle.Yaw];

        cur_positions = [];
        cur_velocities = [];
        for i = 1:length(ta)
            cur_positions = [cur_positions; ta(i).Position];
            cur_velocities = [cur_velocities; ta(i).Velocity];
        end
        positions = cat(3, positions, cur_positions);
        velocities = cat(3, velocities, cur_velocities);
        
        if count > 2 && egoVehicle.Position(2) < 10 && egoVehicle.Position(2) > -10
            S = BehavioralDecisionMaking(1500, velocity_list, position_list, min(t_end, count), velocities, positions, yaw_list);
            if S > max_S_val((vehicle_speed-5)/2, (distance-10)/5)
                max_S_val((vehicle_speed-5)/2, (distance-10)/5) = S;
            end
            if S > 4500000
                collision = 1;
                collision_mat((vehicle_speed-5)/2, (distance-10)/5) = 1;
            end
        end
        count = count + 1;

        
        if egoVehicle.Position(2) < 10 && egoVehicle.Position(2) > -10
            for i = 1 : height(cur_positions)
                tmp = cur_positions;
                dis = sqrt(tmp(i, 1) * tmp(i, 1) + tmp(i, 2) * tmp(i, 2));
                if dis < 3
                    collision = 1;
                    collision_mat((vehicle_speed-5)/2, (distance-10)/5) = 2;
                    break
                end
            end
        end


        
        % Update the tracker if there are new detections
        if any(isValidTime)
            if isa(sensors{1},'drivingRadarDataGenerator')
                vehicleLength = sensors{1}.Profiles.Length;
            else
                vehicleLength = sensors{1}.ActorProfiles.Length;
            end
            confirmedTracks = updateTracks(tracker, detectionClusters, time);
    
            % Update bird's-eye plot
            updateBEP(BEP, egoVehicle, detectionClusters, confirmedTracks, positionSelector, velocitySelector);
        end
        % Snap a figure for the document when the car passes the ego vehicle
        if ta(1).Position(1) > 0 && toSnap
            toSnap = false;
            snapnow
        end
    end
end

function filter = initSimDemoFilter(detection)
% Use a 2-D constant velocity model to initialize a trackingKF filter.
% The state vector is [x;vx;y;vy]
% The detection measurement vector is [x;y;vx;vy]
% As a result, the measurement model is H = [1 0 0 0; 0 0 1 0; 0 1 0 0; 0 0 0 1]
H = [1 0 0 0; 0 0 1 0; 0 1 0 0; 0 0 0 1];
filter = trackingKF('MotionModel', '2D Constant Velocity', ...
    'State', H' * detection.Measurement, ...
    'MeasurementModel', H, ...
    'StateCovariance', H' * detection.MeasurementNoise * H, ...
    'MeasurementNoise', detection.MeasurementNoise);
end

function BEP = createDemoDisplay(egoCar, sensors, situation_num, vehicle_speed, distance)
    % Make/clear a figure
    hFigure = clf('reset');
    set(gcf,'Position',[0, 0, 1200, 640], 'Name', ...  % [0, 0, 1200, 640]
        sprintf(['Vehicle Collision Test (Situation %d: ' ...
        'vehicle speed: %d m/s, distance: %d m)'], ...
        situation_num, vehicle_speed, distance)) 
    % hFigure.WindowState = 'maximized';
    % Make a figure
    % hFigure = figure('Position', [0, 0, 1200, 640], 'Name', 'Sensor Fusion with Synthetic Data Example');
    movegui(hFigure, [0 -1]); % Moves the figure to the left and a little down from the top

    % Add a car plot that follows the ego vehicle from behind
    hCarViewPanel = uipanel(hFigure, 'Position', [0 0 0.5 0.5], 'Title', 'Chase Camera View');
    hCarPlot = axes(hCarViewPanel);
    chasePlot(egoCar, 'Parent', hCarPlot);

    % Add a car plot that follows the ego vehicle from a top view
    hTopViewPanel = uipanel(hFigure, 'Position', [0 0.5 0.5 0.5], 'Title', 'Top View');
    hCarPlot = axes(hTopViewPanel);
    chasePlot(egoCar, 'Parent', hCarPlot, 'ViewHeight', 130, 'ViewLocation', [0 0], 'ViewPitch', 90);

    % Add a panel for a bird's-eye plot
    hBEVPanel = uipanel(hFigure, 'Position', [0.5 0 0.5 1], 'Title', 'Bird''s-Eye Plot');

    % Create bird's-eye plot for the ego vehicle and sensor coverage
    hBEVPlot = axes(hBEVPanel);
    frontBackLim = 60;
    BEP = birdsEyePlot('Parent', hBEVPlot, 'Xlimits', [-frontBackLim frontBackLim], 'Ylimits', [-35 35]);

    % Plot the coverage areas for radars
    for i = 1:1
        cap = coverageAreaPlotter(BEP,'FaceColor','red','EdgeColor','red');
        if isa(sensors{i},'drivingRadarDataGenerator')
            plotCoverageArea(cap, sensors{i}.MountingLocation(1:2),...
                sensors{i}.RangeLimits(2), sensors{i}.MountingAngles(1), sensors{i}.FieldOfView(1));
        else
            plotCoverageArea(cap, sensors{i}.SensorLocation,...
                sensors{i}.MaxRange, sensors{i}.Yaw, sensors{i}.FieldOfView(1));
        end
    end

    % Plot the coverage areas for vision sensors
    for i = 2:2
        cap = coverageAreaPlotter(BEP,'FaceColor','blue','EdgeColor','blue');
        if isa(sensors{i},'drivingRadarDataGenerator')
            plotCoverageArea(cap, sensors{i}.MountingLocation(1:2),...
                sensors{i}.RangeLimits(2), sensors{i}.MountingAngles(1), 45);
        else
            plotCoverageArea(cap, sensors{i}.SensorLocation,...
                sensors{i}.MaxRange, sensors{i}.Yaw, 45);
        end
    end

    % Create a vision detection plotter put it in a struct for future use
    detectionPlotter(BEP, 'DisplayName','vision', 'MarkerEdgeColor','blue', 'Marker','^');

    % Combine all radar detections into one entry and store it for later update
    detectionPlotter(BEP, 'DisplayName','radar', 'MarkerEdgeColor','red');

    % Add road borders to plot
    laneMarkingPlotter(BEP, 'DisplayName','lane markings');

    % Add the tracks to the bird's-eye plot. Show last 10 track updates.
    trackPlotter(BEP, 'DisplayName','track', 'HistoryDepth',10);

    axis(BEP.Parent, 'equal');
    xlim(BEP.Parent, [-frontBackLim frontBackLim]);
    ylim(BEP.Parent, [-40 40]);

    % Add an outline plotter for ground truth
    outlinePlotter(BEP, 'Tag', 'Ground truth');
end

function updateBEP(BEP, egoCar, detections, confirmedTracks, psel, vsel)
    % Update road boundaries and their display
    [lmv, lmf] = laneMarkingVertices(egoCar);
    plotLaneMarking(findPlotter(BEP,'DisplayName','lane markings'),lmv,lmf);

    % update ground truth data
    [position, yaw, length, width, originOffset, color] = targetOutlines(egoCar);
    plotOutline(findPlotter(BEP,'Tag','Ground truth'), position, yaw, length, width, 'OriginOffset', originOffset, 'Color', color);

    % update barrier data
    [bPosition,bYaw,bLength,bWidth,bOriginOffset,bColor,numBarrierSegments] = targetOutlines(egoCar, 'Barriers');
    plotBarrierOutline(findPlotter(BEP,'Tag','Ground truth'),numBarrierSegments,bPosition,bYaw,bLength,bWidth,...
                       'OriginOffset',bOriginOffset,'Color',bColor);

    % Prepare and update detections display
    N = numel(detections);
    detPos = zeros(N,2);
    isRadar = true(N,1);
    for i = 1:N
        detPos(i,:) = detections{i}.Measurement(1:2)';
        if detections{i}.SensorIndex > 6 % Vision detections
            isRadar(i) = false;
        end
    end
    plotDetection(findPlotter(BEP,'DisplayName','vision'), detPos(~isRadar,:));
    plotDetection(findPlotter(BEP,'DisplayName','radar'), detPos(isRadar,:));

    % Remove all object tracks that are unidentified by the vision detection
    % generators before updating the tracks display. These have the ObjectClassID
    % parameter value as 0 and include objects such as barriers.
    isNotBarrier = arrayfun(@(t)t.ObjectClassID,confirmedTracks)>0;
    confirmedTracks = confirmedTracks(isNotBarrier);

    % Prepare and update tracks display
    trackIDs = {confirmedTracks.TrackID};
    labels = cellfun(@num2str, trackIDs, 'UniformOutput', false);
    [tracksPos, tracksCov] = getTrackPositions(confirmedTracks, psel);
    tracksVel = getTrackVelocities(confirmedTracks, vsel);
    plotTrack(findPlotter(BEP,'DisplayName','track'), tracksPos, tracksVel, tracksCov, labels);
end

function [sensors, numSensors] = createSensors(scenario)
% createSensors Returns all sensor objects to generate detections

% Assign into each sensor the physical and radar profiles for all actors
profiles = actorProfiles(scenario);
sensors{1} = drivingRadarDataGenerator('SensorIndex', 1, ...
    'MountingLocation', [1.9 0 0.2], ...
    'TargetReportFormat', 'Detections', ...
    'FieldOfView', [75 5], ...
    'Profiles', profiles);
sensors{2} = visionDetectionGenerator('SensorIndex', 2, ...
    'SensorLocation', [3.7 0], ...
    'MaxRange', 100, ...
    'DetectorOutput', 'Objects only', ...
    'Intrinsics', cameraIntrinsics([1814.81018227767 1814.81018227767],[320 240],[480 640]), ...
    'ActorProfiles', profiles);
numSensors = 2;
end

