% Define an empty scenario.
scenario = drivingScenario;
scenario.SampleTime = 0.01;

% define parameter
count = 0;
situation_num = 1;
sensor_vehicle_speed = 10;
normal_vehicle_speed = 10;
wait_end = 5;

for svs = 10:5:60
    for nvs = 10:5:60
        fprintf('Situation %d: sensor_vehicle_speed: %d m/s, normal_vehicle_speed: %d m/s\n', situation_num, svs, nvs);
        displaySceneAndVehicle(situation_num, count, svs, nvs, wait_end);
        situation_num = situation_num + 1;
    end
end


function displaySceneAndVehicle(situation_num, count, sensor_vehicle_speed, normal_vehicle_speed, wait_end)
    % Create the drivingScenario object and ego car
    [scenario, egoVehicle, speed, waypoints] = createDrivingScenario(sensor_vehicle_speed, normal_vehicle_speed, wait_end);
    
    % Create all the sensors
    [sensors, numSensors] = createSensors(scenario);
    
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
    BEP = createDemoDisplay(egoVehicle, sensors, situation_num, sensor_vehicle_speed, normal_vehicle_speed);
    
    toSnap = true;

    % control vehicle
    while advance(scenario) && ishghandle(BEP.Parent)
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
        MAX_CNT = 25;
        
        if count < MAX_CNT
            speed = [0;sensor_vehicle_speed;sensor_vehicle_speed;
                sensor_vehicle_speed;sensor_vehicle_speed;
                sensor_vehicle_speed;0];
            waittime = [100;0;0;0;0;0;wait_end];
            trajectory(egoVehicle, waypoints, speed, waittime);
            count = count + 1;
            if count == MAX_CNT
                speed = [0;sensor_vehicle_speed;sensor_vehicle_speed;
                    sensor_vehicle_speed;sensor_vehicle_speed;
                    sensor_vehicle_speed;0];
                waittime = [MAX_CNT / 100;0;0;0;0;0;wait_end];
                trajectory(egoVehicle, waypoints, speed, waittime);
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

function BEP = createDemoDisplay(egoCar, sensors, situation_num, sensor_vehicle_speed, normal_vehicle_speed)
    % Make/clear a figure
    hFigure = clf('reset');
    set(gcf,'Position',[0, 0, 1200, 640], 'Name', ...  % [0, 0, 1200, 640]
        sprintf(['Vehicle Collision Test (Situation %d: ' ...
        'sensor_vehicle_speed: %d m/s, normal_vehicle_speed: %d m/s)'], ...
        situation_num, sensor_vehicle_speed, normal_vehicle_speed)) 
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
    % chasePlot

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

function [scenario, egoVehicle, speed, waypoints] = createDrivingScenario(sensor_vehicle_speed, normal_vehicle_speed, wait_end)
% createDrivingScenario Returns the drivingScenario defined in the Designer

% Construct a drivingScenario object.
scenario = drivingScenario;

% Add all road segments
roadCenters = [30 35 0;
    30 -35 0];
marking = [laneMarking('Solid', 'Color', [0.98 0.86 0.36])
    laneMarking('Dashed')
    laneMarking('DoubleSolid', 'Color', [1 1 0])
    laneMarking('Dashed')
    laneMarking('Solid')];
laneSpecification = lanespec(4, 'Width', 4, 'Marking', marking);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road');

roadCenters = [30 0 0;
    0 0 0];
marking = [laneMarking('Solid', 'Color', [0.98 0.86 0.36])
    laneMarking('Dashed')
    laneMarking('DoubleSolid', 'Color', [1 1 0])
    laneMarking('Dashed')
    laneMarking('Solid')];
laneSpecification = lanespec(4, 'Width', 4, 'Marking', marking);
road(scenario, roadCenters, 'Lanes', laneSpecification, 'Name', 'Road1');

% Add the actors
% normal left vehicle
left_car = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [24 30 0.01], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Left Car');
waypoints=[27 30 0.01;
    27.0 11.0 0.01;
    27.0 7.0 0.01;
    27.5 6.0 0.01;
    29.5 6.0 0.01;
    30.5 6.0 0.01;
    32.0 7.0 0.01;
    32.0 8.0 0.01;
    32.0 11.0 0.01;
    32.0 30.00 0.01];
speed = [normal_vehicle_speed;normal_vehicle_speed;normal_vehicle_speed;
    normal_vehicle_speed;normal_vehicle_speed;normal_vehicle_speed;
    normal_vehicle_speed;normal_vehicle_speed;normal_vehicle_speed;0];
waittime = [0;0;0;0;0;0;0;0;0;wait_end];
trajectory(left_car, waypoints, speed, waittime);

% % Add the actors
% % normal right vehicle
% right_car = vehicle(scenario, ...
%     'ClassID', 1, ...
%     'Position', [24 30 0.01], ...
%     'Mesh', driving.scenario.carMesh, ...
%     'Name', 'Right car');
% % waypoints = [24 30 0.01;
% %     25 -30 0.01];
% % speed = [60;60];
% % trajectory(car, waypoints, speed);
% % waypoints = [32 -30 0.01;
% %     32 25 0.01;
% %     32 30 0.01];
% waypoints = [32 -30 0.01;
%     32.0 -11.0 0.01;
%     32.0 -2.5 0.01;
%     30.0 -1.0 0.01;
%     27.5 0.0 0.01;
%     23.0 1.0 0.01;
%     20.0 2.0 0.01;
%     15.0 2.0 0.01];
% speed = [normal_vehicle_speed;normal_vehicle_speed;normal_vehicle_speed;
%     normal_vehicle_speed;normal_vehicle_speed;normal_vehicle_speed;
%     normal_vehicle_speed;0];
% waittime = [0;0;0;0;0;0;0;wait_end];
% trajectory(right_car, waypoints, speed, waittime);

% Add the ego vehicle
% sensor vehicle
egoVehicle = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [17 -2 0.01], ...
    'Mesh', driving.scenario.carMesh, ...
    'Name', 'Car1');
waypoints = [17 -2 0.01;
    23.0 -1.75 0.01;
    27.5 -1.5 0.01;
    30.0 -1 0.01;
    32.0 2.5 0.01;
    32.0 11.0 0.01;
    32.0 30.00 0.01];
speed = [sensor_vehicle_speed;sensor_vehicle_speed;sensor_vehicle_speed;
    sensor_vehicle_speed;sensor_vehicle_speed;sensor_vehicle_speed;0];
trajectory(egoVehicle, waypoints, speed);
end
