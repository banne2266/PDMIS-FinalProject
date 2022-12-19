classdef scene2
    methods
        function [scenario, egoVehicle, speed, waypoints] = create_scene(create_scene, vehicle_speed, distance, wait_end)
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
            % normal vehicle
            car = vehicle(scenario, ...
                'ClassID', 1, ...
                'Position', [24 -distance 0.01], ...
                'Mesh', driving.scenario.carMesh, ...
                'Name', 'Car');
            waypoints = [32 -distance 0.01;
                32 25 0.01;
                32 30 0.01];
            speed = [vehicle_speed;vehicle_speed;0];
            waittime = [0;0;wait_end];
            trajectory(car, waypoints, speed, waittime);
            
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
                32.0 20.0 0.01];
            speed = [1;vehicle_speed;vehicle_speed;
                vehicle_speed;vehicle_speed;0];
            waittime = [0;0;0;0;0;wait_end];
            trajectory(egoVehicle, waypoints, speed, waittime);
        end
    end
end
