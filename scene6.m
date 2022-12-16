classdef scene6
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
            % normal left vehicle
            left_car = vehicle(scenario, ...
                'ClassID', 1, ...
                'Position', [24 distance 0.01], ...
                'Mesh', driving.scenario.carMesh, ...
                'Name', 'Left Car');
            waypoints=[27 distance 0.01;
                27.0 11.0 0.01;
                27.0 7.0 0.01;
                27.5 6.0 0.01;
                29.5 6.0 0.01;
                30.5 6.0 0.01;
                32.0 7.0 0.01;
                32.0 8.0 0.01;
                32.0 11.0 0.01;
                32.0 30.00 0.01];
            speed = [vehicle_speed;vehicle_speed;vehicle_speed;
                vehicle_speed;vehicle_speed;vehicle_speed;
                vehicle_speed;vehicle_speed;vehicle_speed;0];
            waittime = [0;0;0;0;0;0;0;0;0;wait_end];
            trajectory(left_car, waypoints, speed, waittime);
            
            % Add the actors
            % normal right vehicle
            right_car = vehicle(scenario, ...
                'ClassID', 1, ...
                'Position', [24 -distance 0.01], ...
                'Mesh', driving.scenario.carMesh, ...
                'Name', 'Right car');
            waypoints = [32 -distance 0.01;
                31.5 -2.5 0.01;
                30.0 -1.0 0.01;
                27.5 0.0 0.01;
                25.0 1.0 0.01;
                23.0 1.5 0.01;
                20.0 2.0 0.01;
                5.0 2.0 0.01];
            speed = [vehicle_speed;vehicle_speed;vehicle_speed;
                vehicle_speed;
                vehicle_speed;vehicle_speed;vehicle_speed;0];
            waittime = [0;0;0;0;0;0;0;wait_end];
            trajectory(right_car, waypoints, speed, waittime);
            
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
            speed = [0;vehicle_speed;vehicle_speed;
                vehicle_speed;vehicle_speed;0];
            waittime = [1000;0;0;0;0;wait_end];
            trajectory(egoVehicle, waypoints, speed, waittime);
        end
    end
end
