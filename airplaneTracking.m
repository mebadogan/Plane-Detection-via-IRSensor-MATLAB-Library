clear all;clc;close all;
scene = trackingScenario('UpdateRate',1,'StopTime',120);
mesh = tracking.scenario.airplaneMesh;
sensor = irSensor(1);
sensor.MountingLocation = [150 0 50];
sensor.MountingAngles = [-120 -210 0];
sensor.MaxNumDetectionsSource = 'Auto';
sensor.UpdateRate = 100;
sensor.HasElevation = false;
sensor.FocalLength = 400;
% sensor.NoiseEquivalentBandwidth = 300e+10; % To add noise, uncomment this line
airplane1 = platform(scene);
airplane1.Mesh = mesh;
airplane1.Dimensions = struct('Length',50,'Width',50,'Height',25,'OriginOffset',[0 0 0]);
airplane2 = platform(scene);
airplane2.Mesh = mesh;
airplane2.Dimensions = struct('Length',50,'Width',50,'Height',25,'OriginOffset',[0 0 0]);
wpts1 = [0 100 50; 100 0 50; 0 -100 50; -100 0 50; 0 100 50];
wpts2 = [0 50 50; 50 0 50; 0 -50 50; -50 0 50; 0 50 50];
time1 = [0; 25; 50; 75; 100];
time2 = [0; 25; 50; 75; 100];
airplane1.Trajectory = waypointTrajectory(wpts1, time1);
airplane1.Sensors = sensor;
airplane2.Trajectory = waypointTrajectory(wpts2, time2);
airplane2.Sensors = sensor;

figure;
grid;
axis equal;
axis([-300 300 -300 300]);
line1 = animatedline('DisplayName','Trajectory 1','Color','b','LineWidth',3);
line2 = animatedline('DisplayName','Trajectory 2','Color','r','LineWidth',3);
title('Tracking Scenario');
hold on
plot(wpts1(:,1),wpts1(:,2),' ob')
plot((sensor.MountingLocation(1)),(sensor.MountingLocation(2)),'o-','MarkerFaceColor','green','MarkerEdgeColor','black');
text((sensor.MountingLocation(1)),(sensor.MountingLocation(2)),"IRSensor",'HorizontalAlignment','center','VerticalAlignment','top')
text(wpts1(:,1),wpts1(:,2),"t = " + string(time1),'HorizontalAlignment','left','VerticalAlignment','bottom')
plot(wpts2(:,1),wpts2(:,2),' ob')
text(wpts2(:,1),wpts2(:,2),"t = " + string(time2),'HorizontalAlignment','left','VerticalAlignment','bottom')
numberOfDetection = 0;
lp = scatter3(nan,nan,nan,nan,nan);
tp = theaterPlot('Parent',lp.Parent,'XLimits',[-150 150],'YLimits',[-150 150],'ZLimits',[0 100]);
pp = platformPlotter(tp);
legend("off");

while advance(scene)
    view(-45,25);
    simTime = scene.SimulationTime;
    p1 = pose(airplane1);
    p2 = pose(airplane2);
    addpoints(line1,p1.Position(1),p1.Position(2));
    addpoints(line2,p2.Position(1),p2.Position(2));
    position1 = p1.Position;
    velocity1 = p1.Velocity;
    position2 = p2.Position;
    velocity2 = p2.Velocity;
    target1 = struct('PlatformID',1,'Position',position1,'Speed',sqrt((velocity1(1))^(2)+velocity1(2)^(2)));
    target2 = struct('PlatformID',2,'Position',position2,'Speed',sqrt((velocity2(1))^(2)+velocity2(2)^(2)));
    [dets1,numDets1,config1] = sensor(target1,simTime);
    [dets2,numDets2,config2] = sensor(target2,simTime);
    if numDets1 == 1 && numDets2 ==1
        disp("Target 1");disp(" ");
        disp(p1);disp(" ");
        disp("Target 2");disp(" ");
        disp(p2);disp(" ");
        plot((p1.Position(1)),(p1.Position(2)),'o-','MarkerFaceColor','yellow','MarkerEdgeColor','yellow');
        plot((p2.Position(1)),(p2.Position(2)),'o-','MarkerFaceColor','yellow','MarkerEdgeColor','yellow');
        numberOfDetection = numberOfDetection + 2;
        platPoses = platformPoses(scene);
        pos = vertcat(platPoses.Position);
        mesh = cellfun(@(x)x.Mesh,scene.Platforms);
        orient = vertcat(platPoses.Orientation);
        pp.plotPlatform(pos,mesh,orient);
    elseif numDets1 == 1 && numDets2 == 0
        disp("Target 1");disp(" ");
        disp(p1);disp(" ");disp(" ");
        numberOfDetection = numberOfDetection + 1;
        plot((p1.Position(1)),(p1.Position(2)),'o-','MarkerFaceColor','black','MarkerEdgeColor','black');
    elseif numDets2 == 1 && numDets1 == 0
        disp("Target 2");disp(" ");
        disp(p2);disp(" ");
        numberOfDetection = numberOfDetection + 1;
        plot((p2.Position(1)),(p2.Position(2)),'o-','MarkerFaceColor','cyan','MarkerEdgeColor','cyan');
    end
    pause(0.001);
end
disp("Number Of Detections:");
disp(numberOfDetection);
disp("Number Of Points:");
numberOfPoints = scene.UpdateRate *scene.StopTime*2;
disp(numberOfPoints);
disp("Efficiency:")
efficiency = (numberOfDetection/numberOfPoints)*100;
disp(efficiency);
