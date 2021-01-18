% ==============================================================================================================================================================
% Author Name : Udit Bhaskar Talukdar
% Date        : 01.01.2021
% Description : This script loads and rearranges the simulated sensor data The simulated data is generated using matlab
% ==============================================================================================================================================================
clearvars; clc; close all;
simData = load('..\Scenario_Simulation_Data\SCENARIO_6\LaneDATA_cam1.mat');
simData = simData.LaneDATA_cam1;
simDataCam1 = load('..\Scenario_Simulation_Data\SCENARIO_6\LaneDATA_cam1.mat');
simDataCam1 = simDataCam1.LaneDATA_cam1;
simDataCam2 = load('..\Scenario_Simulation_Data\SCENARIO_6\LaneDATA_cam2.mat');
simDataCam2 = simDataCam2.LaneDATA_cam2;
simDataCam3 = load('..\Scenario_Simulation_Data\SCENARIO_6\LaneDATA_cam3.mat');
simDataCam3 = simDataCam3.LaneDATA_cam3;
nTimeSamples = length(simData);
dT = 0.05; % in sec
SimulationTime = simData(end).Time;
% simData is an array of structure of size (1 x nTimeSamples) having the below member variables:
% 1. Time
% 2. ActorPoses
% 3. ObjectDetections.
% 4. LaneDetections.
TimeVector = single(zeros(nTimeSamples,1));
for idx = 1:nTimeSamples
    TimeVector(idx,1) = simData(idx).Time;
end
% ==============================================================================================================================================================
% Lane Detection Data 
% An Array of idx1->curvature, idx2->curvRate, idx3->curvLen, idx4->alpha, idx5->latOffset, idx6->xMin, idx7->xMax, idx8->width
nLaneLines = 2; nLaneDimension = 8; nLaneSensors = 3;
Lane_Line_Measurements.Time = single(zeros(nTimeSamples, 1));
Lane_Line_Measurements.SensorID = int16(zeros(nTimeSamples, 1));
Lane_Line_Measurements.Curvature = single(zeros(nTimeSamples, nLaneLines));
Lane_Line_Measurements.CurvRate = single(zeros(nTimeSamples, nLaneLines));
Lane_Line_Measurements.CurveLength = single(zeros(nTimeSamples, nLaneLines));
Lane_Line_Measurements.HeadingAngle = single(zeros(nTimeSamples, nLaneLines));
Lane_Line_Measurements.LateralOffset = single(zeros(nTimeSamples, nLaneLines));
Lane_Line_Measurements.XMin = single(zeros(nTimeSamples, nLaneLines));
Lane_Line_Measurements.XMax = single(zeros(nTimeSamples, nLaneLines));
Lane_Line_Measurements.Width = single(zeros(nTimeSamples, nLaneLines));
Lane_Line_Measurements = Lane_Line_Measurements(ones(nLaneSensors, 1));
for timeIdx = 1:nTimeSamples
    for snsrIdx = 1:nLaneSensors
	    if(snsrIdx == 1); simDataCam_i = simDataCam1;
		elseif(snsrIdx == 2); simDataCam_i = simDataCam2;
		elseif(snsrIdx == 3); simDataCam_i = simDataCam3;end
		Lane_Line_Measurements(snsrIdx).Time(timeIdx,1) = simDataCam_i(timeIdx).LaneDetections(1).Time;
		Lane_Line_Measurements(snsrIdx).SensorID(timeIdx, 1) = simDataCam_i(timeIdx).LaneDetections(1).SensorIndex;
		for lineIdx = 1: nLaneLines
		    Lane_Line_Measurements(snsrIdx).Curvature(timeIdx, lineIdx) = simDataCam_i(timeIdx).LaneDetections(1).LaneBoundaries(lineIdx).Curvature;
            Lane_Line_Measurements(snsrIdx).CurvRate(timeIdx, lineIdx) = simDataCam_i(timeIdx).LaneDetections(1).LaneBoundaries(lineIdx).CurvatureDerivative;
            Lane_Line_Measurements(snsrIdx).CurveLength(timeIdx, lineIdx) = simDataCam_i(timeIdx).LaneDetections(1).LaneBoundaries(lineIdx).CurveLength;
            Lane_Line_Measurements(snsrIdx).HeadingAngle(timeIdx, lineIdx) = simDataCam_i(timeIdx).LaneDetections(1).LaneBoundaries(lineIdx).HeadingAngle;
            Lane_Line_Measurements(snsrIdx).LateralOffset(timeIdx, lineIdx) = simDataCam_i(timeIdx).LaneDetections(1).LaneBoundaries(lineIdx).LateralOffset;
            Lane_Line_Measurements(snsrIdx).XMin(timeIdx, lineIdx) = simDataCam_i(timeIdx).LaneDetections(1).LaneBoundaries(lineIdx).XExtent(1,1);
            Lane_Line_Measurements(snsrIdx).XMax(timeIdx, lineIdx) = simDataCam_i(timeIdx).LaneDetections(1).LaneBoundaries(lineIdx).XExtent(1,2);
            Lane_Line_Measurements(snsrIdx).Width(timeIdx, lineIdx) = simDataCam_i(timeIdx).LaneDetections(1).LaneBoundaries(lineIdx).Width;
		end
    end
end
LaneSimulation.Data = Lane_Line_Measurements;
LaneSimulation.Time = TimeVector;
% ==============================================================================================================================================================
% Ground Truth and Ego Sensor Data
nActors = length(simData(1).ActorPoses);
nDim = 13; % no. of attributes in the groundtruth;
% id, px, py, pz, vx, vy, vz, roll, pitch, yaw, angular_vel_x, angular_vel_y, angular_vel_z
GroundTruthArray = single(zeros(nTimeSamples, nDim));
GroundTruth = struct;
GriundTruth.ActorID = int16(0);
GroundTruth.Data = GroundTruthArray;
GroundTruth = GroundTruth(ones(nActors, 1));
for idx = 1:nTimeSamples
    poses = simData(idx).ActorPoses;
    for idxActor = 1:nActors
        actorData = simData(idx).ActorPoses(idxActor);
        GroundTruth(idxActor).ActorID = actorData.ActorID;
        GroundTruth(idxActor).Data(idx,1) = simData(idx).ActorPoses(idxActor).Position(1,1);
        GroundTruth(idxActor).Data(idx,2) = simData(idx).ActorPoses(idxActor).Position(1,2);
        GroundTruth(idxActor).Data(idx,3) = simData(idx).ActorPoses(idxActor).Position(1,3);
        GroundTruth(idxActor).Data(idx,4) = simData(idx).ActorPoses(idxActor).Velocity(1,1);
        GroundTruth(idxActor).Data(idx,5) = simData(idx).ActorPoses(idxActor).Velocity(1,2);
        GroundTruth(idxActor).Data(idx,6) = simData(idx).ActorPoses(idxActor).Velocity(1,3);
        GroundTruth(idxActor).Data(idx,7) = simData(idx).ActorPoses(idxActor).Roll;
        GroundTruth(idxActor).Data(idx,8) = simData(idx).ActorPoses(idxActor).Pitch;
        GroundTruth(idxActor).Data(idx,9) = simData(idx).ActorPoses(idxActor).Yaw;
        GroundTruth(idxActor).Data(idx,10) = simData(idx).ActorPoses(idxActor).AngularVelocity(1,1);
        GroundTruth(idxActor).Data(idx,11) = simData(idx).ActorPoses(idxActor).AngularVelocity(1,2);
        GroundTruth(idxActor).Data(idx,12) = simData(idx).ActorPoses(idxActor).AngularVelocity(1,3);
    end
end
% perform sanity check by plotting the Ground Truth
figure(1);
startX = GroundTruth(1).Data(1, 1); startY = GroundTruth(1).Data(1, 2);
endX = GroundTruth(1).Data(nTimeSamples, 1); endY = GroundTruth(1).Data(nTimeSamples, 2);
plot(startX ,startY,'-s','MarkerSize',10,'MarkerEdgeColor','red','MarkerFaceColor',[1 .6 .6]);hold on;
plot(endX ,endY,'-*','MarkerSize',10,'MarkerEdgeColor','blue','MarkerFaceColor',[1 .6 .6]);hold on;
X0 = GroundTruth(1).Data(:, 1); Y0 = GroundTruth(1).Data(:, 2);
plot(X0,Y0,'b');hold on;hold off;axis equal;grid on;
legend('Initial Position of the EV at the start of the simulation',...
       'Final Position of the EV at the end of the simulation',...
       'EV Ground Truth Trajectory',...
       'Location','NorthWest');
disp('Press any Key to Continue'); pause();
% ==============================================================================================================================================================
% Ego Sensor Measurements
EgoSensorMeasurements.px = GroundTruth(1).Data(:, 1);
EgoSensorMeasurements.py = GroundTruth(1).Data(:, 2);
EgoSensorMeasurements.vx = GroundTruth(1).Data(:, 4);
EgoSensorMeasurements.vy = GroundTruth(1).Data(:, 5);
EgoSensorMeasurements.yaw = GroundTruth(1).Data(:, 9);
EgoSensorMeasurements.yawRate = GroundTruth(1).Data(:, 12);
EgoParameters.Data = EgoSensorMeasurements;
EgoParameters.Time = TimeVector;
% ==============================================================================================================================================================
% Save the Data
SimulationScenario.TotalTime = SimulationTime;
SimulationScenario.SampleTime = dT;
SimulationScenario.EgoSensorMeasurements = EgoParameters;
SimulationScenario.LaneSensorMeasurements = LaneSimulation;
save('..\Scenario_Simulation_Data\SCENE_1.mat', 'SimulationScenario');
clearvars; %clear all;
load('..\Scenario_Simulation_Data\SCENE_1.mat');
% ==============================================================================================================================================================
% Simulation Time
SamplingTime = SimulationScenario.SampleTime;
SimulationTime = SimulationScenario.TotalTime;
% ==============================================================================================================================================================
% Lane Measurements CAMERA 1
LINE_CURVATURE_CAM1 = SimulationScenario.LaneSensorMeasurements.Data(1).Curvature;
LINE_CURVDERIVATIVE_CAM1 = SimulationScenario.LaneSensorMeasurements.Data(1).CurvRate;
LINE_CURVELENGTH_CAM1 = SimulationScenario.LaneSensorMeasurements.Data(1).CurveLength;
LINE_HEADINGANGLE_CAM1 = SimulationScenario.LaneSensorMeasurements.Data(1).HeadingAngle;
LINE_LATERALOFFSET_CAM1 = SimulationScenario.LaneSensorMeasurements.Data(1).LateralOffset;
LINE_XMIN_CAM1 = SimulationScenario.LaneSensorMeasurements.Data(1).XMin;
LINE_XMAX_CAM1 = SimulationScenario.LaneSensorMeasurements.Data(1).XMax;
LINE_WIDTH_CAM1 = SimulationScenario.LaneSensorMeasurements.Data(1).Width;
LINE_detTimeStamp_CAM1 = SimulationScenario.LaneSensorMeasurements.Data(1).Time;
% ==============================================================================================================================================================
% Lane Measurements CAMERA 2
LINE_CURVATURE_CAM2 = SimulationScenario.LaneSensorMeasurements.Data(2).Curvature;
LINE_CURVDERIVATIVE_CAM2 = SimulationScenario.LaneSensorMeasurements.Data(2).CurvRate;
LINE_CURVELENGTH_CAM2 = SimulationScenario.LaneSensorMeasurements.Data(2).CurveLength;
LINE_HEADINGANGLE_CAM2 = SimulationScenario.LaneSensorMeasurements.Data(2).HeadingAngle;
LINE_LATERALOFFSET_CAM2 = SimulationScenario.LaneSensorMeasurements.Data(2).LateralOffset;
LINE_XMIN_CAM2 = SimulationScenario.LaneSensorMeasurements.Data(2).XMin;
LINE_XMAX_CAM2 = SimulationScenario.LaneSensorMeasurements.Data(2).XMax;
LINE_WIDTH_CAM2 = SimulationScenario.LaneSensorMeasurements.Data(2).Width;
LINE_detTimeStamp_CAM2 = SimulationScenario.LaneSensorMeasurements.Data(2).Time;
% ==============================================================================================================================================================
% Lane Measurements CAMERA 3
LINE_CURVATURE_CAM3 = SimulationScenario.LaneSensorMeasurements.Data(3).Curvature;
LINE_CURVDERIVATIVE_CAM3 = SimulationScenario.LaneSensorMeasurements.Data(3).CurvRate;
LINE_CURVELENGTH_CAM3 = SimulationScenario.LaneSensorMeasurements.Data(3).CurveLength;
LINE_HEADINGANGLE_CAM3 = SimulationScenario.LaneSensorMeasurements.Data(3).HeadingAngle;
LINE_LATERALOFFSET_CAM3 = SimulationScenario.LaneSensorMeasurements.Data(3).LateralOffset;
LINE_XMIN_CAM3 = SimulationScenario.LaneSensorMeasurements.Data(3).XMin;
LINE_XMAX_CAM3 = SimulationScenario.LaneSensorMeasurements.Data(3).XMax;
LINE_WIDTH_CAM3 = SimulationScenario.LaneSensorMeasurements.Data(3).Width;
LINE_detTimeStamp_CAM3 = SimulationScenario.LaneSensorMeasurements.Data(3).Time;
% ==============================================================================================================================================================
% EGO VEHICLE Measurements
EGO_PX = SimulationScenario.EgoSensorMeasurements.Data.px;
EGO_PY = SimulationScenario.EgoSensorMeasurements.Data.py;
EGO_VX = SimulationScenario.EgoSensorMeasurements.Data.vx;
EGO_VY = SimulationScenario.EgoSensorMeasurements.Data.vy;
EGO_YAW = SimulationScenario.EgoSensorMeasurements.Data.yaw;
EGO_YAWRATE = SimulationScenario.EgoSensorMeasurements.Data.yawRate;
EGO_TimeStamp = SimulationScenario.EgoSensorMeasurements.Time;
% ==========================================================================> END OF SCRIPT <===================================================================
