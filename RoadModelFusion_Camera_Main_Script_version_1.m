% ==============================================================================================================================================================
% Author Name : Udit Bhaskar Talukdar
% Date        : 01.01.2021
% Description : Main Script for Road Model Fusion
% ==============================================================================================================================================================
% Run the below scripts (in this sequence)
run('Scripts_Global_Variables_and_Sim_Data/SensorFusion_Script1_LOAD_SIMULATION_DATA.m');
run('Scripts_Global_Variables_and_Sim_Data/SensorFusion_Script2_RESTRUCTURE_SIMULATION_DATA.m');
run('Scripts_Global_Variables_and_Sim_Data/SensorFusion_Script3_LOAD_DATA_STRUCTURE_PARAMETERS.m');
run('Scripts_Global_Variables_and_Sim_Data/SensorFusion_Script5_SET_SENSOR_CONFIG_PARAMETERS.m');
addpath('Library');
% ==============================================================================================================================================================
% Simulation Parameters
dT = SamplingTime; %in sec (50 millisec)
nTimeSample = floor(SimulationTime/dT) + 1;
% =======================================>  SET MODEL PARAMETERS% <=============================================================================================
ANTICLOCKWISE_ROT = single(1);
CLOCKWISE_ROT = single(-1);
ROT_CONVENTION = ANTICLOCKWISE_ROT;
MEAS_MODEL = CLOTHOID_FUSION.ClothoidCoeffMeasModel(0.5, 0.005, 0.00005, 0.000005);   % set the observation model
epsOffset = 1; epsAlpha = (pi/180)*5; epsCurv = 10; epsCurvRate = 10;
sigmaSq_y0 = 1; sigmaSq_A0 = 0.5; sigmaSq_K0 = 0.05; sigmaSq_Khat = 0.05;
RoadGridUpperLimit = single(150); RoadGridLowerLimit = single(-150); RoadGridResolution = single(5.0);
% =======================================>  SET SENSOR INSTALLATION PARAMETERS <================================================================================                                                                                                             
CameraCALLIBRATIONparam.Intrinsic = SENSOR_LAYOUT.setCameraIntrinsicParam(nCameraTypes, ...
                                                          CAMERA_MAX_RANGE, CAMERA_MAX_AZIMUTH, CAMERA_MAX_ELEVATION, ...
                                                          CAMERA_LONG_ERR_VAR, CAMERA_LAT_ERR_VAR, ...
                                                          CAMERA_PD, CAMERA_FA, ...
                                                          CAMERA_FOV_BOUNDARY_PTS_RANGE, CAMERA_FOV_BOUNDARY_PTS_AZIMUTH, ...
                                                          CameraCALLIBRATIONparam.Intrinsic);
                                                      
CameraCALLIBRATIONparam.Extrinsic = SENSOR_LAYOUT.setSensorExtrinsicParam(nCameras, CAM_TYPE, ACTIVATE_CAM, ...
                                                          CAM_X_INSTALL, CAM_Y_INSTALL, CAM_Z_INSTALL, ...
                                                          CAM_ROLL_INSTALL, CAM_PITCH_INSTALL, CAM_YAW_INSTALL, ...
                                                          CAM_nMeas, CameraCALLIBRATIONparam.Extrinsic, ROT_CONVENTION);
% ==============================================================================================================================================================
FOVPtsEGOframeCAM = SENSOR_LAYOUT.TransformFOVtoEGOframeRadar(CameraCALLIBRATIONparam, nCameras, nCameraTypes, 100);
VISUALIZE.VisualizeCameraLayout(FOVPtsEGOframeCAM, nCameras);
ExecutionCycleTime = single(zeros(nTimeSample,1));
disp('Press any Key to Continue'); pause();
% ============================================================>   START THE ROAD MODEL FUSION <=================================================================
for t = 1:nTimeSample
    if t == 1; trigger = true;
    else; trigger = false; end
     
     
    
    
    % ----------------------------------------> GET CAMERA SENSOR DATA (LINE MEASUREMENTS) AS ARRAY OF STRUCTURE <----------------------------------------------
    CAMERA_CAN_LANE_LINE_BUS = SENSOR_INTERFACE.CAM_SENSOR_INTERFACE(CAM1_Sensor_Simulated_Data, CAM2_Sensor_Simulated_Data, CAM3_Sensor_Simulated_Data, ...
                                                                     CAMERA_CAN_LANE_LINE_BUS, t, nCameras, nLineMeas); 
    % -------------------------------------------------------> GET EGO SENSOR DATA AS A STRUCTURE <-------------------------------------------------------------
    EGO_CAN_BUS = SENSOR_INTERFACE.EGO_SENSOR_INTERFACE(EGO_Sensor_Simulated_Data, EGO_CAN_BUS, t);                      
tic
    % ==========================================================================================================================================================
    % --------------------------------------------------> LINE MEASUREMENTS ARRAY OF STRUCTURES TO ARRAY <------------------------------------------------------
    LANE_LINE_MEAS_MAT = SENSOR_INTERFACE.CREATE_LANE_LINE_MEAS_MAT(nCameras, nLineMeas, CAMERA_CAN_LANE_LINE_BUS, LANE_LINE_MEAS_MAT);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
     
    
    
    % ===================================> LINE MEASUREMENT TO LINE TRACK ASSOCIATION FOR EGO LANE BOUNDARY ESTIMATION <========================================
    % The Steps are as follows :-
    % 1. Cluster the concatenated Line Measurements (All Cameras)
    % 2. Perform state prediction of the Line Tracks from time t-1 to t
    % 3. Perform Gating of Line Measurements from each cameras with the predicted Tracks
    % 4. Perform Data association of the clothoid parameters and Clothoid Stitch for Fused Line estimation
    % 5. In case a Line Cluster is not gated with the predicted Line Track, choose the Line with the largest length and initiate a New Line Track to it
    % ==========================================================================================================================================================
    % -------------------------------------------------------------->  CLUSTER MEASUREMENT <--------------------------------------------------------------------
    LANE_LINES_CLUSTERS = GROUPING.GROUP_LANE_LINES(LANE_LINE_MEAS_MAT, LANE_LINES_CLST_MANAGE, LANE_LINES_CLUSTERS, epsOffset, epsAlpha, epsCurv, epsCurvRate); 
    % ==========================================================================================================================================================
    % ------------------------------------------------------------> PREDICTION OF LINE TRACKS <-----------------------------------------------------------------
    LANE_TRACK_BOUNDARY = CLOTHOID_FUSION.LANE_TRACK_PREDICTION(LANE_TRACK_BOUNDARY, EGO_CAN_BUS, sigmaSq_y0, sigmaSq_A0, sigmaSq_K0, sigmaSq_Khat, dT); 
    % ==========================================================================================================================================================
    % -------------------------------------------------------------> LINE MEASUREMENT GATING <------------------------------------------------------------------
    [ASSIGNMENT_MAT, GATED_LINE_INDEX] = GROUPING.GATE_LINE_MEASUREMENTS(LANE_TRACK_BOUNDARY, LANE_LINE_MEAS_MAT, epsOffset, ASSIGNMENT_MAT, MEAS_MODEL); 
    % ==========================================================================================================================================================
    % -----------------------------------------------------------------> CLOTHOID FUSION <----------------------------------------------------------------------
    LANE_TRACK_BOUNDARY = CLOTHOID_FUSION.LANE_LINE_FUSION(LANE_TRACK_BOUNDARY, ASSIGNMENT_MAT, LANE_LINE_MEAS_MAT, MEAS_MODEL);
    % ==========================================================================================================================================================
    % -------------------------------------------------------> FIND UNGATED LINES AND INITIALTE A TRACK <-------------------------------------------------------
    [UNASSOCIATED_CLUSTERS_LINE, cntLineClst] = GROUPING.FIND_NEW_LINES(LANE_LINE_MEAS_MAT.ValidCumulativeMeasCount(end), GATED_LINE_INDEX, ...
                                                                        LANE_LINES_CLUSTERS, UNASSOCIATED_CLUSTERS_LINE);  
    LANE_TRACK_BOUNDARY = CLOTHOID_FUSION.NEW_LINE_TRACK(trigger, LANE_LINE_MEAS_MAT, LANE_LINES_CLUSTERS, UNASSOCIATED_CLUSTERS_LINE, cntLineClst, LANE_TRACK_BOUNDARY);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
    
     
    % ============================================================> ESTIMATE EGO LANE WIDTH <===================================================================
    [Width, RoadWidth] = CLOTHOID_FUSION.ComputeLaneWidth(LANE_TRACK_BOUNDARY); LaneWidth = (round(10*Width))/10;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

     
     
    
    % ================> EGO LANE CENTER LINE ESTIMATION FROM EGO LANE LEFT FUSED BOUNDARY, EGO LANE LEFT FUSED BOUNDARY & ESTIMATED EGO LANE WIDTH <============
    % The Steps are as follows :-
    % 1. Multi Hypothesis generation of the ego lane center line from lane boundaries and lane width
    % 2. Ego lane center line state prediction from t-1 to t
    % 3. Kalman filter state update of the predicted ego lane center line track with ech of the hypothesised ego lane center lines from step 1
    % 4. Multi-hypothesis Track to Track data fusion of the updated state estimates from step 3 (multi hypothesis merge)
    % ==========================================================================================================================================================
    % ---------------------------------------------------------> HYPOTHESISE EGO LANE CENTER <------------------------------------------------------------------
    EGO_LANE_CENTER_TRACK_HYPOTHESIS = ROAD_MODEL.EGO_LANE_HYPOTHESIS(EGO_LANE_CENTER_TRACK_HYPOTHESIS, LANE_TRACK_BOUNDARY, LaneWidth/2);
    % ==========================================================================================================================================================
    % ----------------------------------------------------------> PREDICT EGO LANE CENTER <---------------------------------------------------------------------
    ROAD_GEOMETRY.EGO_LANE_CENTER_TRACK_ESTIMATION = ROAD_MODEL.EGO_LANE_PREDICTION(ROAD_GEOMETRY.EGO_LANE_CENTER_TRACK_ESTIMATION, EGO_CAN_BUS, ...
                                                                                    sigmaSq_y0, sigmaSq_A0, sigmaSq_K0, sigmaSq_Khat, dT);
    % ==========================================================================================================================================================
    % --------------------------------------------------------> KALMAN FILTER STATE UPDATE <--------------------------------------------------------------------                                                                            
     EGO_LANE_CENTER_TRACK_HYPOTHESIS = ROAD_MODEL.EGO_LANE_UPDATE(EGO_LANE_CENTER_TRACK_HYPOTHESIS, ROAD_GEOMETRY.EGO_LANE_CENTER_TRACK_ESTIMATION, MEAS_MODEL);
    % ==========================================================================================================================================================
    % ----------------------------------------------------------> MULTI HYPOTHESIS MERGE <----------------------------------------------------------------------     
    ROAD_GEOMETRY.EGO_LANE_CENTER_TRACK_ESTIMATION = ROAD_MODEL.EGO_LANE_TRACK_TO_TRACK_FUSION(ROAD_GEOMETRY.EGO_LANE_CENTER_TRACK_ESTIMATION, EGO_LANE_CENTER_TRACK_HYPOTHESIS);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                                                                                            
     
    
    
    % ============================================================> ESTIMATE ROAD MODEL <=======================================================================
    % The Road Model is represented as follows :-
    % 1. Estimated Clothoid parameters of the following :-
    %    Ego Lane Center Line
    %    All the valid Lane Boundaries
    % 2. Road Grid : A grid representation of all the available lanes
    % ==========================================================================================================================================================
    % -----------------------------------------------------> ROAD MODEL CLOTHOID COEFFICIENTS <-----------------------------------------------------------------   
    ROAD_GEOMETRY.EGO_LANE_NEXT_NEXT_RIGHT_BOUNDARY = ROAD_MODEL.LANE_GEOMETRY(ROAD_GEOMETRY.EGO_LANE_NEXT_NEXT_RIGHT_BOUNDARY, ...
                                                                               ROAD_GEOMETRY.EGO_LANE_CENTER_TRACK_ESTIMATION,  (2*LaneWidth + LaneWidth/2));
    ROAD_GEOMETRY.EGO_LANE_NEXT_RIGHT_BOUNDARY      = ROAD_MODEL.LANE_GEOMETRY(ROAD_GEOMETRY.EGO_LANE_NEXT_RIGHT_BOUNDARY, ...
                                                                               ROAD_GEOMETRY.EGO_LANE_CENTER_TRACK_ESTIMATION,  (1*LaneWidth + LaneWidth/2));
    ROAD_GEOMETRY.EGO_LANE_RIGHT_BOUNDARY           = ROAD_MODEL.LANE_GEOMETRY(ROAD_GEOMETRY.EGO_LANE_RIGHT_BOUNDARY, ...
                                                                               ROAD_GEOMETRY.EGO_LANE_CENTER_TRACK_ESTIMATION,   LaneWidth/2);                                                  
    ROAD_GEOMETRY.EGO_LANE_LEFT_BOUNDARY            = ROAD_MODEL.LANE_GEOMETRY(ROAD_GEOMETRY.EGO_LANE_LEFT_BOUNDARY, ...
                                                                               ROAD_GEOMETRY.EGO_LANE_CENTER_TRACK_ESTIMATION,  -LaneWidth/2);
    ROAD_GEOMETRY.EGO_LANE_NEXT_LEFT_BOUNDARY       = ROAD_MODEL.LANE_GEOMETRY(ROAD_GEOMETRY.EGO_LANE_NEXT_LEFT_BOUNDARY, ...
                                                                               ROAD_GEOMETRY.EGO_LANE_CENTER_TRACK_ESTIMATION,  -(1*LaneWidth + LaneWidth/2));
    ROAD_GEOMETRY.EGO_LANE_NEXT_NEXT_LEFT_BOUNDARY  = ROAD_MODEL.LANE_GEOMETRY(ROAD_GEOMETRY.EGO_LANE_NEXT_NEXT_LEFT_BOUNDARY, ...
                                                                               ROAD_GEOMETRY.EGO_LANE_CENTER_TRACK_ESTIMATION,  -(2*LaneWidth + LaneWidth/2));                                                                     
    % ==========================================================================================================================================================
    % -------------------------------------------> ROAD MODEL GRID (FROM CURRENT ESTIMATIONS AND HISTORY) <-----------------------------------------------------                                                              
    ROAD_GRID.EgoLaneCenter              = ROAD_MODEL.LANE_LINE_MODEL(ROAD_GRID.EgoLaneCenter, ROAD_GEOMETRY.EGO_LANE_CENTER_TRACK_ESTIMATION, EGO_CAN_BUS, ...
                                                                      RoadGridResolution, RoadGridUpperLimit, RoadGridLowerLimit, dT);
    ROAD_GRID.EgoLaneRightBoundary       = ROAD_MODEL.LANE_LINE_MODEL(ROAD_GRID.EgoLaneRightBoundary, ROAD_GEOMETRY.EGO_LANE_RIGHT_BOUNDARY, EGO_CAN_BUS, ...
                                                                      RoadGridResolution, RoadGridUpperLimit, RoadGridLowerLimit, dT);
    ROAD_GRID.EgoLaneNxtRightBoundary    = ROAD_MODEL.LANE_LINE_MODEL(ROAD_GRID.EgoLaneNxtRightBoundary, ROAD_GEOMETRY.EGO_LANE_NEXT_RIGHT_BOUNDARY, EGO_CAN_BUS, ...
                                                                      RoadGridResolution, RoadGridUpperLimit, RoadGridLowerLimit, dT);
    ROAD_GRID.EgoLaneNxtNxtRightBoundary = ROAD_MODEL.LANE_LINE_MODEL(ROAD_GRID.EgoLaneNxtNxtRightBoundary, ROAD_GEOMETRY.EGO_LANE_NEXT_NEXT_RIGHT_BOUNDARY, EGO_CAN_BUS, ...
                                                                      RoadGridResolution, RoadGridUpperLimit, RoadGridLowerLimit, dT);
    ROAD_GRID.EgoLaneLeftBoundary        = ROAD_MODEL.LANE_LINE_MODEL(ROAD_GRID.EgoLaneLeftBoundary, ROAD_GEOMETRY.EGO_LANE_LEFT_BOUNDARY, EGO_CAN_BUS, ...
                                                                      RoadGridResolution, RoadGridUpperLimit, RoadGridLowerLimit, dT);
    ROAD_GRID.EgoLaneNxtLeftBoundary     = ROAD_MODEL.LANE_LINE_MODEL(ROAD_GRID.EgoLaneNxtLeftBoundary, ROAD_GEOMETRY.EGO_LANE_NEXT_LEFT_BOUNDARY, EGO_CAN_BUS, ...
                                                                      RoadGridResolution, RoadGridUpperLimit, RoadGridLowerLimit, dT);
    ROAD_GRID.EgoLaneNxtNxtLeftBoundary  = ROAD_MODEL.LANE_LINE_MODEL(ROAD_GRID.EgoLaneNxtNxtLeftBoundary, ROAD_GEOMETRY.EGO_LANE_NEXT_NEXT_LEFT_BOUNDARY, EGO_CAN_BUS, ...
                                                                      RoadGridResolution, RoadGridUpperLimit, RoadGridLowerLimit, dT);      
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
    
    
    
    

    
    
%     figure(1); XLimit = [-10 150]; YLimit = [-60 60]; markerA = '.';markerB = '-';
%     VISUALIZE.displayMeas(LANE_LINE_MEAS_MAT, XLimit, YLimit);hold on;
%     VISUALIZE.displayLane(LANE_TRACK_BOUNDARY.LaneTrackParam(1).ClothoidParam, 'k', markerB, XLimit, YLimit);hold on;
%     VISUALIZE.displayLane(LANE_TRACK_BOUNDARY.LaneTrackParam(2).ClothoidParam, 'k', markerB, XLimit, YLimit);hold on;
%     VISUALIZE.displayLane(ROAD_GEOMETRY.EGO_LANE_CENTER_TRACK_ESTIMATION, 'k', markerB, XLimit, YLimit);hold on;
%     VISUALIZE.displayLane(ROAD_GEOMETRY.EGO_LANE_NEXT_NEXT_RIGHT_BOUNDARY, 'b', markerB, XLimit, YLimit);hold on;
%     VISUALIZE.displayLane(ROAD_GEOMETRY.EGO_LANE_NEXT_RIGHT_BOUNDARY, 'b', markerB, XLimit, YLimit);hold on;
%     VISUALIZE.displayLane(ROAD_GEOMETRY.EGO_LANE_RIGHT_BOUNDARY, 'b', markerB, XLimit, YLimit);hold on;
%     VISUALIZE.displayLane(ROAD_GEOMETRY.EGO_LANE_LEFT_BOUNDARY, 'b', markerB, XLimit, YLimit);hold on;
%     VISUALIZE.displayLane(ROAD_GEOMETRY.EGO_LANE_NEXT_LEFT_BOUNDARY, 'b', markerB, XLimit, YLimit);hold on;
%     VISUALIZE.displayLane(ROAD_GEOMETRY.EGO_LANE_NEXT_NEXT_LEFT_BOUNDARY, 'b', markerB, XLimit, YLimit);hold on;  
%     hold off;
     
     
    figure(1); markerA = '.'; markerB = '-'; XLimit = [-200, 200]; YLimit = [-70, 30];  
    %XLimit = [-520, 520]; YLimit = [-520, 20];
    VISUALIZE.displayRoad(ROAD_GRID.EgoLaneCenter, 'b', markerA, XLimit, YLimit);hold on;
    VISUALIZE.displayRoad(ROAD_GRID.EgoLaneRightBoundary, 'r', markerB, XLimit, YLimit);hold on;
    VISUALIZE.displayRoad(ROAD_GRID.EgoLaneLeftBoundary, 'g', markerB, XLimit, YLimit);hold on;
    VISUALIZE.displayRoad(ROAD_GRID.EgoLaneNxtRightBoundary, 'k', markerB, XLimit, YLimit);hold on;
    VISUALIZE.displayRoad(ROAD_GRID.EgoLaneNxtLeftBoundary, 'm', markerB, XLimit, YLimit);hold on;
    VISUALIZE.displayRoad(ROAD_GRID.EgoLaneNxtNxtRightBoundary, 'b', markerB, XLimit, YLimit);hold on;
    VISUALIZE.displayRoad(ROAD_GRID.EgoLaneNxtNxtLeftBoundary, 'r', markerB, XLimit, YLimit);hold off;
    
     
    ExecutionCycleTime(t) = toc; 
    disp(strcat('Time Elapsed : ', num2str(0.05*t))); 
end






