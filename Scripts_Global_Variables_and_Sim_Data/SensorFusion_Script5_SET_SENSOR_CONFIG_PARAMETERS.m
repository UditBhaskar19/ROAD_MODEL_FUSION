% ==============================================================================================================================================================
% Author Name : Udit Bhaskar Talukdar
% Date        : 01.01.2021
% Description : Sensor Callibration Parameters Specifications   
% ==============================================================================================================================================================
% CAMARA Long Range : Intrinsic Parameters (Field Of View and Maximum Range)
CAMERA_TYPE_FAR_RANGE = uint8(1);
CAM_FAR_Max_Range = single(140);
CAM_FAR_Max_Azimuth = single(18);
CAM_FAR_Max_Elevation = single(-1); % not used 
CAM_FAR_LongErrVar = single(-1); % not used 
CAM_FAR_LatErrVar = single(-1); % not used 
CAM_FAR_ProbOfDetection = single(0.99); % not used 
CAM_FAR_FalseAlarmRate = single(2/45000); % not used 
CAM_FAR_FOVRangePoints = single(zeros(1, 20)); % not used 
CAM_FAR_FOVAzimuthPts = single(zeros(1, 20)); % not used 
% ==============================================================================================================================================================
% CAMARA Mid Range : Intrinsic Parameters (Field Of View and Maximum Range)
CAMERA_TYPE_MID_RANGE = uint8(2);
CAM_MID_Max_Range = single(100);
CAM_MID_Max_Azimuth = single(33);
CAM_MID_Max_Elevation = single(-1); % not used 
CAM_MID_LongErrVar = single(-1); % not used 
CAM_MID_LatErrVar = single(-1); % not used 
CAM_MID_ProbOfDetection = single(0.99); % not used 
CAM_MID_FalseAlarmRate = single(2/45000); % not used 
CAM_MID_FOVRangePoints = single(zeros(1, 20)); % not used 
CAM_MID_FOVAzimuthPts = single(zeros(1, 20)); % not used 
% ==============================================================================================================================================================
% CAMARA Short Range : Intrinsic Parameters (Field Of View and Maximum Range)
CAMERA_TYPE_SHORT_RANGE = uint8(3);
CAM_SHORT_Max_Range = single(70);
CAM_SHORT_Max_Azimuth = single(58);
CAM_SHORT_Max_Elevation = single(-1); % not used 
CAM_SHORT_LongErrVar = single(-1); % not used 
CAM_SHORT_LatErrVar = single(-1); % not used 
CAM_SHORT_ProbOfDetection = single(0.99); % not used 
CAM_SHORT_FalseAlarmRate = single(2/45000); % not used 
CAM_SHORT_FOVRangePoints = single(zeros(1, 20)); % not used 
CAM_SHORT_FOVAzimuthPts = single(zeros(1, 20)); % not used 
% ==============================================================================================================================================================
% CAMARA Intrinsic Parameters (Field Of View and Maximum Range)
CAMERA_MAX_RANGE = [CAM_FAR_Max_Range; CAM_MID_Max_Range; CAM_SHORT_Max_Range];
CAMERA_MAX_AZIMUTH = [CAM_FAR_Max_Azimuth; CAM_MID_Max_Azimuth; CAM_SHORT_Max_Azimuth];
CAMERA_MAX_ELEVATION = [CAM_FAR_Max_Elevation; CAM_MID_Max_Elevation; CAM_SHORT_Max_Elevation];
CAMERA_LONG_ERR_VAR = [CAM_FAR_LongErrVar; CAM_MID_LongErrVar; CAM_SHORT_LongErrVar];
CAMERA_LAT_ERR_VAR = [CAM_FAR_LatErrVar; CAM_MID_LatErrVar; CAM_SHORT_LatErrVar];
CAMERA_PD = [CAM_FAR_ProbOfDetection; CAM_MID_ProbOfDetection; CAM_SHORT_ProbOfDetection];
CAMERA_FA = [CAM_FAR_FalseAlarmRate; CAM_MID_FalseAlarmRate; CAM_SHORT_FalseAlarmRate];
CAMERA_FOV_BOUNDARY_PTS_RANGE = [CAM_FAR_FOVRangePoints; CAM_MID_FOVRangePoints; CAM_SHORT_FOVRangePoints];
CAMERA_FOV_BOUNDARY_PTS_AZIMUTH = [CAM_FAR_FOVAzimuthPts; CAM_MID_FOVAzimuthPts; CAM_SHORT_FOVAzimuthPts];                                                                  
% ==============================================================================================================================================================
% Sensor Extrinsic Parameter (X, Y, Z , Yaw mounting parameters) w.r.t ego vehicle center (in Clock Wise Direction)
% Camera installs in clockwise direction (Check the order once a data is generated)
CAM_X_INSTALL   = single([3.7; 3.7; 3.7]);
CAM_Y_INSTALL   = single([0;   0;  0]);
CAM_Z_INSTALL   = single([0;   0;  0]); % not set
CAM_YAW_INSTALL = single([0;   0;  0]);
CAM_ROLL_INSTALL = single([0;  0;  0]); % not set
CAM_PITCH_INSTALL = single([0; 0;  0]); % not set
CAM_nMeas = uint16([2; 2; 2]);
CAM_TYPE = uint8([CAMERA_TYPE_FAR_RANGE, CAMERA_TYPE_MID_RANGE, CAMERA_TYPE_SHORT_RANGE]);
% ==============================================================================================================================================================
% Sensor Layout(SL) module parameters (This module is one time computation which gets actived using a "trigger")
% Sensor Activation Flags for 3 Cameras (1:Sensor Active, 0:Sensor Inactive)
ACTIVATE_CAM = boolean([1; 1; 1]);