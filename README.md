# ROAD_MODEL_FUSION
Road Model Fusion for AD/ADAS functions
## 0. Introduction :-
#### We have an array of camera sensors installed in front of the ego vehicle for ADAS/AD applications. The goal of this project is to estimate estimate the Road Model which includes Ego Lane Geometry and Road Grid
![](https://github.com/UditBhaskar91/ROAD_MODEL_FUSION/blob/main/Visualization_and_Analysis/Animated_Gifs/LaneLineMeasClothoidCroppedVideo.gif)
**Fig 1 : (Inputs) Measurements : Ego Lane Boundary Camera detections in the form of Clothoid Parameters**
<br/><br/>



## 1. Estimation Output :-
### 1.1 Road Geometry (Ego Lane Center Line + Adjacent Lane Boundaries) :-
![](https://github.com/UditBhaskar91/ROAD_MODEL_FUSION/blob/main/Visualization_and_Analysis/Animated_Gifs/LaneLineFusionClothoidCroppedVideo.gif)
**Fig 1 : Ego Lane Center Line Estimation**
<br/><br/>
![](https://github.com/UditBhaskar91/ROAD_MODEL_FUSION/blob/main/Visualization_and_Analysis/Animated_Gifs/RoadModelClothoidCroppedVideo.gif)
**Fig 2 : Ego Lane Center Line Estimation + Adjacent Lane Boundaries**
<br/><br/>
### 1.2 Road Grid (Ego Lane Center Line + Adjacent Lane Boundaries + History):-
![](https://github.com/UditBhaskar91/ROAD_MODEL_FUSION/blob/main/Visualization_and_Analysis/Animated_Gifs/RoadModelGridCroppedVideo.gif)
**Fig 3 : Road Grid**
<br/><br/>




## 2. Fusion Architecture Overview :-
![](https://github.com/UditBhaskar91/OBJECT_TRACKING_MULTI_SENSOR_FUSION/blob/main/Visualization_and_Analysis/filesForReadme/0_b_fusion_archi_overview.PNG)
<br/>
## 3. High Level Fusion Architecture:-
![](https://github.com/UditBhaskar91/OBJECT_TRACKING_MULTI_SENSOR_FUSION/blob/main/Visualization_and_Analysis/filesForReadme/1a_high_level_archi.PNG)
![](https://github.com/UditBhaskar91/OBJECT_TRACKING_MULTI_SENSOR_FUSION/blob/main/Visualization_and_Analysis/filesForReadme/1b_high_level_archi.PNG)
<br/><br/>
## 4. Module Level Fusion Architecture:-
### 4.1 Preprocessing Subsystem:-
![](https://github.com/UditBhaskar91/OBJECT_TRACKING_MULTI_SENSOR_FUSION/blob/main/Visualization_and_Analysis/filesForReadme/2_a_module_archi_preprocessing.PNG)
<br/><br/>
### 4.2 Radar Fusion Subsystem :-
![](https://github.com/UditBhaskar91/OBJECT_TRACKING_MULTI_SENSOR_FUSION/blob/main/Visualization_and_Analysis/filesForReadme/3_a_module_archi_radar_fusion.PNG)
![](https://github.com/UditBhaskar91/OBJECT_TRACKING_MULTI_SENSOR_FUSION/blob/main/Visualization_and_Analysis/filesForReadme/3_b_module_archi_radar_fusion.PNG)
![](https://github.com/UditBhaskar91/OBJECT_TRACKING_MULTI_SENSOR_FUSION/blob/main/Visualization_and_Analysis/filesForReadme/3_c_module_archi_radar_fusion.PNG)
<br/><br/>
### 4.3 Camera Fusion Subsystem :-
![](https://github.com/UditBhaskar91/OBJECT_TRACKING_MULTI_SENSOR_FUSION/blob/main/Visualization_and_Analysis/filesForReadme/4_a_module_archi_camera_fusion.PNG)
![](https://github.com/UditBhaskar91/OBJECT_TRACKING_MULTI_SENSOR_FUSION/blob/main/Visualization_and_Analysis/filesForReadme/4_b_module_archi_camera_fusion.PNG)
![](https://github.com/UditBhaskar91/OBJECT_TRACKING_MULTI_SENSOR_FUSION/blob/main/Visualization_and_Analysis/filesForReadme/4_c_module_archi_camera_fusion.PNG)
<br/><br/>
### 4.4 Radar & Camera Fusion Subsystem :-
![](https://github.com/UditBhaskar91/OBJECT_TRACKING_MULTI_SENSOR_FUSION/blob/main/Visualization_and_Analysis/filesForReadme/5_a_module_archi_track_fusion.PNG)
