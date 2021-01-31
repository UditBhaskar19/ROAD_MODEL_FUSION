# ROAD_MODEL_FUSION
Road Model Fusion for AD/ADAS functions
## 0. Introduction :-
#### We have an array of camera sensors installed in front of the ego vehicle for ADAS/AD applications. The goal of this project is to estimate the Road Model which includes Ego Lane Geometry and Road Grid
![](https://github.com/UditBhaskar19/ROAD_MODEL_FUSION/blob/main/Visualization_and_Analysis/Animated_Gifs/LaneLineMeasClothoidCroppedVideo.gif)
**Fig 1 : (Inputs) Measurements: Ego Lane Boundary Camera detections in the form of Clothoid Parameters**
<br/><br/>



## 1. Estimation Output :-
### 1.1 Road Model (Ego Lane Boundary Line Fusion + Road Geometry + Road Grid) :-
![](https://github.com/UditBhaskar19/ROAD_MODEL_FUSION/blob/main/Visualization_and_Analysis/Animated_Gifs/LaneLineFusionClothoidCroppedVideo.gif)
![](https://github.com/UditBhaskar19/ROAD_MODEL_FUSION/blob/main/Visualization_and_Analysis/Animated_Gifs/RoadModelClothoidCroppedVideo.gif)
![](https://github.com/UditBhaskar19/ROAD_MODEL_FUSION/blob/main/Visualization_and_Analysis/Animated_Gifs/RoadModelGridCroppedVideo.gif)
<br/><br/>



## 2. High Level Design:-
### 2.1 Line Measurements To Line Track Association for Ego Lane Boundary Estimation :-
![](https://github.com/UditBhaskar19/ROAD_MODEL_FUSION/blob/main/Visualization_and_Analysis/filesForReadme/1a_Road_Fusion_hld.PNG)
### 2.2 Ego Lane Center Line Estimation :-
![](https://github.com/UditBhaskar19/ROAD_MODEL_FUSION/blob/main/Visualization_and_Analysis/filesForReadme/1b_Road_Fusion_hld.PNG)
### 2.3 Road Model Computation :-
![](https://github.com/UditBhaskar19/ROAD_MODEL_FUSION/blob/main/Visualization_and_Analysis/filesForReadme/1c_Road_Fusion_hld.PNG)



## 3. High Level Fusion Architecture:-
![](https://github.com/UditBhaskar19/ROAD_MODEL_FUSION/blob/main/Visualization_and_Analysis/filesForReadme/2a_Road_Fusion_Architecture.PNG)
## 4. Road Grid Parameterization:-
![](https://github.com/UditBhaskar19/ROAD_MODEL_FUSION/blob/main/Visualization_and_Analysis/filesForReadme/2b_Road_Grid.PNG)
