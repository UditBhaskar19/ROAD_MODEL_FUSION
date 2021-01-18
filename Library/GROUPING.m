% --------------------------------------------------------------------------------------------------------------------------------------------------------------
% Author Name : Udit Bhaskar Talukdar
% Date        : 01/01/2021
% Description : Measurement to Track and Track to Track Gating function library, contains the following functions
%             : GATE_LINE_MEASUREMENTS()             :  Gating of Sensor measurements , create association matrix for track and sensor measurements
%             : GatingAndLogLikelihood()             :  Perform Gating and Compute the likelihood of the predicted measurement in logarithmic scale
%             : GROUP_LANE_LINES()                   :  clusters Lane Boundaries (Lines) using Nearest Neighbour algorithm
%             : FIND_NEW_LINES()                     :  extract the measurement indexes from the gated measurement clusters
%             : FIND_UNGATED_CLUSTERS()              :  Find the list of unassociated Lane Clusters
% --------------------------------------------------------------------------------------------------------------------------------------------------------------
classdef GROUPING
    methods(Static)
        % ======================================================================================================================================================
        function [ASSIGNMENT_MAT, GATED_LINE_INDEX] = GATE_LINE_MEASUREMENTS(LANE_TRACK_BOUNDARY, LANE_LINE_MEAS_MAT, GammaSq, ASSIGNMENT_MAT, MEAS_MODEL)
            % Gating of Sensor measurements , create association matrix for track and sensor measurements 
            % INPUTS : LANE_TRACK_BOUNDARY    : Predicted Lane Boundary Line Track
            %        : LANE_LINE_MEAS_MAT     : Lane Line Measurements
            %        : MEAS_MODEL             : Measurement model 
            %        : GammaSq                : Gating Threshold
            %        : ASSIGNMENT_MAT         : (INITIALIZED) Association matrix (nTracks , nTracks + nMeas)
            % OUTPUT : ASSIGNMENT_MAT     : (UPDATED) Association matrix (nTracks , nTracks + nMeas)
            %          GATED_MEAS_INDEX   : boolean flag array indicating gated measurement indexes
            % --------------------------------------------------------------------------------------------------------------------------------------------------
            nTracks = LANE_TRACK_BOUNDARY.nValidTracks;                   % Total number of Predicted Line Tracks
            nLineMeas = LANE_LINE_MEAS_MAT.ValidCumulativeMeasCount(end); % Total number of Line Measurements from camera  
            GATED_LINE_INDEX = false(1,nLineMeas); % initialize the gated measurements index indicator to false (indicates that none of the mesurements are gated initially)
            for idx = 1:nTracks    % initialize the assignment matrix to zeros
                ASSIGNMENT_MAT(1,idx).LogLikelihoodMat(:) = single(0);
                ASSIGNMENT_MAT(1,idx).isGatedMat(:) = false;
                ASSIGNMENT_MAT(1,idx).nMeas = single(0);
            end
            if(nTracks == 0 || nLineMeas == 0)  % do not perform gating if no objects or no measurements are present
               return;
            end
            for xIdx = 1:nTracks        % for each of the Lane Line Track extract the state data and perform gating
                yOffset = LANE_TRACK_BOUNDARY.LaneTrackParam(1,xIdx).ClothoidParam.LateralOffset;
                X.x = [LANE_TRACK_BOUNDARY.LaneTrackParam(1,xIdx).ClothoidParam.LateralOffset;
                       LANE_TRACK_BOUNDARY.LaneTrackParam(1,xIdx).ClothoidParam.HeadingAngle; ...
                       LANE_TRACK_BOUNDARY.LaneTrackParam(1,xIdx).ClothoidParam.Curvature; ...
                       LANE_TRACK_BOUNDARY.LaneTrackParam(1,xIdx).ClothoidParam.CurvatureDerivative]; 
                X.P =  LANE_TRACK_BOUNDARY.LaneTrackParam(1,idx).ClothoidParam.ErrCOV;
                for zIdx = 1:nLineMeas  % for each of the Lane Line Measurements
                    z = LANE_LINE_MEAS_MAT.MeasArray(:,zIdx);
                    [LogLikelihood, isGated] = GROUPING.GatingAndLogLikelihood(z, MEAS_MODEL, X, yOffset, GammaSq);
                    ASSIGNMENT_MAT(1,xIdx).LogLikelihoodMat(1,zIdx) = LogLikelihood;
                    ASSIGNMENT_MAT(1,xIdx).isGatedMat(1,zIdx) = isGated;
                    if(isGated)
                       GATED_LINE_INDEX(1,zIdx) = isGated;
                       ASSIGNMENT_MAT(1,xIdx).nMeas = ASSIGNMENT_MAT(1,xIdx).nMeas + 1;
                    end
                end
            end
        end
        % ======================================================================================================================================================
        function [LogLikelihood, isGated] = GatingAndLogLikelihood(Z, measmodel, state_pred, yOffset, GammaSq)
            % Perform Gating and Compute the likelihood of the predicted measurement in logarithmic scale
            % INPUT :       z : measurement , struct with 2 fields, x: meas vector, R: meas noise covariance
            %        measmodel: a structure specifies the measurement model parameters  
            %       state_pred: a structure with two fields:
            %                x: predicted object state mean (state dimension) x 1 vector 
            %                P: predicted object state covariance (state dimension) x (state dimension) matrix
            %         GammaSq : Square of Gamma for gating
            % OUTPUT : LogLikelihood : log likelihood of the predicted state (valid only if the the state is gated with a measurement)
            %        :       isGated : boolean flag indicating if the measurement is gated
            % --------------------------------------------------------------------------------------------------------------------------------------------------
            INVALID = single(99);
            yk = Z(1:4,1) - measmodel.h(state_pred.x);
            H = measmodel.H(state_pred.x);   % Measurement model Jacobian
            S = H * state_pred.P * H' + measmodel.R; % Innovation covariance
            S = (S + S')/2;                  % Make sure matrix S is positive definite
            mDist = yk' * (S\yk);            % mahalanobis distance  
            d = abs(Z(1,1) - yOffset);
            if (d <= GammaSq)
                LogLikelihood =  - 0.5*log(det(2*pi*S)) - 0.5*mDist;
                isGated = true;
            else
                LogLikelihood = -INVALID;
                isGated = false;
            end
        end
        % ======================================================================================================================================================
        function LANE_LINES_CLUSTERS = GROUP_LANE_LINES(LANE_LINE_MEAS_MAT, MANAGE_CLUSTER, LANE_LINES_CLUSTERS, ...
                                                        epsOffset, epsAlpha, epsCurv, epsCurvRate)
            % clusters Lane Boundaries (Lines) using Nearest Neighbour algorithm
            % INPUT:  LANE_LINE_MEAS_MAT : array of lane line measurements from camera
            %         MANAGE_CLUSTERS    : A structure of arrays for maintaining information for clustering, contains the following fields
            %         clusterMember      :
            %         clusterMemberFlag  :
            %         measurementVisited :
            %         LANE_LINES_CLUSTERS : initialized structure of line clusters and related information
            %         epsOffset, epsAlpha, epsCurv, epsCurvRate  : clustering eps threshold for position and velocity
            % OUTPUT: LANE_LINES_CLUSTERS  : structure of line clusters
            % --------------------------------------------------------------------------------------------------------------------------------------------------
            LANE_LINES_CLUSTERS.nClusters = uint16(0);
            LANE_LINES_CLUSTERS.ClusterSizes(:) = uint16(0);
            LANE_LINES_CLUSTERS.ClusterIDs(:) = uint16(0);
            LANE_LINES_CLUSTERS.ClustIDAssig(:) = uint16(0);
            sizeClust = uint16(0); ClusterID = uint16(0);
            latOffset_idx = 1; alpha_idx = 2;  curv_idx = 3; curvRate_idx = 4; % Constant used as arrayIndex
            nMeas = LANE_LINE_MEAS_MAT.ValidCumulativeMeasCount(end); % number of valid measurements
            for i = 1:nMeas 
                if(~MANAGE_CLUSTER.measurementVisited(i))        % check if the measurement has been added to an existing cluster/expanded
                    MANAGE_CLUSTER.measurementVisited(i) = true; % form a new cluster and 'tag' it as added
                    ClusterID = ClusterID + 1;    % create a new cluster ID
                    sizeClust = sizeClust + 1;    % update the current cluster size
                    MANAGE_CLUSTER.clusterMember(sizeClust) = i; % add the measurement 'i' to the list (used for computing the cluster center later)
                    LANE_LINES_CLUSTERS.nClusters = LANE_LINES_CLUSTERS.nClusters + 1;  % update the total number of clusters
                    LANE_LINES_CLUSTERS.ClusterIDs(1,LANE_LINES_CLUSTERS.nClusters) = ClusterID; % set the cluster ID
                    LANE_LINES_CLUSTERS.ClustIDAssig(i) = ClusterID; % tag the measurement 'i' with its cluster ID
                    for j = i+1:nMeas % compare meas 'i' with other unclustered measurement 'j's to identify if 'j' forms a cluster with 'i'
                        if(~MANAGE_CLUSTER.measurementVisited(j)) % compare only if 'j' is not a part of existing cluster
                            offsetDiff = norm([LANE_LINE_MEAS_MAT.MeasArray(latOffset_idx,i) - LANE_LINE_MEAS_MAT.MeasArray(latOffset_idx,j), ...
                                               LANE_LINE_MEAS_MAT.MeasArray(latOffset_idx,i) - LANE_LINE_MEAS_MAT.MeasArray(latOffset_idx,j)]);
                            alphaDiff = norm([LANE_LINE_MEAS_MAT.MeasArray(alpha_idx,i) - LANE_LINE_MEAS_MAT.MeasArray(alpha_idx,j), ...
                                              LANE_LINE_MEAS_MAT.MeasArray(alpha_idx,i) - LANE_LINE_MEAS_MAT.MeasArray(alpha_idx,j)]);
                            curvDiff = norm([LANE_LINE_MEAS_MAT.MeasArray(curv_idx,i) - LANE_LINE_MEAS_MAT.MeasArray(curv_idx,j), ...
                                             LANE_LINE_MEAS_MAT.MeasArray(curv_idx,i) - LANE_LINE_MEAS_MAT.MeasArray(curv_idx,j)]);
                            curvRateDiff = norm([LANE_LINE_MEAS_MAT.MeasArray(curvRate_idx,i) - LANE_LINE_MEAS_MAT.MeasArray(curvRate_idx,j), ...
                                                 LANE_LINE_MEAS_MAT.MeasArray(curvRate_idx,i) - LANE_LINE_MEAS_MAT.MeasArray(curvRate_idx,j)]);
                            if(offsetDiff <= epsOffset ) %&& alphaDiff <= epsAlpha && curvDiff <= epsCurv && curvRateDiff <= epsCurvRate)
                               MANAGE_CLUSTER.measurementVisited(j) = true; % tag the measurement 'j' indicating that it is a part of a cluster
                               LANE_LINES_CLUSTERS.ClustIDAssig(j) = ClusterID; % tag the measurement 'j' with its cluster ID
                               sizeClust = sizeClust + 1;    % update the current cluster size
                               MANAGE_CLUSTER.clusterMember(sizeClust) = j; % add the measurement 'j' to the list (used for computing the cluster center later)
                            end %end of check   
                        end
                    end
                    LANE_LINES_CLUSTERS.ClusterSizes(1, LANE_LINES_CLUSTERS.nClusters) = sizeClust; % cluster size
                end
                MANAGE_CLUSTER.clusterMember(:) = uint16(0); % reset the Cluster member list to zero
                sizeClust = uint16(0); % reset the size cluster list tozero
            end
        end
        % ======================================================================================================================================================
        function [UNASSOCIATED_CLUSTERS, cntMeasClst] = FIND_NEW_LINES(nSnsrMeas, GATED_MEAS_INDEX, CLUSTERS_MEAS, UNASSOCIATED_CLUSTERS)
            % Find the list of unassociated Lane Clusters
            % INPUTS : nSnsrMeas             : number of sensor measurements
            %          GATED_MEAS_INDEX      : gated measurememnt index array
            %          CLUSTERS_MEAS         : Measurement clusters from each sensors
            %          UNASSOCIATED_CLUSTERS : Unassociated clusters initialized data structure
            % OUTPUTS: UNASSOCIATED_CLUSTERS : Unassociated clusters updated data structure
            %          cntMeasClst           : number of ungated measurement clusters
            % --------------------------------------------------------------------------------------------------------------------------------------------------
            UNASSOCIATED_CLUSTERS(:) = uint16(0);
            cntMeasClst = uint16(0);
            if(nSnsrMeas ~= 0)
               UNGATED_MEAS_INDEX_LIST = find(GATED_MEAS_INDEX(1,1:nSnsrMeas) == 0); % list of ungated measurement index
               nUngatedMeas = length(UNGATED_MEAS_INDEX_LIST);                       % number of ungated meas
               isMeasVisited = false(1,nSnsrMeas);                        
               for idx = 1:nUngatedMeas
                   ungatedMeasIdx = UNGATED_MEAS_INDEX_LIST(1,idx);
                   if(~isMeasVisited(1,ungatedMeasIdx))
                       clusterID = CLUSTERS_MEAS.ClustIDAssig(1,ungatedMeasIdx);     % Cluster ID
                       cntMeasClst = cntMeasClst + 1;
                       UNASSOCIATED_CLUSTERS(1,cntMeasClst) = clusterID;
                       MeasList = (CLUSTERS_MEAS.ClustIDAssig == clusterID);         % find the measurement index with the same cluster ID
                       isMeasVisited(1,MeasList) = true;
                   end
               end
            end
        end
        % ======================================================================================================================================================
    end
end
      









