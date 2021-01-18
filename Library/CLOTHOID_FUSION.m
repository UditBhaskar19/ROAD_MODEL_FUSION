% --------------------------------------------------------------------------------------------------------------------------------------------------------------
% Author Name : Udit Bhaskar Talukdar
% Date        : 01/01/2021
% Description : Line Fusion Library function, for fused line geometry estimations :
%             : normalizeLogWeights()             : Normalize the weights in log scale      
%             : LANE_LINE_FUSION()                : Lane Boundary Line Measurements To Line Boundary Line Track Fusion                
%             : LANE_LINE_CLOTHOID_STITCH()       : Clothoid Coefficient after multiple clothoid stitch  
%             : LANE_LINE_ASSOCIATION()           : PDAF : Probabilistic Data Association of line Track and gated Line Measurements
%             : CurveAssociation()                : Association by gaussian merge
%             : momentMatching()                  : Approximate a Gaussian mixture density as a single Gaussian using moment matching      
%             : update()                          : performs linear/nonlinear (Extended) Kalman update step
%             : ClothoidCoeffMeasModel()          : creates the measurement model for a 2D nearly constant curvature rate motion model 
%             : KALMAN_FILTER_UPDATE_LANE_LINE()  : Kalman Filter State Update of the Line Track
%             : LANE_TRACK_PREDICTION()           : State prediction of the Lane Boundary Line Tracks
%             : ConstantCurvRateModel()           : performs linear/nonlinear(Extended) Kalman prediction step for line track
%             : NEW_LINE_TRACK()                  : Initialization of new Lane Boundary Line Track 
%             : ComputeLaneWidth()                : Compute Ego Lane Width
% --------------------------------------------------------------------------------------------------------------------------------------------------------------
classdef CLOTHOID_FUSION
    methods(Static)
        % ====================================================================================================================================================== 
        function [LogWeights, sumLogWeights] = normalizeLogWeights(LogWeights)
            % Normalize the weights in log scale
            % INPUT  :    LogWeights: log weights, e.g., log likelihoods
            % OUTPUT :    LogWeights: log of the normalized weights
            %        : sumLogWeights: log of the sum of the non-normalized weights
            % --------------------------------------------------------------------------------------------------------------------------------------------------
            if length(LogWeights) == 1
               sumLogWeights = LogWeights;
               LogWeights = LogWeights - sumLogWeights;
               return;
            end
            [logWeights_aux, Index] = sort(LogWeights, 'descend');
            sumLogWeights = max(logWeights_aux) + log(1 + sum(exp(LogWeights(Index(2:end)) - max(logWeights_aux))));
            LogWeights = LogWeights - sumLogWeights;   % normalize
        end
        % ======================================================================================================================================================
        function LANE_TRACK_BOUNDARY = LANE_LINE_FUSION(LANE_TRACK_BOUNDARY_in, ASSIGNMENT_MAT, LANE_LINE_MEAS_MAT, measmodel)
		    % Lane Boundary Line Measurements To Line Boundary Line Track Fusion
            % INPUT  :    LANE_TRACK_BOUNDARY_in: Predicted Line Coefficients
			%        :    ASSIGNMENT_MAT : Assignment matrix for the gated line measurements
			%        :    LANE_LINE_MEAS_MAT : line Measurement Matrix
			%        :    measmodel : measurement model
            % OUTPUT :    LANE_TRACK_BOUNDARY: Estimated (fused) lane boundary line coefficients
            % --------------------------------------------------------------------------------------------------------------------------------------------------
            LANE_TRACK_BOUNDARY = LANE_TRACK_BOUNDARY_in;
            nLineMeas = LANE_LINE_MEAS_MAT.ValidCumulativeMeasCount(end);
            if(LANE_TRACK_BOUNDARY.nValidTracks == 0 || nLineMeas == 0)
               return;
            end
            for i = 1:LANE_TRACK_BOUNDARY.nValidTracks
                if ASSIGNMENT_MAT(1,i).nMeas ~= 0
                   LineLenPred = LANE_TRACK_BOUNDARY.LaneTrackParam(1,i).ClothoidParam.CurveLength;
                   GatedMeasIdx = find(ASSIGNMENT_MAT(1,i).isGatedMat(1,1:nLineMeas) == 1);
                   LineMeasurements = LANE_LINE_MEAS_MAT.MeasArray(:,GatedMeasIdx);
                   LogWeights = ASSIGNMENT_MAT(1,i).LogLikelihoodMat(1,GatedMeasIdx);
                   LaneLinePred = [LANE_TRACK_BOUNDARY.LaneTrackParam(1,i).ClothoidParam.LateralOffset; ...
                                   LANE_TRACK_BOUNDARY.LaneTrackParam(1,i).ClothoidParam.HeadingAngle; ...
                                   LANE_TRACK_BOUNDARY.LaneTrackParam(1,i).ClothoidParam.Curvature; ...
                                   LANE_TRACK_BOUNDARY.LaneTrackParam(1,i).ClothoidParam.CurvatureDerivative; ...
                                   LANE_TRACK_BOUNDARY.LaneTrackParam(1,i).ClothoidParam.CurveLength];
                   LaneLinePredClothoidCoeffErrCOV = LANE_TRACK_BOUNDARY.LaneTrackParam(1,i).ClothoidParam.ErrCOV; % for heading, curvature, curvrate
                   StateUpdate = CLOTHOID_FUSION.KALMAN_FILTER_UPDATE_LANE_LINE(LaneLinePred, LaneLinePredClothoidCoeffErrCOV, LineMeasurements, measmodel);
                   LaneLineFus = CLOTHOID_FUSION.LANE_LINE_ASSOCIATION(StateUpdate, LogWeights);
                   FusedLine   = CLOTHOID_FUSION.LANE_LINE_CLOTHOID_STITCH_v2(LaneLineFus, LineLenPred);
                   % update the parameters
                   LANE_TRACK_BOUNDARY.LaneTrackParam(1,i).ClothoidParam.LateralOffset = FusedLine.x(1,1);
                   LANE_TRACK_BOUNDARY.LaneTrackParam(1,i).ClothoidParam.HeadingAngle  = FusedLine.x(2,1);
                   LANE_TRACK_BOUNDARY.LaneTrackParam(1,i).ClothoidParam.Curvature     = FusedLine.x(3,1);
                   LANE_TRACK_BOUNDARY.LaneTrackParam(1,i).ClothoidParam.CurvatureDerivative = FusedLine.x(4,1);
                   LANE_TRACK_BOUNDARY.LaneTrackParam(1,i).ClothoidParam.CurveLength  = FusedLine.x(5,1);
                   LANE_TRACK_BOUNDARY.LaneTrackParam(1,i).ClothoidParam.ErrCOV = FusedLine.P;
                end
            end
        end
        % ======================================================================================================================================================
        function MergedLine = LANE_LINE_CLOTHOID_STITCH_v0(LaneLineFus, Lpred)
		    % Clothoid Coefficient after clothoid stitch of largest and second largest curve
            % INPUT  :    LaneLineFus: Estimate of lane boundary line
			%        :    Lpred : length of the estimated line
            % OUTPUT :    MergedLine : Coefficient after clothoid stitching
            % --------------------------------------------------------------------------------------------------------------------------------------------------
            MergedLine = struct; MergedLine.x = single(zeros(5,1)); MergedLine.P = single(zeros(4,4));  % init
            [LogWeights, ~] = CLOTHOID_FUSION.normalizeLogWeights(LaneLineFus.beta);  % compute
            if(length(LogWeights) == 3)
               dL = single(zeros(1,3));
               dL(1,1) = LaneLineFus.x(5,1);
               dL(1,2) = LaneLineFus.x(5,2) - LaneLineFus.x(5,1);
               dL(1,3) = LaneLineFus.x(5,3) - LaneLineFus.x(5,2);
               beta = single(zeros(1,length(dL)));
               beta(:) = 1/length(dL);
               alpha = dL/sum(dL);
               MergedLine.x(1,1) = beta * LaneLineFus.x(1,1:end)';     % merged init offset
               MergedLine.x(2,1) = beta * LaneLineFus.x(2,1:end)';     % merged init heading
               MergedLine.x(3,1) = beta * LaneLineFus.x(3,1:end)';     % merged init curvature
               curvRate1 =  LaneLineFus.x(4,1);
               curvRate2 =  LaneLineFus.x(4,2);
               curvRate3 =  LaneLineFus.x(4,3);
               MergedLine.x(4,1) = curvRate1*alpha(1,1) + curvRate2*alpha(1,2) + curvRate3*alpha(1,3);  % merged curvature rates
               MergedLine.x(5,1) = max(sum(dL), Lpred);                % merged curve length
               PP = 0;
               for i = 1:length(dL)
                   PP = PP + beta(1,i)*reshape(LaneLineFus.P(:,:,end-i+1),[4,4]);
               end
               MergedLine.P = PP; % merged curve error covariance
            elseif(length(LogWeights) == 2)
               dL = single(zeros(1,2));
               dL(1,1) = LaneLineFus.x(5,end-1);
               dL(1,2) = LaneLineFus.x(5,end) - LaneLineFus.x(5,end-1);
               beta = single(zeros(1,length(dL)));
               beta(:) = 1/length(dL);
               alpha = dL/sum(dL);
               MergedLine.x(1,1) = beta * LaneLineFus.x(1,end-1:end)';     % merged init offset
               MergedLine.x(2,1) = beta * LaneLineFus.x(2,end-1:end)';     % merged init heading
               MergedLine.x(3,1) = beta * LaneLineFus.x(3,end-1:end)';     % merged init curvature
               curvRate2 = LaneLineFus.x(4,end-1);
               curvRate3 = LaneLineFus.x(4,end);
               MergedLine.x(4,1) = curvRate2*alpha(1,1) + curvRate3*alpha(1,2); % merged curvature rates
               MergedLine.x(5,1) = max(sum(dL), Lpred);                    % merged curve length
               PP = 0;
               for i = 1:length(dL)
                   PP = PP + beta(1,i)*reshape(LaneLineFus.P(:,:,end-i+1),[4,4]);
               end
               MergedLine.P = PP; % merged curve error covariance
            elseif(length(LogWeights) == 1)
               MergedLine.x(1,1) = LaneLineFus.x(1,1);
               MergedLine.x(2,1) = LaneLineFus.x(2,1);
               MergedLine.x(3,1) = LaneLineFus.x(3,1);
               MergedLine.x(4,1) = LaneLineFus.x(4,1);
               MergedLine.x(5,1) = max(LaneLineFus.x(5,1), Lpred);
               MergedLine.P = reshape(LaneLineFus.P(:,:,1),[4,4]);
           end
        end
        % ======================================================================================================================================================
        function MergedLine = LANE_LINE_CLOTHOID_STITCH_v1(LaneLineFus, Lpred)
		    % Clothoid Coefficient after clothoid stitch of largest and second largest curve
            % INPUT  :    LaneLineFus: Estimate of lane boundary line
			%        :    Lpred : length of the estimated line
            % OUTPUT :    MergedLine : Coefficient after clothoid stitching
            % --------------------------------------------------------------------------------------------------------------------------------------------------
            MergedLine = struct; MergedLine.x = single(zeros(5,1)); MergedLine.P = single(zeros(4,4));  % init
            [LogWeights, ~] = CLOTHOID_FUSION.normalizeLogWeights(LaneLineFus.beta);  % compute
            if(length(LogWeights) == 3)
               dL = single(zeros(1,3));
               dL(1,1) = LaneLineFus.x(5,1);
               dL(1,2) = LaneLineFus.x(5,2) - LaneLineFus.x(5,1);
               dL(1,3) = LaneLineFus.x(5,3) - LaneLineFus.x(5,2);
               beta = single(zeros(1,length(dL)));
               beta(:) = 1/length(dL);
               alpha = dL/sum(dL);
               MergedLine.x(1,1) = beta * LaneLineFus.x(1,1:end)';     % merged init offset
               MergedLine.x(2,1) = beta * LaneLineFus.x(2,1:end)';     % merged init heading
               MergedLine.x(3,1) = beta * LaneLineFus.x(3,1:end)';     % merged init curvature
               curvRate1 = (LaneLineFus.x(4,1) + LaneLineFus.x(4,2) + LaneLineFus.x(4,3))/3;
               curvRate2 = (LaneLineFus.x(4,2) + LaneLineFus.x(4,3))/2;
               curvRate3 =  LaneLineFus.x(4,3);
               MergedLine.x(4,1) = curvRate1*alpha(1,1) + curvRate2*alpha(1,2) + curvRate3*alpha(1,3);  % merged curvature rates
               MergedLine.x(5,1) = max(sum(dL), Lpred);                % merged curve length
               PP = 0;
               for i = 1:length(dL)
                   PP = PP + beta(1,i)*reshape(LaneLineFus.P(:,:,end-i+1),[4,4]);
               end
               MergedLine.P = PP; % merged curve error covariance
            elseif(length(LogWeights) == 2)
               dL = single(zeros(1,2));
               dL(1,1) = LaneLineFus.x(5,end-1);
               dL(1,2) = LaneLineFus.x(5,end) - LaneLineFus.x(5,end-1);
               beta = single(zeros(1,length(dL)));
               beta(:) = 1/length(dL);
               alpha = dL/sum(dL);
               MergedLine.x(1,1) = beta * LaneLineFus.x(1,end-1:end)';     % merged init offset
               MergedLine.x(2,1) = beta * LaneLineFus.x(2,end-1:end)';     % merged init heading
               MergedLine.x(3,1) = beta * LaneLineFus.x(3,end-1:end)';     % merged init curvature
               curvRate2 = (LaneLineFus.x(4,end-1) + LaneLineFus.x(4,end))/2;
               curvRate3 = LaneLineFus.x(4,end);
               MergedLine.x(4,1) = curvRate2*alpha(1,1) + curvRate3*alpha(1,2); % merged curvature rates
               MergedLine.x(5,1) = max(sum(dL), Lpred);                    % merged curve length
               PP = 0;
               for i = 1:length(dL)
                   PP = PP + beta(1,i)*reshape(LaneLineFus.P(:,:,end-i+1),[4,4]);
               end
               MergedLine.P = PP; % merged curve error covariance
            elseif(length(LogWeights) == 1)
               MergedLine.x(1,1) = LaneLineFus.x(1,1);
               MergedLine.x(2,1) = LaneLineFus.x(2,1);
               MergedLine.x(3,1) = LaneLineFus.x(3,1);
               MergedLine.x(4,1) = LaneLineFus.x(4,1);
               MergedLine.x(5,1) = max(LaneLineFus.x(5,1), Lpred);
               MergedLine.P = reshape(LaneLineFus.P(:,:,1),[4,4]);
           end
        end
        % ======================================================================================================================================================
        function MergedLine = LANE_LINE_CLOTHOID_STITCH_v2(LaneLineFus, Lpred)
		    % Clothoid Coefficient after clothoid stitch of largest and second largest curve
            % INPUT  :    LaneLineFus: Estimate of lane boundary line
			%        :    Lpred : length of the estimated line
            % OUTPUT :    MergedLine : Coefficient after clothoid stitching
            % --------------------------------------------------------------------------------------------------------------------------------------------------
            MergedLine = struct; MergedLine.x = single(zeros(5,1)); MergedLine.P = single(zeros(4,4));  % init
            [LogWeights, ~] = CLOTHOID_FUSION.normalizeLogWeights(LaneLineFus.beta);  % compute
            if(length(LogWeights) > 1)
               dL = single(zeros(1,2));
               dL(1,1) = LaneLineFus.x(5,end-1);
               dL(1,2) = LaneLineFus.x(5,end) - LaneLineFus.x(5,end-1);
               beta = single(zeros(1,length(dL)));
               beta(:) = 1/length(dL);
               alpha = dL/sum(dL);
               MergedLine.x(1,1) = beta * LaneLineFus.x(1,end-1:end)';     % merged init offset
               MergedLine.x(2,1) = beta * LaneLineFus.x(2,end-1:end)';     % merged init heading
               MergedLine.x(3,1) = beta * LaneLineFus.x(3,end-1:end)';     % merged init curvature
               curvRate2 = (LaneLineFus.x(4,end-1) + LaneLineFus.x(4,end))/2;
               curvRate3 = LaneLineFus.x(4,end);
               MergedLine.x(4,1) = curvRate2*alpha(1,1) + curvRate3*alpha(1,2); % merged curvature rates
               MergedLine.x(5,1) = max(sum(dL), Lpred);                    % merged curve length
               PP = 0;
               for i = 1:length(dL)
                   PP = PP + beta(1,i)*reshape(LaneLineFus.P(:,:,end-i+1),[4,4]);
               end
               MergedLine.P = PP; % merged curve error covariance
            else
               MergedLine.x(1,1) = LaneLineFus.x(1,1);
               MergedLine.x(2,1) = LaneLineFus.x(2,1);
               MergedLine.x(3,1) = LaneLineFus.x(3,1);
               MergedLine.x(4,1) = LaneLineFus.x(4,1);
               MergedLine.x(5,1) = max(LaneLineFus.x(5,1), Lpred);
               MergedLine.P = reshape(LaneLineFus.P(:,:,1),[4,4]);
           end
        end
        % ======================================================================================================================================================
        function MergedLine = LANE_LINE_CLOTHOID_STITCH_v3(LaneLineFus, Lpred)
		    % Clothoid Coefficient from PDAF fusion of largest and second largest curve only
            % INPUT  :    LaneLineFus: Estimate of lane boundary line
			%        :    Lpred : length of the estimated line
            % OUTPUT :    MergedLine : Coefficient after clothoid stitching
            % --------------------------------------------------------------------------------------------------------------------------------------------------
            MergedLine = struct; MergedLine.x = single(zeros(5,1)); MergedLine.P = single(zeros(4,4));  % init
            [LogWeights, ~] = CLOTHOID_FUSION.normalizeLogWeights(LaneLineFus.beta);  % compute
            dL = single(zeros(1,length(LogWeights)));
            dL(1,1) = LaneLineFus.x(5,1);
            dL(1,2:end) = LaneLineFus.x(5,2:end) - LaneLineFus.x(5,1:end-1);
            beta = exp(LogWeights);
            beta(:) = 1/length(beta);
            MergedLine.x(1,1) = LaneLineFus.x(1,1);     % merged init offset
            MergedLine.x(2,1) = LaneLineFus.x(2,1);     % merged init heading
            MergedLine.x(3,1) = LaneLineFus.x(3,1);     % merged init curvature
            curvRate3 = LaneLineFus.x(4,1);
            MergedLine.x(4,1) = curvRate3;
            MergedLine.x(5,1) = max(sum(dL), Lpred);    % merged curve length
            PP = 0;
            for i = 1:length(LaneLineFus.x(1,:))
                PP = PP + beta(1,i)*reshape(LaneLineFus.P(:,:,i),[4,4]);
            end
            MergedLine.P = PP; % merged curve error covariance
        end
        % ======================================================================================================================================================
        function LaneLineFus = LANE_LINE_ASSOCIATION(StateUpdate, LogWeights)
		    % PDAF : Probabilistic Data Association of line Track and gated Line Measurements
            % INPUT  :    StateUpdate: Kalman Filter State estimates from Line Measurements
			%        :    LogWeights : Association log likelihoods
            % OUTPUT :    LaneLineFus : Coefficient after Lane Line Measurement to Track Association
            % --------------------------------------------------------------------------------------------------------------------------------------------------
            nSeg = size(StateUpdate.x, 2);                              % number of line measurements
            LaneLineFus = struct;
            LaneLineFus.x = single(zeros(5,nSeg));
            LaneLineFus.P = single(zeros(4,4, nSeg));
            LaneLineFus.beta = single(zeros(1,nSeg));
            CurveLengths = StateUpdate.x(5, :);                         % extract the segment lengths
            [SortedCurvLen, idx] = sort(CurveLengths, 'ascend');        % sort the curv lengths for each gated segments in ascending order
            StateUpdate.x = StateUpdate.x(:,idx);                       % rearrange the state vectors 
            StateUpdate.P = StateUpdate.P(:,:,idx);                     % rearrange the state covariance
            LogWeights = LogWeights(:,idx);                             % rearrange the log weights
            nFusedSegments = 0;
            for i = 1:nSeg                                              % iterate over each of the overlaps
                if (i == 1); curvLenStart = 0;                          % for first iteration the starting curv length is '0'
                else; curvLenStart = SortedCurvLen(1,i-1); end          % else the starting curv length is the previous value
                curvLenEnd = SortedCurvLen(1,i);                        % end of the segment
                ListCurveIndex = find(SortedCurvLen > curvLenStart);    % list of curve lengths index (Measurement matrix) which forms the overlap
                CurvesArray = StateUpdate.x(:,ListCurveIndex);          % extract the corresponding portion of the line measurement array (5 x n)
                CurvesErrCovArray = StateUpdate.P(:,:,ListCurveIndex);  % extract the corresponding portion of the line measurement array (4 x 4 X n)
                CurvesLogLikelihood = LogWeights(:,ListCurveIndex);     % Curve Association log likelihood weights
                nSegments = length(ListCurveIndex);                     % compute the number of segments
                if(nSegments ~= 0)
                   nFusedSegments = nFusedSegments + 1;
                   [NormLogWeights, sumLogWeights] = CLOTHOID_FUSION.normalizeLogWeights(CurvesLogLikelihood);  % normalize log weights
                   LineFus = CLOTHOID_FUSION.CurveAssociation(CurvesArray, CurvesErrCovArray,  NormLogWeights, curvLenEnd);  % fuse those segments
                   LaneLineFus.x(:,nFusedSegments) = LineFus.Xfus;
                   LaneLineFus.P(:,:,nFusedSegments) = LineFus.Pfus;
                   LaneLineFus.beta(:,nFusedSegments) = sumLogWeights;
                end
            end
            LaneLineFus.x = LaneLineFus.x(:,1:nFusedSegments);
            LaneLineFus.P = LaneLineFus.P(:,:,1:nFusedSegments);
            LaneLineFus.beta = LaneLineFus.beta(:,1:nFusedSegments);
        end
        % ======================================================================================================================================================
        function FusedCurveSegment = CurveAssociation(CurvesArray, CurvesErrCovArray,  LogWeights, curvLenEnd)
            % Association by gaussian merge
            % INPUT  :    CurvesArray: An array of KF state updated curves (clothoid coefficients)
			%        :    CurvesErrCovArray : corresponding error covariance matrix
			%        :    LogWeights : log likelihood weights
			%        :    curvLenEnd : maximum overlapped curvelengths of the curves
            % OUTPUT :    FusedCurveSegment : fused clothoid coefficients
            % --------------------------------------------------------------------------------------------------------------------------------------------------
            nSeg = size(CurvesArray, 2); nDim = size(CurvesArray, 1);
            Xfus = single(zeros(nDim, 1));
            X = struct; X.x = single(zeros(4,1)); X.P = single(zeros(4,4)); X = X(ones(1, nSeg));
            for i = 1:nSeg
                X(i).x = CurvesArray(1:4,i);
                X(i).P = reshape(CurvesErrCovArray(:,:,i),[4,4]);
            end
            [x, P] = CLOTHOID_FUSION.momentMatching(LogWeights, X);  % y0, a0, K0, Khat
            Xfus(1:4,1) = x; Xfus(5,1) = curvLenEnd;
            FusedCurveSegment.Xfus = Xfus; FusedCurveSegment.Pfus = P;
        end
        % ======================================================================================================================================================
        function [x, P] = momentMatching(weights, X)
            % Approximate a Gaussian mixture density as a single Gaussian using moment matching 
            % INPUT: weights: normalised weight of Gaussian components in logarithm domain (number of Gaussians) x 1 vector 
            %              X: structure array of size (number of Gaussian components x 1), each structure has two fields 
            %              x: means of Gaussian components (variable dimension) x 1 vector 
            %              P: variances of Gaussian components (variable dimension) x (variable dimension) matrix  
            % OUTPUT:  state: a structure with two fields:
            %              x: approximated mean (variable dimension) x 1 vector 
            %              P: approximated covariance (variable dimension) x (variable dimension) matrix 
            % --------------------------------------------------------------------------------------------------------------------------------------------------
            nComponents = length(weights);
            if nComponents == 1
                x = X(1,1).x;
                P = X(1,1).P;
                return;
            end
            mixtureMean = 0; Paverage = 0; meanSpread = 0;
            w = exp(weights);          % convert normalized weights from log scale to linear scale (exp(w))
            for idx = 1:nComponents    % compute the weighted average of the means (in the gaussian mixture)
                mixtureMean = mixtureMean + w(1,idx) * X(1,idx).x;
            end
            for idx = 1:nComponents                          % compute weighted average covariance and spread of the mean
                Paverage = Paverage + w(1,idx) * X(1,idx).P; % compute weighted average covariance
                meanSpread = meanSpread + w(1,idx).*( ( mixtureMean - X(1,idx).x )*( mixtureMean - X(1,idx).x )' ); % spread of the mean
            end
            x = mixtureMean;
            P = Paverage + meanSpread;
        end
        % ======================================================================================================================================================
        function state_upd = update(state_pred, z, measmodel)
            % performs linear/nonlinear (Extended) Kalman update step
            % INPUT: z: measurement (measurement dimension) x 1 vector
            %        state_pred: a structure with two fields:
            %                x: predicted object state mean (state dimension) x 1 vector 
            %                P: predicted object state covariance (state dimension) x (state dimension) matrix
            %        K: Kalman Gain
            %        measmodel: a structure specifies the measurement model parameters 
            % OUTPUT:state_upd: a structure with two fields:
            %                   x: updated object state mean (state dimension) x 1 vector                 
            %                   P: updated object state covariance (state dimension) x (state dimension) matrix 
            % --------------------------------------------------------------------------------------------------------------------------------------------------
            Hx = measmodel.H(state_pred.x);     %measurement model jacobian
            S = Hx * state_pred.P * Hx' + measmodel.R; %Innovation covariance
            S = (S + S')/2;                     %Make sure matrix S is positive definite
            K = (state_pred.P * Hx')/S;         %kalman gain
            state_upd.x = state_pred.x + K*(z - measmodel.h(state_pred.x));  %State update
            state_upd.P = (eye(size(state_pred.x,1)) - K*Hx)*state_pred.P;   %Covariance update
        end
        % ======================================================================================================================================================
        function MeasurementModel = ClothoidCoeffMeasModel(sigmaSq_y0, sigmaSq_A0, sigmaSq_K0, sigmaSq_Khat)
            % creates the measurement model for a 2D nearly constant curvature rate motion model 
            % INPUT: sigmaSq_y0, sigmaSq_A0, sigmaSq_K0, sigmaSq_Khat: standard deviation of measurement noise 
            % OUTPUT: obj.d: measurement dimension 
            %         obj.H: function handle return an observation matrix (4 x 6 matrix) 
            %         obj.R: measurement noise covariance (4 x 4 matrix)
            %         obj.h: function handle return a measurement (4 x 1 vector)
            % Its is assumed that the measurements are in the order (latOffset, heading, curvature, curvatureRate)
            % --------------------------------------------------------------------------------------------------------------------------------------------------
            MeasurementModel.dim = 3;
            MeasurementModel.H = @(x) single([1,0,0,0;...
                                              0,1,0,0;...
                                              0,0,1,0;...
                                              0,0,0,1]);
            MeasurementModel.R = single([sigmaSq_y0, 0,0,0;...
                                         0,sigmaSq_A0, 0,0;...
                                         0,0,sigmaSq_K0 ,0;...
                                         0,0,0,sigmaSq_Khat]);
            MeasurementModel.h = @(x) MeasurementModel.H(x) * x;
        end
        % ======================================================================================================================================================
        function StateUpdate = KALMAN_FILTER_UPDATE_LANE_LINE(StatePred, StateErrCOV, MeasMat, measmodel)
		    % Kalman Filter State Update of the Line Track
            % INPUT  :    StatePred: Predicted State estimate
			%        :    StateErrCOV : Error Covariance of the Predicted State estimate
			%        :    MeasMat : Line Measurements
			%        :    measmodel : measurement model
            % OUTPUT :    LaneLineFus : Coefficient after Lane Line Measurement to Track Association
            % --------------------------------------------------------------------------------------------------------------------------------------------------
            nMeas = size(MeasMat, 2);
            StateUpdate = struct;
            StateUpdate.x = single(zeros(5,nMeas));  % yOffset, a0, K0, Khat, L
            StateUpdate.P = single(zeros(4,4, nMeas));  % y0, a0, K0, Khat
            X.x = StatePred(1:4,1); X.P = StateErrCOV;
            for i = 1:nMeas              % for each of the measurements
                Z = MeasMat(1:4,i);      % extract the measurement
                state_upd = CLOTHOID_FUSION.update(X, Z, measmodel);
                StateUpdate.x(1:4,i) = state_upd.x;
                StateUpdate.x(5,i)   = MeasMat(5,i);
                StateUpdate.P(:,:,i) = state_upd.P;
            end
        end
        % ======================================================================================================================================================
        function LANE_TRACK_BOUNDARY = LANE_TRACK_PREDICTION(LANE_TRACK_BOUNDARY_in, EGO_CAN_BUS, sigmaSq_y0, sigmaSq_a0, sigmaSq_K0, sigmaSq_Khat, dT)
            % State prediction of the Lane Boundary Line Tracks
            % INPUTS : LANE_TRACK_BOUNDARY_in : previous lane boundary line tracks (at time t-1)
            %        : EGO_CAN_BUS : ego sensor parameters
            %        : sigmaSq_y0, sigmaSq_a0, sigmaSq_K0, sigmaSq_Khat : process noise
            %        : dT : sample time
            % OUTPUT : LANE_TRACK_BOUNDARY : predicted lane boundary line tracks
            % --------------------------------------------------------------------------------------------------------------------------------------------------
            LANE_TRACK_BOUNDARY = LANE_TRACK_BOUNDARY_in;
            if(LANE_TRACK_BOUNDARY.nValidTracks == 0) % do not perform state prediction if no objects are present
                return;
            end
            DEG2RAD = single(pi/180);
            vEgo  = sqrt( (EGO_CAN_BUS.vx)^2 + (EGO_CAN_BUS.vy)^2 );
            dYaw  = (-1) * DEG2RAD * EGO_CAN_BUS.yawRate * dT;
            dS = vEgo * dT;
            for idx = 1:LANE_TRACK_BOUNDARY.nValidTracks
                % prediction of the track estimate
                state.x = [LANE_TRACK_BOUNDARY.LaneTrackParam(1,idx).ClothoidParam.LateralOffset; ... 
                           LANE_TRACK_BOUNDARY.LaneTrackParam(1,idx).ClothoidParam.HeadingAngle; ... 
                           LANE_TRACK_BOUNDARY.LaneTrackParam(1,idx).ClothoidParam.Curvature; ...
                           LANE_TRACK_BOUNDARY.LaneTrackParam(1,idx).ClothoidParam.CurvatureDerivative];
                state.P  = LANE_TRACK_BOUNDARY.LaneTrackParam(1,idx).ClothoidParam.ErrCOV;
                curvLen = LANE_TRACK_BOUNDARY.LaneTrackParam(1,idx).ClothoidParam.CurveLength;
                [state, ~] = CLOTHOID_FUSION.ConstantCurvRateModel(state, curvLen, sigmaSq_y0, sigmaSq_a0, sigmaSq_K0, sigmaSq_Khat, dS, dYaw);  % prediction and compensation
                % Write back (Fusion Prediction)
                LANE_TRACK_BOUNDARY.LaneTrackParam(1,idx).ClothoidParam.LateralOffset = state.x(1,1);
                LANE_TRACK_BOUNDARY.LaneTrackParam(1,idx).ClothoidParam.HeadingAngle = state.x(2,1);
                LANE_TRACK_BOUNDARY.LaneTrackParam(1,idx).ClothoidParam.Curvature = state.x(3,1);
                LANE_TRACK_BOUNDARY.LaneTrackParam(1,idx).ClothoidParam.CurvatureDerivative = state.x(4,1);
                LANE_TRACK_BOUNDARY.LaneTrackParam(1,idx).ClothoidParam.ErrCOV = state.P;
                LANE_TRACK_BOUNDARY.LaneTrackParam(1,idx).ClothoidParam.CurveLength = ...
                               LANE_TRACK_BOUNDARY.LaneTrackParam(1,idx).ClothoidParam.CurveLength - dS;
            end
        end
        % ======================================================================================================================================================
        function [StatePred , ProcessModel] = ConstantCurvRateModel(state, L, sigmaSq_y0, sigmaSq_a0, sigmaSq_K0, sigmaSq_Khat, dS, dYaw)
            %performs linear/nonlinear(Extended) Kalman prediction step for line track
            %INPUT: state: a structure with two fields:
            %           x: object state mean (dim x 1 vector) 
            %           P: object state covariance ( dim x dim matrix ) 
			%          sigmaSq_y0, sigmaSq_a0, sigmaSq_K0, sigmaSq_Khat : process noise
            %          dS: distance travelled by the ego vehicle along the arc
            %          dYaw : change in ego vehicle yaw
            %OUTPUT:state_pred: a structure with two fields:
            %           x: predicted object state mean (dim x 1 vector)
            %           P: predicted object state covariance ( dim x dim matrix )
            % --------------------------------------------------------------------------------------------------------------------------------------------------
            ProcessModel.dim = 4; % state dimension
            ProcessModel.F = single([1, dS, 0.5*(dS^2), 0.1667*(dS^3); ...
                                     0,  1,         dS,    0.5*(dS^2); ...
                                     0,  0,          1,            dS; ...
                                     0,  0,          0,             1]);
            ProcessModel.Q = single([sigmaSq_y0, 0, 0, 0; ...
                                     0, sigmaSq_a0, 0, 0; ...
                                     0, 0, sigmaSq_K0, 0; ...
                                     0, 0, 0, sigmaSq_Khat]);
            [~, yOffset, ~, ~] =  CURVE.ClothoidToCartTaylorApprox(0, state.x(1,1), state.x(2,1), state.x(3,1), state.x(4,1), dS, L);                     
            ProcessModel.f(1,1) = yOffset;
            ProcessModel.f(2,1) = state.x(2,1) + state.x(3,1)*dS + 0.5*state.x(4,1)*dS*dS;
            ProcessModel.f(3,1) = state.x(3,1) + state.x(4,1)*dS;
            ProcessModel.f(4,1) = state.x(4,1);
            StatePred.x = ProcessModel.f;
            StatePred.P = ProcessModel.F * state.P * ProcessModel.F' + ProcessModel.Q;
            StatePred.x(1,1) = StatePred.x(1,1) + dYaw;
        end
        % ======================================================================================================================================================
        function LANE_TRACK_BOUNDARY = NEW_LINE_TRACK(trigger, LANE_LINE_MEAS_MAT, LANE_LINES_CLUSTERS, UNASSOCIATED_CLUSTERS_LINE, ...
                                                      cntLineClst, LANE_TRACK_BOUNDARY)
			% Initialization of new Lane Boundary Line Track 
            % INPUTS : trigger : flag that indicated the system is executed for the first time
            %        : LANE_LINE_MEAS_MAT : Lane Line Measurements
            %        : LANE_LINES_CLUSTERS : Lane Line Clusters
            %        : UNASSOCIATED_CLUSTERS_LINE : Unassociate clusters
			%        : cntLineClst : count the number of clusters
			%        : LANE_TRACK_BOUNDARY : Lane Line Track Boundary
            % OUTPUT : LANE_TRACK_BOUNDARY : New Lane Line Track Updated Lane Line Track Boundary Data structure
            % --------------------------------------------------------------------------------------------------------------------------------------------------											   
            if(~trigger); return; end
            nLineMeas = LANE_LINE_MEAS_MAT.ValidCumulativeMeasCount(1,end); % number of Lane line measurements 
            curvLenIdx = 5;                                                 % array index for curve length 
            maxSize = 100; LaneBoundaries = single(zeros(6, maxSize));      % array to hold the fused and stitched lane boundaries
            if(cntLineClst == 0); return; end
            for i = 1:cntLineClst                                           % iterate over each of the line clusters
                clstrID = UNASSOCIATED_CLUSTERS_LINE(1,i);                  % extract line cluster ID
                LineIDsIndex = find(LANE_LINES_CLUSTERS.ClustIDAssig(1,1:nLineMeas) == clstrID);  % extract the line measurement array indexes for the belonging to the same cluster
                LineMeasurements = LANE_LINE_MEAS_MAT.MeasArray(:,LineIDsIndex); %#ok<FNDSB> % extract the line measurements 
                [~, idx] = sort(LineMeasurements(curvLenIdx,:), 'descend');      % sort the curv lengths for each cluster members in ascending order
                x0 = 0;LaneBoundaries(1, i) = x0;LaneBoundaries(2:6, i) = LineMeasurements(:,idx(1));  % use the largest curve
            end
      
            % compute width (create an index in the decreasing order of yoffset)
            curvLengths = LaneBoundaries(6,1:cntLineClst);        % extract the curve lengths for all the lane boundaries
            [~, indexCurvLen] = sort(curvLengths, 'descend');     % sort them in descending order
            LaneBoundaryMain = LaneBoundaries(:,indexCurvLen(1)); % select the largest lane boundary
            for i = 1:cntLineClst-1                               % iterate over the remaining borders
                LaneBoundaries(6, indexCurvLen(i+1)) = LaneBoundaryMain(6,1);
            end
            LaneBoundaries = LaneBoundaries(:, 1:cntLineClst);    % new lane borders
      
            % Update the Lane Track Boundary
            y0idx = 2; a0idx = 3; k0idx = 4; khatidx = 5; Lidx = 6;
            nExistingLanes = LANE_TRACK_BOUNDARY.nValidTracks;
            for k = 1:cntLineClst
                idx = nExistingLanes + k;
                LANE_TRACK_BOUNDARY.LaneTrackParam(idx).ClothoidParam.LateralOffset = LaneBoundaries(y0idx, k);
                LANE_TRACK_BOUNDARY.LaneTrackParam(idx).ClothoidParam.HeadingAngle  = LaneBoundaries(a0idx, k);
                LANE_TRACK_BOUNDARY.LaneTrackParam(idx).ClothoidParam.Curvature     = LaneBoundaries(k0idx, k);
                LANE_TRACK_BOUNDARY.LaneTrackParam(idx).ClothoidParam.CurvatureDerivative = LaneBoundaries(khatidx, k);
                LANE_TRACK_BOUNDARY.LaneTrackParam(idx).ClothoidParam.CurveLength   = LaneBoundaries(Lidx, k);
            end
            LANE_TRACK_BOUNDARY.nValidTracks = LANE_TRACK_BOUNDARY.nValidTracks + cntLineClst;
        end
        % ======================================================================================================================================================
        function[Width, RoadWidth] = ComputeLaneWidth(LANE_TRACK_BOUNDARY)
		    % Compute Ego Lane Width
            % INPUTS : LANE_TRACK_BOUNDARY : Ego Lane Boundary Line Track
            % OUTPUT : Width : average of the road width
			%        : RoadWidth : array of road widths at different points
            % --------------------------------------------------------------------------------------------------------------------------------------------------
            L1 = LANE_TRACK_BOUNDARY.LaneTrackParam(1).ClothoidParam.CurveLength;
            L2 = LANE_TRACK_BOUNDARY.LaneTrackParam(2).ClothoidParam.CurveLength;
            if(L1 >= L2)
                x0_main   = 0;
                y0_main   = LANE_TRACK_BOUNDARY.LaneTrackParam(1).ClothoidParam.LateralOffset;
                a0_main   = LANE_TRACK_BOUNDARY.LaneTrackParam(1).ClothoidParam.HeadingAngle;
                k0_main   = LANE_TRACK_BOUNDARY.LaneTrackParam(1).ClothoidParam.Curvature;
                khat_main = LANE_TRACK_BOUNDARY.LaneTrackParam(1).ClothoidParam.CurvatureDerivative;
                L_main    = LANE_TRACK_BOUNDARY.LaneTrackParam(1).ClothoidParam.CurveLength;
         
                x0_other   = 0;
                y0_other   = LANE_TRACK_BOUNDARY.LaneTrackParam(2).ClothoidParam.LateralOffset;
                a0_other   = LANE_TRACK_BOUNDARY.LaneTrackParam(2).ClothoidParam.HeadingAngle;
                k0_other   = LANE_TRACK_BOUNDARY.LaneTrackParam(2).ClothoidParam.Curvature;
                khat_other = LANE_TRACK_BOUNDARY.LaneTrackParam(2).ClothoidParam.CurvatureDerivative;
                L_other    = LANE_TRACK_BOUNDARY.LaneTrackParam(2).ClothoidParam.CurveLength;
            else
                x0_main   = 0;
                y0_main   = LANE_TRACK_BOUNDARY.LaneTrackParam(2).ClothoidParam.LateralOffset;
                a0_main   = LANE_TRACK_BOUNDARY.LaneTrackParam(2).ClothoidParam.HeadingAngle;
                k0_main   = LANE_TRACK_BOUNDARY.LaneTrackParam(2).ClothoidParam.Curvature;
                khat_main = LANE_TRACK_BOUNDARY.LaneTrackParam(2).ClothoidParam.CurvatureDerivative;
                L_main    = LANE_TRACK_BOUNDARY.LaneTrackParam(2).ClothoidParam.CurveLength;
         
                x0_other   = 0;
                y0_other   = LANE_TRACK_BOUNDARY.LaneTrackParam(1).ClothoidParam.LateralOffset;
                a0_other   = LANE_TRACK_BOUNDARY.LaneTrackParam(1).ClothoidParam.HeadingAngle;
                k0_other   = LANE_TRACK_BOUNDARY.LaneTrackParam(1).ClothoidParam.Curvature;
                khat_other = LANE_TRACK_BOUNDARY.LaneTrackParam(1).ClothoidParam.CurvatureDerivative;
                L_other    = LANE_TRACK_BOUNDARY.LaneTrackParam(1).ClothoidParam.CurveLength;
            end
            nn = 100; dL = L_other/(nn-1); s = 0:dL:L_other;
            other_phi_s = a0_other + abs(k0_other).*s + 0.5*abs(khat_other).*s.^2;
            main_curv_s = ( sqrt(k0_main^2 -2*abs(khat_main).*(a0_main - other_phi_s)) - abs(k0_main) )/abs(khat_main);
      
            [xMain, yMain, a0Main, ~] = CURVE.ClothoidToCartTaylorApprox(x0_main, y0_main, a0_main, k0_main, khat_main, main_curv_s, L_main);
            [xOther, yOther, a0Other, ~] = CURVE.ClothoidToCartTaylorApprox(x0_other, y0_other, a0_other, k0_other, khat_other, s, L_other);
            RoadWidth = single(size(main_curv_s));
            X1 = single(size(main_curv_s)); Y1 = single(size(main_curv_s));
            X2 = single(size(main_curv_s)); Y2 = single(size(main_curv_s));
            for i = 1:length(main_curv_s)
                x1 = xMain(i);  y1 = yMain(i);
                x2 = xOther(i); y2 = yOther(i);
                m1 = tan(a0Main(i));
                c1 = y1 - m1*x1;
                m2 = -(1/tan(a0Other(i)));
                c2 = y2 - m2*x2;
                x = (c1 - c2)/(m2 - m1);
                y = m1*x + c1;
                RoadWidth(i) = sqrt((x - x2)^2 + (y - y2)^2);
                X1(i) = x; Y1(i) = y; X2(i) = x2; Y2(i) = y2;
            end
            RoadWidth = real(RoadWidth);
            Width = sum(RoadWidth)/length(RoadWidth);
        end
        % ======================================================================================================================================================
    end
end