% --------------------------------------------------------------------------------------------------------------------------------------------------------------
% Author Name : Udit Bhaskar Talukdar
% Date        : 01/01/2021
% Description : EGO LANE & ROAD PROFILE Geometry computation library , Contains the following functions :
%             : LANE_LINE_MODEL()                : Road Grid Computation and History Generation
%             : LANE_GEOMETRY()                  : Lane Geometry Coefficient computation
%             : EGO_LANE_HYPOTHESIS()            : Ego Lane Center Line Multiple hypotheisied Geometry Coefficient computation
%             : GenerateLaneHypothesis()         : Generate parallel line 
%             : EGO_LANE_PREDICTION()            : Ego lane Center Line Prediction
%             : EGO_LANE_UPDATE()                : Ego lane Center Line Kalman Filter Update
%             : EGO_LANE_TRACK_TO_TRACK_FUSION() : Ego Lane Center Line Hypothesis estimation by multiple hypothesis merge (Track to Track Fusion)
% --------------------------------------------------------------------------------------------------------------------------------------------------------------
classdef ROAD_MODEL
    methods(Static)
        % ======================================================================================================================================================
        function LineGrid = LANE_LINE_MODEL(LineGridPrev, LineClothoidCoeff, EgoSnsrData, delS, SLimitUpper, SLimitLower, dT)
            % Road Grid Computation and History Generation
            % INPUT : LineGridPrev : Road Grid data previous t-1
            %         LineClothoidCoeff  : Clothoid coefficient of the ego lane center line estimation at curent time t
            %         EgoSnsrData      : ego sensor data
            %         delS, SLimitUpper, SLimitLower : Line grid limits (resolution, maxlength, minlength)
            %         dT               : sample time
            % OUTPUT : EgoLaneCenter   : Updated Road Grid data
            % --------------------------------------------------------------------------------------------------------------------------------------------------
            LineGrid = LineGridPrev; % Road Grid data previous time (t-1)
            if(LineGridPrev.validCurvLenFront == 0 && LineGridPrev.validCurvLenRear == 0 && LineClothoidCoeff.CurveLength == 0)
               return;
            end
            %HistoryLimit = 2000000;
            HistoryLimit = length(0:delS:abs(SLimitLower));      % compute the max capacity of the history buffer 
            nPts = length(0:delS:abs(SLimitUpper));              % compute the max capacity of the current buffer 
            DEG2RAD = single(pi/180);                            % degree to radian conversion factor
            dyaw  = (-1) * DEG2RAD * EgoSnsrData.yawRate * dT;   % ego vehicle change in yaw
            vEgo  = sqrt((EgoSnsrData.vx)^2 + (EgoSnsrData.vy)^2);  % ego vehicle velocity w.r.t global frame
            dS = vEgo * dT;                                      % distance traveled by the ego vehicle during dT
            vxEgo = vEgo*cos(dyaw); vyEgo = vEgo*sin(dyaw);      % lateral and longitudinal velocity of the ego vehicle in body frame
            dXEgo = [-vxEgo*dT ; -vyEgo*dT];                     % distance vector travelled by the ego vehicle during dT
            yawCompMat = [ cos(dyaw), -sin(dyaw); ...
                           sin(dyaw),  cos(dyaw)];               % ego vehicle yaw rate compensation matrix
            % extract the line clothoid parameters
            x0 = 0; y0 = LineClothoidCoeff.LateralOffset; 
            phi0 = LineClothoidCoeff.HeadingAngle; 
            K0 = LineClothoidCoeff.Curvature; 
            Khat = LineClothoidCoeff.CurvatureDerivative; 
            L = LineClothoidCoeff.CurveLength;

            % generate road profile grid points for the first time given that we have an estimation for the ego lane
            if(LineGridPrev.validCurvLenFront == 0 && LineGridPrev.validCurvLenRear == 0 && LineClothoidCoeff.CurveLength ~= 0)
               s = SLimitUpper:-delS:0;        % generate arc lengths for every 'ds'
               [x, y, phi, K] = CURVE.ClothoidToCartTaylorApprox(x0, y0, phi0, K0, Khat, s, L); % generate x,y,slope,curvature on those positions
               LineGrid.X(1,1:nPts) = x;                         % set x coordinate of the grid
               LineGrid.Y(1,1:nPts) = y;                         % set y coordinate of the grid
               LineGrid.arcLength(1,1:nPts) = s;                 % set the arc length w.r.t ego frame of the grid
               LineGrid.HeadingAngle(1,1:nPts) = phi;            % set the slope at the x,y points of the grid
               LineGrid.Curvature(1,1:nPts) = K;                 % set the curvature at the x,y points of the grid
               LineGrid.CurvatureDerivative(1,1:nPts) = Khat;    % set the curvatureRate at the x,y points of the
               LineGrid.validCurvLenFront = SLimitUpper;         % set the upperlimit of the line grid
               LineGrid.validCurvLenRear = 0;                    % set the lowerlimit of the line grid
               LineGrid.BufferLastValidIdx = nPts;               % set the number of points generated on the line grid
             else % update the buffer  
               lastBuffIdx = LineGrid.BufferLastValidIdx;        % last occupied index of the history
               prev_s = LineGrid.arcLength(1,1:lastBuffIdx);     % arc lengths at different points on the grid from the previous cycle
               curr_s = prev_s - dS;                             % compensate the arc lengths for the ego vehicle movement dS
               idxBack = find(curr_s(1,1:lastBuffIdx) < 0);      % list of index whose arc lengths are negetive (indicating it has moved behind the ego vehicle : push it in history)
               idxFrnt = find(curr_s(1,1:lastBuffIdx) >= 0);     % list of index whose arc lengths are positive (compute parameters from line estimations at these arc lengths)
               sMax = SLimitUpper;
               sMin = curr_s(1,idxFrnt(1,end));                  % minimum positive arc length
               sMax = sMax + sMin;                               % extended maximum arc length
               s = sMax:-delS:sMin;                              % generate new points in from of the EV
               [x, y, phi, K] = CURVE.ClothoidToCartTaylorApprox(x0, y0, phi0, K0, Khat, s, L);   % generate x,y,slope,curvature on those positions
               nPtsBack = length(idxBack);                       % number of new history points
               nPtsTobeRemoved = 0;                              % number of history points if the buffer is full
               if(nPtsBack > HistoryLimit)                       % if the number of points exceeds a threshold then remove the 'oldest' points
                  nPtsTobeRemoved = nPtsBack - HistoryLimit;     % compute the number of points to be removed from the history buffer
               end
               % extract the relevant points for ego motion compensation of history  
               idxHistory = find(curr_s(1,1:(lastBuffIdx-nPtsTobeRemoved)) < 0);        % History that needs to be maintained 
               Xprev = LineGrid.X(1,idxHistory);                                        % x prev      
               Yprev = LineGrid.Y(1,idxHistory);                                        % y prev
               Sprev = LineGrid.arcLength(1,idxHistory);                                % arc lengths prev
               phiPrev = LineGrid.HeadingAngle(1,idxHistory);                           % phi prev
               kPrev = LineGrid.Curvature(1,idxHistory);                                % K prev
               khatPrev = LineGrid.CurvatureDerivative(1,idxHistory);                   % k hat prev
               % ego motion compensation for history update
               XX = [Xprev ; Yprev];               % create a vector/matrix of x, y points
               xHist = yawCompMat * XX + dXEgo;    % ego motion compensation by rotation followed by translation
               sHist = Sprev - dS;                 % arc lengths ego compensation
               phiHist = phiPrev + dyaw;           % slope ego compensation
               kHist = kPrev;                      % curvature doesnot change
               khatHist = khatPrev;                % curvature rate doesnot change
               % update the data buffer (front of EV)
               LineGrid.X(1,1:nPts) = x;
               LineGrid.Y(1,1:nPts) = y;
               LineGrid.arcLength(1,1:nPts) = s;
               LineGrid.HeadingAngle(1,1:nPts) = phi;
               LineGrid.Curvature(1,1:nPts) = K;
               LineGrid.CurvatureDerivative(1,1:nPts) = Khat;
               LineGrid.validCurvLenFront = sMax;
               % update the data buffer (rear of EV)
               nPtsHistory = length(idxHistory);
               idxStart = nPts + 1; idxEnd = nPts + nPtsHistory;
               LineGrid.X(1,idxStart:idxEnd) = xHist(1,:);
               LineGrid.Y(1,idxStart:idxEnd) = xHist(2,:);
               LineGrid.arcLength(1,idxStart:idxEnd)    = sHist;
               LineGrid.HeadingAngle(1,idxStart:idxEnd) = phiHist;
               LineGrid.Curvature(1,idxStart:idxEnd)    = kHist;
               LineGrid.CurvatureDerivative(1,idxStart:idxEnd) = khatHist;
               LineGrid.validCurvLenRear = min(sHist);
               LineGrid.BufferLastValidIdx = idxEnd;
            end
        end
        % ======================================================================================================================================================
        function LaneBoundaryCoeff = LANE_GEOMETRY(LaneBoundaryCoeff_in, EGO_LANE_CENTER, d)
	    % Lane Geometry Coefficient computation
            % INPUT : LaneBoundaryCoeff_in : initialized data structure to hold the computed clothoid coefficients.
            %         EGO_LANE_CENTER  : Clothoid coefficient of the ego lane center line estimation at curent time t
            %         d                : parallel distance of the computed curve
            % OUTPUT : LaneBoundaryCoeff   : computed clothoid coefficients
            % --------------------------------------------------------------------------------------------------------------------------------------------------
            LaneBoundaryCoeff = LaneBoundaryCoeff_in;
            if(EGO_LANE_CENTER.CurveLength == 0)
               return;
            end
            [~, y0, phi0, K0, K_hat, L] = ROAD_MODEL.GenerateLaneHypothesis(EGO_LANE_CENTER, d);
            LaneBoundaryCoeff.LateralOffset = y0;
            LaneBoundaryCoeff.HeadingAngle = phi0;
            LaneBoundaryCoeff.Curvature = K0;
            LaneBoundaryCoeff.CurvatureDerivative = K_hat;
            LaneBoundaryCoeff.CurveLength = L;
        end
        % ======================================================================================================================================================
        function EGO_LANE_HYPOTHESIS = EGO_LANE_HYPOTHESIS(EGO_LANE_HYPOTHESIS_in, LANE_BOUNDARY, LaneHalfWidth)
	    % Ego Lane Center Line Multiple hypotheisied Geometry Coefficient computation
            % INPUT : EGO_LANE_HYPOTHESIS_in : initialized data structure to hold the computed clothoid coefficients.
            %         LANE_BOUNDARY          : Clothoid coefficient of the estimated ego lane boundary lines.
            %         LaneHalfWidth          : ego lane half width
            % OUTPUT : EGO_LANE_HYPOTHESIS   : Multiple hypotheisied clothoid  Coefficients
            % --------------------------------------------------------------------------------------------------------------------------------------------------
            EGO_LANE_HYPOTHESIS = EGO_LANE_HYPOTHESIS_in;
            if(LANE_BOUNDARY.nValidTracks == 0)
               return;
            end
            for i = 1:LANE_BOUNDARY.nValidTracks
                LateralOffsetSign = sign(LANE_BOUNDARY.LaneTrackParam(i).ClothoidParam.LateralOffset);
                LaneHalfWidthVector = (-LateralOffsetSign*LaneHalfWidth);
                [~, y0, phi0, K0, K_hat, L] = ROAD_MODEL.GenerateLaneHypothesis(LANE_BOUNDARY.LaneTrackParam(i).ClothoidParam, LaneHalfWidthVector);
                EGO_LANE_HYPOTHESIS.ClothoidParam(1,i).LateralOffset = y0;
                EGO_LANE_HYPOTHESIS.ClothoidParam(1,i).HeadingAngle = phi0;
                EGO_LANE_HYPOTHESIS.ClothoidParam(1,i).Curvature = K0;
                EGO_LANE_HYPOTHESIS.ClothoidParam(1,i).CurvatureDerivative = K_hat;
                EGO_LANE_HYPOTHESIS.ClothoidParam(1,i).CurveLength = L;
            end
            EGO_LANE_HYPOTHESIS.nValidTracks = LANE_BOUNDARY.nValidTracks;
        end
        % ======================================================================================================================================================
        function [x0, y0, phi0, K0, K_hat, L] = GenerateLaneHypothesis(a, d)
            % Generate parallel line 
            % INPUT : a : clothoid coefficients
            %         d : parallel distance
            % OUTPUT : x0, y0, phi0, K0, K_hat, L   : clothoid coefficients of the parallel curve
            % --------------------------------------------------------------------------------------------------------------------------------------------------
            x0 = 0; y0 = a.LateralOffset;                         % extract the init x and y
            phi0 = a.HeadingAngle; K0 = a.Curvature;              % extract init heading and curvature
            Khat = a.CurvatureDerivative; L = a.CurveLength;      % extract the curvature rate and curve length
            nPts = 100; sMax = L; sMin = 0; ds = (sMax - sMin)/(nPts - 1); s = sMin:ds:sMax;  % generate 'nPts' equally spaced arc along the curve
            alphaSegment = phi0 + K0.*s + 0.5*Khat.*s.^2;         % compute the heading at those 'nPts' positions along the curve
            [xSeg, ySeg, ~, ~] = CURVE.ClothoidToCartTaylorApprox(x0, y0, phi0, K0, Khat, s, L); % generate x,y points along the curve
            X = xSeg - d.*sin(alphaSegment);
            Y = ySeg + d.*cos(alphaSegment);
            C = CURVE.LeastSquaresFit3rdDegreePolynomial(X, Y);   % least squares fitting of a 3rd order polynomial
            [x0, y0, phi0, K0, K_hat, L] = CURVE.Polynomial2Clothoid_v2(C(1), C(2), C(3), C(4), 0, max(X), 1000);
            %[x0, y0, phi0, K0, K_hat, L] = CURVE.Polynomial2Clothoid(C(1), C(2), C(3), C(4), 0, max(X), 1000);
            %[x0, y0, phi0, K0, K_hat, L] = CURVE.ClothoidTranslation(x0, y0, phi0, K0, Khat, L, d);
        end
        % ======================================================================================================================================================
        function EGO_LANE_CENTER = EGO_LANE_PREDICTION(EGO_LANE_CENTER_in, EGO_CAN_BUS, sigmaSq_y0, sigmaSq_a0, sigmaSq_K0, sigmaSq_Khat, dT)
            % Ego lane Center Line Prediction
            % INPUT : EGO_LANE_CENTER_in : Ego lane Center Line coefficients at previous time t-1
            %         EGO_CAN_BUS : ego sensor data
	    %         sigmaSq_y0, sigmaSq_a0, sigmaSq_K0, sigmaSq_Khat : process noise
	    %         dT : sample time
            % OUTPUT : EGO_LANE_CENTER   : Ego lane Center Line predicted coefficients
            % --------------------------------------------------------------------------------------------------------------------------------------------------
            EGO_LANE_CENTER = EGO_LANE_CENTER_in;
            if(EGO_LANE_CENTER.CurveLength == 0)
               return;
            end
            DEG2RAD = single(pi/180);
            vEgo  = sqrt( (EGO_CAN_BUS.vx)^2 + (EGO_CAN_BUS.vy)^2 );
            dYaw  = (-1) * DEG2RAD * EGO_CAN_BUS.yawRate * dT;
            dS = vEgo * dT;
            state.x = [EGO_LANE_CENTER.LateralOffset; ...
                       EGO_LANE_CENTER.HeadingAngle; ... 
                       EGO_LANE_CENTER.Curvature; ...
                       EGO_LANE_CENTER.CurvatureDerivative];
            state.P  = EGO_LANE_CENTER.ErrCOV;
            curvLen = EGO_LANE_CENTER.CurveLength;
            [state, ~] = CLOTHOID_FUSION.ConstantCurvRateModel(state, curvLen, sigmaSq_y0, sigmaSq_a0, sigmaSq_K0, sigmaSq_Khat, dS, dYaw);  % prediction and ego yaw rate compensation
            EGO_LANE_CENTER.LateralOffset = state.x(1,1);
            EGO_LANE_CENTER.HeadingAngle = state.x(2,1);
            EGO_LANE_CENTER.Curvature = state.x(3,1);
            EGO_LANE_CENTER.CurvatureDerivative = state.x(4,1);
            EGO_LANE_CENTER.ErrCOV = state.P;
            EGO_LANE_CENTER.CurveLength = EGO_LANE_CENTER.CurveLength - dS;
        end
        % ======================================================================================================================================================
        function EGO_LANE_HYPOTHESIS = EGO_LANE_UPDATE(EGO_LANE_HYPOTHESIS_in, EGO_LANE_CENTER, measmodel)
	    % Ego lane Center Line Kalman Filter Update
            % INPUT : EGO_LANE_HYPOTHESIS_in : data structure to hold the Kalman Filter state updated Ego lane Center Line from different hypothesis (measurement)  
            %         EGO_LANE_CENTER : Ego lane Center Line Prediction
	    %         measmodel : measurement model
            % OUTPUT : EGO_LANE_HYPOTHESIS   : Kalman Filter state updated Ego lane Center Line from different hypothesis 
            % --------------------------------------------------------------------------------------------------------------------------------------------------
            EGO_LANE_HYPOTHESIS = EGO_LANE_HYPOTHESIS_in;
            if(EGO_LANE_CENTER.CurveLength == 0)
               return;
            end
           if(EGO_LANE_CENTER.CurveLength ~= 0 && EGO_LANE_HYPOTHESIS.nValidTracks == 0)
              for i = EGO_LANE_HYPOTHESIS.nValidTracks
                  EGO_LANE_HYPOTHESIS.ClothoidParam(1,i) = EGO_LANE_CENTER;
              end
              return;
           end
           X.x = [EGO_LANE_CENTER.LateralOffset; ...
                  EGO_LANE_CENTER.HeadingAngle; ...
                  EGO_LANE_CENTER.Curvature; ...
                  EGO_LANE_CENTER.CurvatureDerivative];
           X.P = EGO_LANE_CENTER.ErrCOV;
           for i = EGO_LANE_HYPOTHESIS.nValidTracks
               Z = [EGO_LANE_HYPOTHESIS.ClothoidParam(1,i).LateralOffset; ...
                    EGO_LANE_HYPOTHESIS.ClothoidParam(1,i).HeadingAngle; ...
                    EGO_LANE_HYPOTHESIS.ClothoidParam(1,i).Curvature; ...
                    EGO_LANE_HYPOTHESIS.ClothoidParam(1,i).CurvatureDerivative];
               StateUpdate = CLOTHOID_FUSION.update(X, Z, measmodel);
               EGO_LANE_HYPOTHESIS.ClothoidParam(1,i).LateralOffset = StateUpdate.x(1,1);
               EGO_LANE_HYPOTHESIS.ClothoidParam(1,i).HeadingAngle = StateUpdate.x(2,1);
               EGO_LANE_HYPOTHESIS.ClothoidParam(1,i).Curvature = StateUpdate.x(3,1);
               EGO_LANE_HYPOTHESIS.ClothoidParam(1,i).CurvatureDerivative = StateUpdate.x(4,1);
               EGO_LANE_HYPOTHESIS.ClothoidParam(1,i).ErrCOV = StateUpdate.P;
               EGO_LANE_HYPOTHESIS.ClothoidParam(1,i).CurveLength = max(EGO_LANE_HYPOTHESIS.ClothoidParam(1,i).CurveLength, EGO_LANE_CENTER.CurveLength);
           end
        end
        % ======================================================================================================================================================    
        function EGO_LANE_CENTER = EGO_LANE_TRACK_TO_TRACK_FUSION(EGO_LANE_CENTER_in, EGO_LANE_HYPOTHESIS)
	    % Ego Lane Center Line Hypothesis estimation by multiple hypothesis merge (Track to Track Fusion)
            % INPUT : EGO_LANE_CENTER_in : data structure to hold the estimated Ego Lane Center Line geometry  
            %         EGO_LANE_HYPOTHESIS : Ego lane Center Line different hypothesised geometry 
            % OUTPUT : EGO_LANE_CENTER   : estimated Ego Lane Center Line geometry
            % --------------------------------------------------------------------------------------------------------------------------------------------------
               EGO_LANE_CENTER = EGO_LANE_CENTER_in;
               if(EGO_LANE_HYPOTHESIS.nValidTracks == 0)
                  return;
               end
               if(EGO_LANE_HYPOTHESIS.nValidTracks == 1)
                  EGO_LANE_CENTER = EGO_LANE_HYPOTHESIS.ClothoidParam(1,1);
               else
                  X = single(zeros(5,EGO_LANE_HYPOTHESIS.nValidTracks)); 
                  P = single(zeros(4,4,EGO_LANE_HYPOTHESIS.nValidTracks));
                  for i = 1:EGO_LANE_HYPOTHESIS.nValidTracks
                      X(1,i)   = EGO_LANE_HYPOTHESIS.ClothoidParam(1,i).LateralOffset;
                      X(2,i)   = EGO_LANE_HYPOTHESIS.ClothoidParam(1,i).HeadingAngle;
                      X(3,i)   = EGO_LANE_HYPOTHESIS.ClothoidParam(1,i).Curvature;
                      X(4,i)   = EGO_LANE_HYPOTHESIS.ClothoidParam(1,i).CurvatureDerivative;
                      X(5,i)   = EGO_LANE_HYPOTHESIS.ClothoidParam(1,i).CurveLength;
                      P(:,:,i) = EGO_LANE_HYPOTHESIS.ClothoidParam(1,i).ErrCOV;
                  end
                  Xfus = single(zeros(5,1)); Pfus = single(zeros(4,4));
                  Lmax = max(X(5,:)); W = X(5,:)/sum(X(5,:));
                  for i = 1:EGO_LANE_HYPOTHESIS.nValidTracks
                      Xfus = Xfus + W(1,i).*X(:,i);
                      Pfus = Pfus + W(1,i).*(reshape(P(:,:,i),[4,4]));
                  end
                  EGO_LANE_CENTER.LateralOffset       = Xfus(1,1);
                  EGO_LANE_CENTER.HeadingAngle        = Xfus(2,1);
                  EGO_LANE_CENTER.Curvature           = Xfus(3,1);
                  EGO_LANE_CENTER.CurvatureDerivative = Xfus(4,1);
                  EGO_LANE_CENTER.CurveLength         = Lmax;
                  EGO_LANE_CENTER.ErrCOV              = Pfus;
               end
        end
        % ====================================================================================================================================================== 
    end
end
