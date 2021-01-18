classdef VISUALIZE
    methods(Static)
        % ======================================================================================================================================================
        % display measurements
        function displayMeas(LANE_LINE_MEAS_MAT, XLimit, YLimit)
            for idx = 1:LANE_LINE_MEAS_MAT.ValidCumulativeMeasCount(1,end)
                x0 = 0; y0 = LANE_LINE_MEAS_MAT.MeasArray(1,idx); a0 = LANE_LINE_MEAS_MAT.MeasArray(2,idx);
                K0 = LANE_LINE_MEAS_MAT.MeasArray(3,idx); Khat = LANE_LINE_MEAS_MAT.MeasArray(4,idx); L = LANE_LINE_MEAS_MAT.MeasArray(5,idx);
                nPts = 100; sMax = L; sMin = 0; ds = (sMax - sMin)/(nPts - 1); s = sMin:ds:sMax;
                [x, y, ~, ~] = CURVE.ClothoidToCartTaylorApprox(x0, y0, a0, K0, Khat, s, L);
                plot(x, y, '.', 'color', 'r');axis equal;hold on;
                set(gca,'XLim',XLimit); set(gca,'YLim',YLimit)
            end
            set(gca,'XLim',XLimit); set(gca,'YLim',YLimit);
        end
        % ======================================================================================================================================================
        function displayRoad(points, color, marker, XLimit, YLimit)
            validIdx = points.BufferLastValidIdx;
            x = points.X(1,1:validIdx); y = points.Y(1,1:validIdx); 
            plot(x, y, marker, 'color', color);axis equal;hold on;
            set(gca,'XLim',XLimit); set(gca,'YLim',YLimit);
        end
        % ======================================================================================================================================================
        function displayLane(ClothoidCoeff, color, marker, XLimit, YLimit)
            x0 = 0; y0 = ClothoidCoeff.LateralOffset; 
            a0 = ClothoidCoeff.HeadingAngle; 
            K0 = ClothoidCoeff.Curvature; 
            Khat = ClothoidCoeff.CurvatureDerivative; 
            L = ClothoidCoeff.CurveLength;
            nPts = 1000; sMax = L; sMin = 0; ds = (sMax - sMin)/(nPts - 1); s = sMin:ds:sMax;
            [x, y, ~, ~] = CURVE.ClothoidToCartTaylorApprox(x0, y0, a0, K0, Khat, s, L);
            plot(x, y, marker, 'color', color);axis equal;hold on;
            set(gca,'XLim',XLimit); set(gca,'YLim',YLimit);
        end
        % ======================================================================================================================================================
        function VisualizeCameraLayout(FOVPtsEGOframeCAM, nCameras)
            %camColors = ['b', 'o', 'c', 'o', 'b', 'o', 'c', 'o'];
            camColors = ['b', 'm', 'm', 'm', 'b', 'm', 'm', 'm'];
            VehDim = [3.7, 3.7, -1, -1; 0.9, -0.9, -0.9, 0.9];
            figure(1)
            opaque = 0.05;
            fill(VehDim(1,:), VehDim(2,:), 'k', 'edgecolor','none', 'facealpha',1);hold on
            for idx = 1:nCameras
                fill(FOVPtsEGOframeCAM(idx).Xcoord, FOVPtsEGOframeCAM(idx).Ycoord, camColors(idx), 'edgecolor','none', 'facealpha',opaque);axis equal;hold on;grid on
                line(FOVPtsEGOframeCAM(idx).Xcoord, FOVPtsEGOframeCAM(idx).Ycoord,'color', camColors(idx),'linewidth',0.1); % Make the edges...
            end
            set(gca,'XLim',[-160 210])
            set(gca,'YLim',[-110 110])
            hold off;
        end
        % ==============================================================================================================================================================
    end
end