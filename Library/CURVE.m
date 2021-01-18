% --------------------------------------------------------------------------------------------------------------------------------------------------------------
% Author Name : Udit Bhaskar Talukdar
% Date        : 01/01/2021
% Description : Curve Geometry computation library , Contains the following functions :
%             : ClothoidToCartesianNumericalIntegration() : Generate (x, y) coordinates in cartesian from a clothoid equation by numerical integration
%             : ClothoidToCartTaylorApprox()              : Generate (x, y) coordinates in cartesian from a clothoid equation by 3rd order taylor series approximation
%             : ClothoidToCartSmallAngleApprox()          : Generate (x, y) coordinates in cartesian from a clothoid equation by small angle approximation
%             : ClothoidTranslation()                     : Compute the coefficients of a parallel clothoid
%             : Polynomial2Clothoid()                     : Compute the clothoid coefficients from polynomial coefficients
%             : Polynomial2Clothoid_v2()                  : Compute the clothoid coefficients from polynomial coefficients
%             : LeastSquaresFit3rdDegreePolynomial()      : 3rd Order polynimial curve fitting by least squares estimates
% --------------------------------------------------------------------------------------------------------------------------------------------------------------
classdef CURVE
    methods(Static)
        % ======================================================================================================================================================
        function [X, Y, phi_n, K_n] = ClothoidToCartesianNumericalIntegration(x0, y0, phi0, K0, K_hat, s, nPts)
            % Generate (x, y) coordinates in cartesian from a clothoid equation
            % INPUT : x0, y0 : initial coordinates in meters
            %           phi0 : initial heading
            %             K0 : initial curvature
            %          K_hat : curvature rate
            %              s : curv length
            %           nPts : number of points
            % OUTPUT :     X : a vector of x coordinate
            %              Y : a vector of Y coordinate
            %          phi_n : a vector of tangent angle
            %            K_n : a vector of curvature
            % --------------------------------------------------------------------------------------------------------------------------------------------------
            dL = s/(nPts-1);   % compute the small arc length given the number of points
            L_n = 0:dL:s;      % compute a vector of arc lengths
            K_n = K0 + K_hat.*L_n;    % compute a vector of curvature at different arc lengths
            phi_n = phi0 + K0.*L_n + 0.5*K_hat.*L_n.^2;  % compute a vector of heading angles at different arc lengths
            X = x0 + cumsum(cos(phi_n))*dL;  % compute a vector of x coordinates at different arc lengths
            Y = y0 + cumsum(sin(phi_n))*dL;  % cmpute a vector of y coordinates at different arc lengths
        end
        % ======================================================================================================================================================
        function [Xs, Ys, phi_s, Ks] = ClothoidToCartTaylorApprox(x0, y0, phi0, K0, K_hat, s, L)
		    % Generate (x, y) coordinates in cartesian from a clothoid equation by 3rd order taylor series approximation
            % INPUT : x0, y0 : initial coordinates in meters
            %           phi0 : initial heading
            %             K0 : initial curvature
            %          K_hat : curvature rate
            %              L : curv length
			%              s : interpolated curve lengths
            % OUTPUT :    Xs : a vector of x coordinate
            %             Ys : a vector of Y coordinate
            %          phi_s : a vector of tangent angle
            %            K_s : a vector of curvature
            % --------------------------------------------------------------------------------------------------------------------------------------------------
            Ks = K0 + K_hat.*s;
            phi_s = phi0 + K0.*s + 0.5*K_hat.*s.^2;
    
            L1 = L/2; L2 = L1^2; L3 = L1^3;
            S = sin(phi0 + K0*L1 + 0.5*K_hat*L2);
            C = cos(phi0 + K0*L1 + 0.5*K_hat*L2);
            K = K0 + K_hat*L1;
     
            X1 = S*K;
            X2 = 0.5*(C*K^2 + K_hat*S);
            X3 = C*K^3*0.1667 - K_hat*C*K*0.5;
            Xs = x0 + (C + L1*X1 - L2*X2 -     L3*X3).*s ...
                    + (  -0.5*X1 + L1*X2 + 1.5*L2*X3).*s.^2 ...
                    + (       -0.3333*X2 -     L1*X3).*s.^3 ...
                    + (                      0.25*X3).*s.^4;
     
            Y1 = C*K;
            Y2 = 0.5*(S*K^2 - K_hat*C);
            Y3 = C*K^3*0.1667 + K_hat*S*K*0.5;
            Ys = y0 + (S - L1*Y1 - L2*Y2 +     L3*Y3).*s ...
                    + (   0.5*Y1 + L1*Y2 - 1.5*L2*Y3).*s.^2 ...
                    + (       -0.3333*Y2 +     L1*Y3).*s.^3 ...
                    + (                  -   0.25*Y3).*s.^4;
        end
        % ======================================================================================================================================================
        function [Xs, Ys, phi_s, Ks] = ClothoidToCartSmallAngleApprox(x0, y0, phi0, K0, K_hat, s)
            % Generate (x, y) coordinates in cartesian from a clothoid equation by small angle approximation
            % INPUT : x0, y0 : initial coordinates in meters
            %           phi0 : initial heading
            %             K0 : initial curvature
            %          K_hat : curvature rate
            %              s : curv length(s)
            % OUTPUT :    Xs : a vector of x coordinate
            %             Ys : a vector of Y coordinate
            %          phi_s : a vector of tangent angle
            %            K_s : a vector of curvature
            % --------------------------------------------------------------------------------------------------------------------------------------------------
            Ks = K0 + K_hat.*s;
            phi_s = phi0 + K0.*s + 0.5*K_hat.*s.^2;
            Xs = x0 + s;
            Ys = y0 + phi0.*s + 0.5*K0.*s.^2 + 0.1667*K_hat.*s.^3;
        end
        % ======================================================================================================================================================
        function [x0_t, y0_t, phi_t, K_t, K_hat_t, L_t] = ClothoidTranslation(x0, y0, phi0, K0, K_hat, L, d)
            % Compute the coefficients of a parallel clothoid
            % INPUTS : x0, y0 : initial coordinates in meters
            %           phi0 : initial heading
            %             K0 : initial curvature
            %          K_hat : curvature rate
            %              L : curv length
            %              d : parallel curve offset
            % OUTPUT :  x0_t : translated init x0
            %           y0_t : translated init y0
            %          phi_t : translated init heading
            %            K_t : translated init curvature
            %        K_hat_t : translated init curvature
            %            L_t : Length of the translated curve
            % -------------------------------------------------------------------------------------------------------------------------------------------------- 
            R0_t = 1/K0 + d;        % Radius at the start
            K0_t = 1/R0_t;          % curvature at the start
            R1_t = 1/(K0 + K_hat * L) + d;  % Radius at the end
            K1_t = 1/R1_t;                  % curvature end
     
            phi_t = phi0; % tangent start
            phi_e_t = phi0 + K0*L + 0.5*K_hat*L^2; % tangent end
            L_t = L + phi_e_t * d;  % length of the translated curve
     
            fact = 0.5*( (2*phi_e_t/L_t) - K0_t - K1_t );  % Correction factor
            K0_t = K0_t + fact; % corrected curvature start
            K1_t = K1_t + fact; % corrected curvature end
     
            K_t = K0_t;
            K_hat_t = (K1_t - K0_t)/L_t;   % curvature rate
            x0_t = x0 + d*sin(phi_t);
            y0_t = y0 - d*cos(phi_t);
        end
        % ======================================================================================================================================================
        function [x0, y0, phi0, K0, K_hat, L] = Polynomial2Clothoid_v2(a0, a1, a2, a3, xInit, xEnd, nPts)
            % Compute the clothoid coefficients from polynomial coefficients
            % INPUTS : a0, a1, a2, a3 : coefficients of a 3rd order polynomial
            %          Xmax : upper limit of the polynomial
            %             L : Length of the polynomial
            % OUTPUTS :  X0 : 
            %            Y0 : 
            %           phi : 
            %             K :
            %         K_hat : 
            % --------------------------------------------------------------------------------------------------------------------------------------------------
            dx = (xEnd - xInit)/(nPts - 1); 
            X = xInit:dx:xEnd;
            Y = a0 + a1.*X + a2.*X.^2 + a3.*X.^3;
            dS = zeros(1,length(Y)); dS(1,1) = 0;
            dS(1,2:end) = sqrt((X(1,2:end) - X(1,1:end-1)).^2 + (Y(1,2:end) - Y(1,1:end-1)).^2);
            S = cumsum(dS);
            y_hat = a1 + 2*a2*X + 3*a3*X.^2;           
            y_hat_hat = 2*a2 + 6*a3*X;
            phi = atan(y_hat);                               
            K = ( y_hat_hat )./( (1 + y_hat.^2).^(3/2) );   
            Khats = (K(1,2:end) - K(1,1))./S(1,2:end);
            wts = dS(1,2:end)/S(1,end);
            x0 = xInit; 
            y0 = Y(1,1); 
            phi0 = phi(1,1);
            K0 = K(1,1);
            K_hat = wts * Khats';
            L = S(1,end);
        end
        % ======================================================================================================================================================
        function [x0, y0, phi0, K0, K_hat, L] = Polynomial2Clothoid(a0, a1, a2, a3, xInit, xEnd, nPts)
            % Compute the clothoid coefficients from polynomial coefficients
            % INPUTS : a0, a1, a2, a3 : coefficients of a 3rd order polynomial
            %          Xmax : upper limit of the polynomial
            %             L : Length of the polynomial
            % OUTPUTS :  X0 : 
            %            Y0 : 
            %           phi : 
            %             K :
            %         K_hat : 
            % --------------------------------------------------------------------------------------------------------------------------------------------------
            dx = (xEnd - xInit)/(nPts - 1);
            xx = xInit:dx:xEnd;
            yy = a0 + a1.*xx + a2.*xx.^2 + a3.*xx.^3;
            L = sum(sqrt((xx(2:end) - xx(1:end-1)).^2 + (yy(2:end) - yy(1:end-1)).^2));
            x0 = xInit;
            y0 = a0 + a1*x0 + a2*x0^2 + a3*x0^3;
            y_hat_0 = a1 + 2*a2*x0 + 3*a3*x0^2;
            y_hat_hat_0 = 2*a2 + 6*a3*x0;
            phi0 = atan(y_hat_0);
            K0 = ( y_hat_hat_0 )/( (1 + y_hat_0^2)^(3/2) );
            y_hat_1 = a1 + 2*a2*xEnd + 3*a3*xEnd^2;
            y_hat_hat_1 = 2*a2 + 6*a3*xEnd;
            K1 = ( y_hat_hat_1 )/( (1 + y_hat_1^2)^(3/2) );
            K_hat = (K1 - K0)/L;
        end
        % ======================================================================================================================================================
        function PolyCoeff = LeastSquaresFit3rdDegreePolynomial(Xin, Yin)
		    % 3rd Order polynimial curve fitting by least squares estimates
            % INPUTS  : (Xin, Yin) : Vector of X and Y coordinates of the points
            % OUTPUTS : PolyCoeff  : a vector of polynimial coefficients
            % --------------------------------------------------------------------------------------------------------------------------------------------------
            Y = Yin(:);  % create a column vector 
            X = zeros(length(Xin), 4);
            for i = 1:length(Xin)
                X(i,1) = 1; X(i,2) = Xin(i)^1; X(i,3) = Xin(i)^2; X(i,4) = Xin(i)^3;
            end
            PolyCoeff = (pinv(X'*X))*(X'*Y); %output coefficients
        end
        % ======================================================================================================================================================
    end
end