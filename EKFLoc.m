%
% Author Patric Jensfelt
%
clear
close all
figure(1)

% Load simulation parameters
simulation_parameters;

dispGaussApprox = 1;

% Create the UI
create_ui

% Display handles to
h = [];

% Plot the landmarks
plot(xL,yL,'ko','MarkerSize',5)
text(xL(1), yL(1)+0.3, '1')
text(xL(2), yL(2)+0.3, '2')
text(xL(3), yL(3)+0.3, '3')
text(xL(4), yL(4)+0.3, '4')

xt=0;
X0 = zeros(1,1);
P0 = 1e-6*eye(1);
X = X0;
P = P0;

firstIter = 0;

while (run)
    
    % Update true pose
    xt = xt + tspeed*dT;
    
    % Generate measurements
    rho = sqrt((xL-xt).^2+yL.^2) + rhoStd*randn(1,NL);
    phi = atan2(yL,xL-xt) + phiStd*randn(1,NL);
    
    if forceUpdate == 1 || abs(tspeed) > 1e-6
        
        if forceUpdate == 1
            disp('Asked to force an update')
        end
        forceUpdate = 0;
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Predict motion
        dnoise = (tspeed*dT*tdStd)*randn;
        
        X(1) = X(1) + (tspeed*dT+dnoise);
                
        D = tspeed*dT;
        
        A = [1];
        
        W = [1];

        Q = diag([(D*tdStd)^2]);
        
        P = A*P*A' + W*Q*W';
        
        H=[];
        R = [];
        innov = [];
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Measurement update
        
        
        % Loop over the measurements
        for l = 1:NL
            
            % Check if this landmark is used or not
            if lMask(l) == 0
                continue
            end
            
            zRho = sqrt((xL(l)-X(1)).^2+(yL(l)).^2);
            
            % Use range measurement
            if zRhoStd > 0
                H = [H; [-(xL(l) - X(1))/zRho]];
                innov = [innov; rho(l) - zRho];
                R = [R zRhoStd^2];
            end
            
            % Use bearing measurement
            if zPhiStd > 0
                zPhi = atan2(yL(l),xL(l)-X(1));
                dPhi = phi(l) - zPhi;
                while dPhi > pi
                    dPhi = dPhi - 2.0*pi;
                end
                while dPhi < -pi
                    dPhi = dPhi + 2.0*pi;
                end
                H = [H; [ (yL(l))/zRho^2 ]];
                innov = [innov; dPhi];
                R = [R zPhiStd^2];
            end
        end
        
        if ~isempty(innov)
            % Turn our diagonal vector with variances into a matrix
            R = diag(R);
            
            K = P*H'*inv(H*P*H' + R);
            X = X + K*innov;
            P = (eye(1) - K*H)*P;
            
        end
        
    end
    
    if reset
        xt=0;
        X = X0;
        P = P0;
        reset = 0;
    end
    
    if setUniform
        X = zeros(1,1);
        P = 1e4*eye(1);
        setUniform = 0;
    end
    
    if addDisturbance
        xt = xt + 0.25*rand(1,1);
        addDisturbance = 0;
    end
    
    if ~isempty(h)
        delete(h);
    end
    hold on
    h = [plot(X(1,:),0,'bx') display_robot(xt,'k')];

    % Display Gaussian uncertainty for robot
    xsig = sqrt(P(1));
    xgauss = linspace(X(1)-5*xsig, X(1)+5*xsig, 100);
    ygauss = exp(-0.5*((xgauss-X(1))/xsig).^2)/xsig/sqrt(2*pi);
    h = [h plot(xgauss, 0.01+ygauss, 'b')];
    
    if zRhoStd > 0 || zPhiStd > 0
        for k=1:NL
            
            % Check if this landmark is used or not
            if lMask(k) == 0
                continue
            end
            
            h = [h plot([xt xt+rho(k)*cos(phi(k))],[0 rho(k)*sin(phi(k))],'m')];
        end
    end
    
    if dispGaussApprox
        %h = [h plot_2dgauss(X(1:2), P(1:2,1:2), 'b', true)];
        dirLen = 0.5;
    end
    
    hold off
    
    axis([-2 12 0 14])
    
    drawnow;
    
end





