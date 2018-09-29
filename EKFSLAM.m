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

mappedL = [];

while (run)
    
    % Update true pose
    xt = xt + tspeed*dT;
    
    % Generate measurements
    rho = sqrt((xL-xt).^2+yL.^2) + rhoStd*randn(1,NL);
    phi = atan2(yL,xL-xt) + phiStd*randn(1,NL);
    
    if forceUpdate == 1 || superGPS == 1 || abs(tspeed) > 1e-6
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Predict motion
        dnoise = (tspeed*dT*tdStd)*randn;
        
        X(1) = X(1) + (tspeed*dT+dnoise);
        
        D = tspeed*dT;

        A = eye(length(X),length(X));
        W = zeros(length(X),1);
        A(1,1) = 1;
        W(1,1) = 1;
        Q = diag([(D*tdStd)^2]);

        P = A*P*A' + W*Q*W';
        
        H=[];
        R = [];
        innov = [];
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Measurement update
        
        if forceUpdate == 1 || abs(tspeed) > 1e-6
            
            if forceUpdate == 1
                disp('Asked to force an update')
            end
            forceUpdate = 0;
            
            
            % Loop over the measurements
            for l = 1:NL
                
                % Check if this landmark is used or not
                if lMask(l) == 0
                    continue
                end
                
                %disp(sprintf('Updating with measurement of landmark %d',l))
                
                xpos = find(mappedL==l);
                
                if isempty(xpos)
                    % First time we see this landmark. Need to extend state
                    % vector
                    mappedL = [mappedL l];
                    xpos = length(X) + 1;
                    X(end+1,1) = X(1,1) + rho(l)*cos(phi(l));
                    X(end+1,1) = 0 + rho(l)*sin(phi(l));
                    P = [P zeros(size(P,1),2);
                        zeros(2,size(P,2)) 1e10*eye(2)];
                else
                    xpos = 2 * xpos;
                end
                
                zRho = sqrt((X(xpos)-X(1)).^2+X(xpos+1).^2);
                
                % Use range measurement
                if zRhoStd > 0
                    Hnew = zeros(1, length(X));
                    Hnew(1) = [-(X(xpos) - X(1))/zRho];
                    if ~isempty(mappedL)
                        Hnew(xpos) = [(X(xpos) - X(1))/zRho];
                        Hnew(xpos+1) = [X(xpos+1)/zRho];
                    end
                    if ~isempty(H) && size(Hnew,2) > size(H,2)
                        H = [H zeros(size(H,1), 2)];
                    end
                    H = [H; Hnew];
                    innov = [innov; rho(l) - zRho];
                    R = [R zRhoStd^2];
                end
                
                % Use bearing measurement
                if zPhiStd > 0
                    zPhi = atan2(X(xpos+1),X(xpos)-X(1));
                    dPhi = phi(l) - zPhi;
                    while dPhi > pi
                        dPhi = dPhi - 2.0*pi;
                    end
                    while dPhi < -pi
                        dPhi = dPhi + 2.0*pi;
                    end
                    Hnew = zeros(1, length(X));
                    Hnew(1) = yL(l)/zRho^2;
                    
                    if ~isempty(mappedL)
                        Hnew(xpos) = -X(xpos+1)/zRho^2;
                        Hnew(xpos+1) = (X(xpos)-X(1))/zRho^2;
                    end
                    if ~isempty(H) && size(Hnew,2) > size(H,2)
                        H = [H zeros(size(H,1), 2)];
                    end
                    H = [H; Hnew];
                    innov = [innov; dPhi];
                    R = [R zPhiStd^2];
                end
            end
        end
        
        if superGPS
            superGPS = 0;
            Hnew = zeros(1, length(X));
            Hnew(1) = 1;
            if ~isempty(H) && size(Hnew,2) > size(H,2)
                H = [H zeros(size(H,1), 2)];
            end
            H = [H; Hnew];
            innov = [innov; xt - X(1)];
            R = [R 1e-6];
        end
        
        if ~isempty(innov)
            % Turn our diagonal vector with variances into a matrix
            R = diag(R);
            
            K = P*H'*inv(H*P*H' + R);
            X = X + K*innov;
            P = (eye(size(P,1), size(P,2)) - K*H)*P;
            %det(P)
        end
        
    end
    
    if reset
        xt=0;
        X = X0;
        P = P0;
        mappedL = [];
        reset = 0;
    end
    
    if addDisturbance
        xt = xt + 0.25*rand(1,1);
        addDisturbance = 0;
    end
    
    if injectNoise
        injectNoise = 0;
        P(1) = P(1) + 0.1;
    end
    
    if ~isempty(h)
        delete(h);
    end
    hold on
    h = [plot(X(1),0,'bx') display_robot(xt,'k')];
    
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
        % Display Gaussian uncertainty for robot
        xsig = sqrt(P(1));
        xgauss = linspace(X(1)-5*xsig, X(1)+5*xsig, 100);
        ygauss = exp(-0.5*((xgauss-X(1))/xsig).^2)/xsig/sqrt(2*pi);
        h = [h plot(xgauss, 0.01+ygauss, 'b')];
        
        for k=1:length(mappedL)
            xl = X((2*k):(2*k+1));
            Pl = P((2*k):(2*k+1),(2*k):(2*k+1));
            %det(Pl)
            h = [h plot_2dgauss(xl, Pl, 'b', true)];
        end
    else
        if ~isempty(mappedL)
            for k=1:length(mappedL)
                h = [h plot(X(2*k),X(2*k+1),'or')];
            end
        end
    end
    
    hold off
    
    axis([-2 12 0 14])
    
    drawnow;
    
end
