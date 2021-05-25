function [output] = myFunction2(mode)    
    load_data();  
    
    switch mode
        % case 0 test the full simulation whereas case 1 and 2 test only one step. 
        case 0        
            % this simulator runs for 50 steps
            nsteps = 50;
            % the map is not given, plotting is just to tell us how
            % accurate is our solution 
            plot_map();           
            % the covariance of the measurements noise. 
            Q = diag([0.5,3*pi/180]).^2;
            % 
            mu = [];
            Sigma = [];            
            for k = 1:nsteps
                % the true pose is given
                xr          = ask_the_oracle(k);    
                plot_robot(xr)    
                % The sensor measurements to all the landmarks at the 
                % current time step. z is a matrix of shape nx2 
                % where n is the number of landmarks in the map. 
                % The first column in z is the range (m) and
                % the second column is the bearing (rad).
                z          = sense(k);
                
                if k == 1
                    % We use the sensor readings from the first time step 
                    % to initialise all the landmarks in the map. 
                    % this function takes:
                    %   z : The sensor measurements to all the landmarks 
                    %        at the current time step.
                    %   Q: the covariance of the measurements
                    %   xr: the true pose of the robot [x;y;theta].
                    % The function returns mu and Sigma after initializing
                    % all t he landmarks
                    [mu, Sigma] = initLandmarks(z,Q,xr);
                else                    
                    for i=1:length(z)
                        % i is the id of a landmark
                        % zi  is the range and bearing to landmark i
                        zi     = z(i,:);
                        
                        % this function takes:
                        %   i: id of landmark
                        %   zi: the range and nearing to landmark i
                        %   Q: the covariance of the measurements
                        %   mu,Sigma: the current estimate of the map.
                        %   xr: the true pose of the robot [x;y;theta].
                        % The function returns mu and Sigma after performing
                        % an update step using the sensor readings of
                        % landmark i                        
                        [mu, Sigma] = update_step(i,zi,Q,mu,Sigma,xr);                        
                        
                        if k == nsteps
                            lidx   = i*2 -1;
                            lSigma = Sigma(lidx:lidx+1,lidx:lidx+1);
                            plot_cov(mu(lidx:lidx+1),lSigma,3);                
                            scatter(mu(2*i-1),mu(2*i),200,'k+');                                        
                        end
                    end
                end                
            end
            output = [mu,Sigma];
        case 1
            Q = diag([0.5,3*pi/180]).^2;
            xr          = ask_the_oracle(1);                
            z          = sense(1);
            [mu, Sigma] = initLandmarks(z,Q,xr);
            output = [mu,Sigma];
        case 2
            Q = diag([0.5,3*pi/180]).^2;
            xr          = ask_the_oracle(1);                
            z          = sense(1);
            [mu, Sigma] = initLandmarks(z,Q,xr);
            
            xr          = ask_the_oracle(1);
            z          = sense(2);  
            plot_map();
            plot_robot(xr)
            for i=1:length(z)
                zi     = z(i,:);
                [mu, Sigma] = update_step(i,zi,Q,mu,Sigma,xr);
                lidx   = i*2 -1;
                lSigma = Sigma(lidx:lidx+1,lidx:lidx+1);
                plot_cov(mu(lidx:lidx+1),lSigma,3);                
                scatter(mu(2*i-1),mu(2*i),200,'k+');   
            end
            output = [mu,Sigma];            
    end    
end
% ---------------
function [mu, Sigma] = initLandmarks(z,Q,xr)
   
% initialize it
mu = [];
Sigma = [];
zpass = z;

% xr
X_Value = xr(1);
Y_Value = xr(2);
theta_val = xr(3);

    for i = 1 : length(zpass)
        M1Row1Col1 = X_Value + zpass(i,1)*cos(theta_val + zpass(i,2) );
        M1Row2Col1 = Y_Value + zpass(i,1)*sin(theta_val +zpass(i,2)) ;
        
        NewMat = [M1Row1Col1;   
                        M1Row2Col1];
                    
        mu = [mu ; NewMat];            
        
        M2Row1Col1 = cos(theta_val+zpass(i,2));
        M2Row1Col2 = -zpass(i,1)*sin(theta_val+zpass(i,2));
        M2Row2Col1 = sin(theta_val+zpass(i,2));
        M2Row2Col2 = zpass(i,1)*cos(theta_val+zpass(i,2));
        
        RotMatrix = [M2Row1Col1 ,M2Row1Col2;
            M2Row2Col1 , M2Row2Col2];
        
        diagg = blkdiag(Sigma , RotMatrix * Q * RotMatrix');
        Sigma = diagg;
        
    end
    
end 
function [mu, Sigma] = update_step(landmarkID,zi,Q,mu,Sigma,xr)
 % xr
X_Value = xr(1);
Y_Value = xr(2);
theta_val = xr(3);

    i = landmarkID * 2 - 1;
    indexing = i;
    rangePart1 = (X_Value - mu(indexing))^2 ;
    rangePart2 = (Y_Value - mu(indexing + 1))^2;
    range = sqrt(rangePart1 + rangePart2);
    bearing = atan2( mu(i+1) - Y_Value , mu(i) - X_Value ) - xr(3);
    
    error = zi - [range , bearing];
    err = error;
    err = [err(1); wrapToPi(err(2))];
    
    arrr = zeros(2, 10);
    emptyarr = arrr;
    
    M3Row1Col1 = (mu(i)-X_Value)/range;
    M3Row1Col2 = (mu(i+1)-Y_Value)/range;
    M3Row2Col1 = -(mu(i+1)-Y_Value)/range^2;
    M3Row3Col2 = (mu(i)-X_Value)/range^2;
    array2 = [M3Row1Col1 , M3Row1Col2;
              M3Row2Col1 , M3Row3Col2];
          
    
    emptyarr(: , i : i+1) = array2;
    
    G_matrix = emptyarr;
    G = G_matrix;
    
    KOne = Sigma*G';
    K1 = KOne;
    KTwo = G*Sigma*G' + Q;
    K2 = KTwo;
    K = K1/K2;

    mu = mu + K*(err);
    Sigma = (eye(10)-K*G)*Sigma;
    
end
% -----------Add your functions below this line and use them in the two functions above---
