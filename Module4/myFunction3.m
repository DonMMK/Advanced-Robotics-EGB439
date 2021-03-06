%mode 0: runs the full simulation.
%mode 1: runs a landmarks initialisation step. 
%mode 2: runs a prediction step.
%mode 3: runs an update step.
function [output] = myFunction3(mode)  
    load_data();  
    % Initialization of the robot pose.
    mu =   [0;0;0*pi/180];
    Sigma =diag([0.1 0.1 0.1*pi/180]).^2;
    
    switch mode
        case 0        
            % this simulator runs for 100 steps
            nsteps = 100;
            % the covariance of the process and measurements noise. 
            R = diag([0.5 50*pi/180]).^2;
            Q = diag([0.5, 5*pi/180]).^2;
            % main loop
            for k = 1:nsteps
               %This function returns the distance travelled (d)
               % and the change in angle (dth) between time step k-1 and k.
               [d,dth]  = get_odom(k);    
               % complete the prediction step in the body of the function below
               [mu,Sigma] = predict_step(mu,Sigma,d,dth,R);
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
                    %   mu: contains the predicted pose of the robot
                    % The function returns mu and Sigma after initializing
                    % all the landmarks
                    [mu, Sigma] = initLandmarks(z,Q,mu,Sigma);             
                else                    
                   for i=1:length(z)
                        % i is the id of a landmark
                        % zi  is the range and bearing to landmark i
                        zi     = z(i,:);
                        
                        % this function takes:
                        %   i: id of landmark
                        %   zi: the range and nearing to landmark i
                        %   Q: the covariance of the measurements
                        %   mu,Sigma: the current estimate of the pose of the robot and the map.
                        %   
                        % The function returns mu and Sigma after performing
                        % an update step using the sensor readings of
                        % landmark i                        
                        [mu, Sigma] = update_step(i,zi,Q,mu,Sigma);                        

                   end

                end   
                %******************************* the code below is just for plotting  
                if k == nsteps
                    grid on; axis equal;
                    axis([-2 5 -2 5]);
                    hold on
                    plot_robot(mu)
                    plot_cov(mu,Sigma,3,'b')
                    for i=1:length(z)
                        lidx = 3+2*i;
                        li = mu(lidx-1:lidx);
                        scatter(li(1),li(2),'b+');
                        lSigma = Sigma(lidx-1:lidx,lidx-1:lidx);
                        plot_cov(mu(lidx-1:lidx),lSigma,3,'g');
                    end
                end
            end
            plot_map();             
            output = [mu,Sigma];
        case 1
            Q = diag([0.5, 5*pi/180]).^2;                           
            z          = sense(1);
            [mu, Sigma] = initLandmarks(z,Q,mu,Sigma);
            output = [mu,Sigma];
        case 2            
            R = diag([0.5 50*pi/180]).^2;
            [d,dth]  = get_odom(1);    
            [mu,Sigma] = predict_step(mu,Sigma,d,dth,R);
            output = [mu,Sigma];
        case 3
            Q = diag([0.5,3*pi/180]).^2;
            R = diag([0.5 50*pi/180]).^2;
            [d,dth]  = get_odom(1);    
            [mu,Sigma] = predict_step(mu,Sigma,d,dth,R);
            z          = sense(1);
            [mu, Sigma] = initLandmarks(z,Q,mu,Sigma);            
            
            [d,dth]  = get_odom(2);    
            [mu,Sigma] = predict_step(mu,Sigma,d,dth,R);
            z          = sense(2);
            
            grid on; axis equal;
            axis([-2 5 -2 5]);
            hold on
            plot_robot(mu)
            plot_cov(mu,Sigma,3,'b')
            for i=1:length(z)
                zi     = z(i,:);  
                [mu, Sigma] = update_step(i,zi,Q,mu,Sigma);
                lidx   = 3 + i*2 -1;
                lSigma = Sigma(lidx:lidx+1,lidx:lidx+1);
                plot_cov(mu(lidx:lidx+1),lSigma,3,'g');                
                scatter(mu(lidx),mu(lidx+1),50,'b+');   
            end
            output = [mu,Sigma];            
    end    
end


% --------------- complete the three functions below


% This function takes:
%     mu,Sigma: the current estimate of the pose of the robot and the map.
%     as well as the odometry information (d the distance travelled from time step k-1 and k and dth, the change of heading) 
%     and the matrix R (the covariance of the odometry noise). 
% The function performs a prediction step of the EKF localiser and returns the mean and covariance of the robot pose and the map.   
% note: although the prediction step does not change the estimation 
%      of the landmarks in the map, this function accepts the full state space
%      and only alter the pose of the robot in it.
function [mu,Sigma] =predict_step(mu,Sigma,d,dth,R)
  
    Matrix1Row1Col1 = d*cos(mu(3)) ;
    Matrix1Row2Col1 = d*sin(mu(3)) ;
    Matrix1Row3Col1 = dth;
    
    mu(1:3) = mu(1:3) + [Matrix1Row1Col1;Matrix1Row2Col1;Matrix1Row3Col1];
    
    Var_One = 1;
    Var_Zero = 0;
    
    Mat2R1C3 = -d*sin(mu(3));
    Mat2R2C3 = d*cos(mu(3));
    Jacobian_X = [1, 0, Mat2R1C3;
                  0, 1, Mat2R2C3;   
                  0, 1, 1];
              
    Mat3R1C1 = cos(mu(3));
    Mat3R2C1 = sin(mu(3));
    Jacobian_U = [ Mat3R1C1, 0;
                   Mat3R2C1, 0;
                   0       , 1];
               
    if size(Sigma) == [3 3]
        Sigma_P1 = Jacobian_X * Sigma * Jacobian_X';
        Sigma_P2 = Jacobian_U *R * Jacobian_U';
        Sigma = Sigma_P1 + Sigma_P2;
    else
        Jacobian_X = blkdiag(Jacobian_X,eye(20));
        Jacobian_U = [Jacobian_U;zeros(20,2)];
        Sigma2_P1 = Jacobian_X*Sigma*Jacobian_X';
        Sigma2_P2 = Jacobian_U*R*Jacobian_U'; 
        Sigma = Sigma2_P1 + Sigma2_P2;
    end
    
end
% We use the sensor readings from the first time step 
% to initialise all the landmarks in the map. 
% this function takes:
%   z : The sensor measurements to all the landmarks 
%        at the current time step.
%   Q: the covariance of the measurements
%   mu,Sigma: the current estimate of the robot pose and the map (the map will be empty so the size of mu is 3x1 and Sigma 3x3).
% The function returns mu and Sigma after initialising (if n is the number of landmarks, the function returns mu of size (3+2n)x1 and Sigma of size (3+2n)x(3+2xn))
% all t he landmarks
function [mu, Sigma] = initLandmarks(z,Q,mu,Sigma)

    ArrayStore = [];
    ArrayStoreMU = [];
    
    for i = 1 : length(z)
        M4Part1 = mu(1) + z(i,1)*cos(mu(3)+z(i,2));
        M4Part2 = mu(2) + z(i,1)*sin(mu(3)+z(i,2)); 
        
        NewArray = [ M4Part1; M4Part2 ];
        
        ArrayStoreMU = [ArrayStoreMU; NewArray];
        
        M5R1C1 = cos(mu(3)+z(i,2));
        M5R1C2 = -z(i,1)*sin(mu(3)+z(i,2));
        M5R2C1 = sin(mu(3)+z(i,2));
        M5R2C2 = z(i,1)*cos(mu(3)+z(i,2));
        
        MatrixLz = [M5R1C1 , M5R1C2 ; M5R2C1 , M5R2C2];
        
        ArrayStore = blkdiag(ArrayStore,MatrixLz*Q*MatrixLz');
        
    end
    
    
    if size(mu)  == [3 1]
        TempMU = ArrayStoreMU;
        mu = [mu;TempMU];
    else
        TempMU = ArrayStoreMU;
        mu(4:end) = TempMU;
    end

    if size(Sigma) == [3 3]
        PassSig = blkdiag(Sigma,ArrayStore);
        Sigma = PassSig;
    else
        Sigma(4:end,4:end) = ArrayStore;
        
    end       
    
end

% this function takes:
    %  landmarkID: id of a landmark
    %   zi: the range and nearing to this landmark 
    %   Q: the covariance of the measurements
    %   mu,Sigma: the current estimate of the robot pose and the map.
    %   
    % The function returns mu and Sigma after performing
    % an update step using the sensor readings of
    % the landmark   
function [mu, Sigma] = update_step(landmarkID,zi,Q,mu,Sigma)

    X_Value = mu(1);
    Y_Value = mu(2);
    i = landmarkID * 2 - 1 + 3;
    indexing = i;
    rangePart1 = (X_Value - mu(indexing))^2 ;
    rangePart2 = (Y_Value - mu(indexing + 1))^2; 
    range = sqrt(rangePart1 + rangePart2); 
    bearing = atan2( mu(i+1) - Y_Value , mu(i) - X_Value ) - mu(3);
    
    error = zi - [range , bearing];
    err = error;
    err = [err(1); wrapToPi(err(2))];
    
    M3Row1Col1 = -(mu(i)-X_Value)/range;
    M3Row1Col2 = -(mu(i+1)-Y_Value)/range;
    M3Row2Col1 = (mu(i+1)-Y_Value)/range^2;
    M3Row3Col2 = -(mu(i)-X_Value)/range^2;
    Garray = [M3Row1Col1 , M3Row1Col2 , 0;
              M3Row2Col1 , M3Row3Col2 , -1];
          
    ZeroG = zeros(2 ,20);
    G = [Garray , ZeroG];
    
    
    G3R1C1 = (mu(i) - X_Value)/range;
    G3R1C2 = (mu(i+1)- Y_Value)/range;
    G3R2C1 = -(mu(i+1)- Y_Value)/range^2;
    G3R2C2 = (mu(i)- X_Value)/range^2 ;
    GLast = [G3R1C1 ,G3R1C2;
              G3R2C1, G3R2C2 ];
          
    G(:,i:i+1) = GLast;
          
    K1 = Sigma*G';
    K11 = K1;
    K2 = G*Sigma*G' + Q;
    K22 = K2;
    K = K11/K22;

    mu = mu + K*(err);
    Sigma = (eye(23)-K*G)*Sigma;
    
    
    
end


% ----------------------------
% write the extra functions that you need and call them in the three functions above




