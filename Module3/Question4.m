% measurements probabilities = [p(z=1|sign) , p(z=1|no_sign)]
measur_prob = [0.65,0.1];
% U: the motion probability [p(C2|C1,U=1), P(C1|C1,U=1)]
U = [0.9 0.1];
% signs_map : the map of where the signs are in the grid
signs_map = [0 , 1 , 0 , 0 , 1 , 0 , 0 , 0 , 0 , 0];
prior_belief   =  ones(1,length(signs_map) ) ./ length(signs_map);

for t=1:15
    % at each time step, the robot is commanded to move one cell forward
    % update the robot's belief about its location after motion
    robot_belief_after_motion = predict(prior_belief,U);
    
    % after motion, the robot sense the sign
    % z is the sign detector output. 1 sign, 0 no_sign
    z = sense_sign(t);
    % update the robot's belief about its location after sensing  
    
    posterior_belief = update(robot_belief_after_motion,signs_map,measur_prob,z)';
    
    % calcs
    prior_belief = posterior_belief;
end

for c =1:length(posterior_belief)
    fprintf('The probability that the robot is in cell %d is %0.2f%s\n',c,posterior_belief(c)*100,'%')
end

function robot_belief_after_motion = predict(robot_belief_before_motion,U)
% inputs: robot_belief_before_motion is the probability distribution of the robot's location over the cells
%       : U is the motion probability [p(C2|C1,U=1), P(C1|C1,U=1)]
% output: robot_belief_after_motion is the probability distribution of the robot's location over the cells after executing the motions
    
    prepath = robot_belief_before_motion;
    Numericc = prepath;
    Probability = U;
    length_of_robot = 10;
    
    for count = 1: length_of_robot
    
        if ( (count - 1) < 1 )
        Part1 = ( Numericc(count) * Probability(2) );
        Part2 = (Numericc(length(Numericc)) * Probability(1) );
        prepath(count) = Part1 + Part2;
        else
            PPart1 = (Numericc(count)*Probability(2));
            PPart2 = (Numericc(count-1)*Probability(1));
            prepath(count) = PPart1 + PPart2;
        end    
        if (count - floor(count/length_of_robot)) * length_of_robot ==0
            Numericc = prepath;
        end
    end
    robot_belief_after_motion = prepath;



end

function posterior_belief = update(robot_belief_after_motion,signs_map,measur_prob,z)
% signs_map : the map of where the signs are in the grid
% measur_prob: measurements probabilities = [p(z=1|sign) , p(z=1|no_sign)] 
% z is the detector output. 1 sign, 0 no_sign

    VarZero = 0;
    VarOne = 1;
    Size_Signs_Map = length(signs_map);
    
    if z == VarZero
      measur_prob = VarOne - measur_prob;
    end
    
    
    nancy = ones(length(signs_map) , 1);
        for i = 1:length(signs_map)
            if signs_map(i) == VarOne
                nancy(i) = measur_prob(1) * robot_belief_after_motion(i);
            else
                nancy(i) = measur_prob(2) * robot_belief_after_motion(i);
            end
        end
        
    donkey = VarZero;
    for i = 1:length(signs_map)
        if signs_map(i) == VarOne
            donkey = donkey + ( measur_prob(1) * robot_belief_after_motion(i) );
        else
            donkey = donkey + ( measur_prob(2) * robot_belief_after_motion(i) );
        end
    end
    
    posterior_belief = nancy / donkey; 

end
