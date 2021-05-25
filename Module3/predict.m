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

    
    