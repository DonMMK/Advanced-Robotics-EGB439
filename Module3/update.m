function posterior_belief = update(prior_belief,signs_map,measur_prob,z)
% prior_belief: The prior belief of the robot about its location before incorporating any sensor information
% signs_map : the map of where the signs are in the hallway.
% measurements probabilities = [p(z=1|sign) , p(z=1|no_sign)]
% z is the detector output. 1 sign, 0 no_sign.
    
    VarZero = 0;
    VarOne = 1;
    Size_Signs_Map = length(signs_map);
    
    if z == VarZero
      measur_prob = VarOne - measur_prob;
    end
    
    
    nancy = ones(length(signs_map) , 1);
        for i = 1:length(signs_map)
            if signs_map(i) == VarOne
                nancy(i) = measur_prob(1) * prior_belief(i);
            else
                nancy(i) = measur_prob(2) * prior_belief(i);
            end
        end
        
    donkey = VarZero;
    for i = 1:length(signs_map)
        if signs_map(i) == VarOne
            donkey = donkey + ( measur_prob(1) * prior_belief(i) );
        else
            donkey = donkey + ( measur_prob(2) * prior_belief(i) );
        end
    end
    
    posterior_belief = nancy / donkey; 


end