function h = CalcTheGoal(placeCoords,n,goalNode)

    current_node = placeCoords(:,n);
    goal_node = placeCoords(:,goalNode);
    
    inside_bracket = current_node - goal_node;
    inside_inside = sum((inside_bracket).^2);
    h = sqrt(inside_inside)/1e5;

      
end 