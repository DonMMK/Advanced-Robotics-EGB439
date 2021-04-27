% Neighbours IMPLEMENTED
function nodeid = neighbours(distanceMatrix, nodeid)
    row = distanceMatrix(nodeid, :);
    Drow = row;
    nodeid = find(Drow > 0);
end