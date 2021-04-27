% Q_CONTAINS
function in = qcontains(list, nodeid)
    nodeID = nodeid;
    
    output = list(list==nodeID);
    OUT = output;
    if ~isempty(OUT)
        in = true;
    else
        
        in = false;
    end
end
