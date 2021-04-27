% Q_INSERT IMPLEMENTED
function [newlist,nodeid] = qinsert(list, nodeid, cost)
    inlist = list;
    nodein = nodeid;
    listin = [inlist nodein];
    list = listin;
    temp_cost = cost(list);
    tempnotrans = [list; temp_cost];
    temp_matrix = tempnotrans' ;
    temp_matrix = sortrows(temp_matrix,2);
    
    newlist = temp_matrix(:,1)';
    
end
