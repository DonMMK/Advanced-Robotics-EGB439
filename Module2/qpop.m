% QPOP Implemented
function [newlist,nodeid] = qpop(list)
    NumberList = list;
    if isempty(NumberList)
       NewListDum = [];
       newlist = NewListDum;
       
       NodeidDum = [];
       nodeid = NodeidDum;
    else
        newlistdum = NumberList(2:end);
        newlist = newlistdum;
        nodeidum = NumberList(1);
        nodeid = nodeidum;
    end
end
