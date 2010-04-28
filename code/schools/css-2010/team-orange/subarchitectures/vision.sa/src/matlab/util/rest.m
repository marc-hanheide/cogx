function idxs2=rest(idxs,idxs1)
%REST
%idxs2=rest(allIdxs,idxs1);

idxs2=idxs(~ismember(idxs,idxs1));