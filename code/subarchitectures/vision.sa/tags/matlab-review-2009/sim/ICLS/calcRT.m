function rt=calcRT(rs,tnq)
%rt=calcRT(rs,tnq)
%calculate racionality/effectiveness

rt=rs./sqrt(tnq+1);

rt=rs./(tnq+1);

rt=rs./log(tnq+200);

rt=rs./log(tnq+max(rt(:))/max(tnq(:)));
