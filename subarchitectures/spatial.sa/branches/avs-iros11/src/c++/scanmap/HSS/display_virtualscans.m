%d = load('dumbo-virtual360-scans.txt');

clf
step = 25;
doNotErase = 1;

for k = 1:step:size(d,1)
    xr = d(k,3);
    yr = d(k,4);
    tr = d(k,5);
    r = d(k,6:2:end);
    a = d(k,7:2:end);
    xy = [r.*cos(a);r.*sin(a);ones(1,length(r))];
    xy = xy(:,r<5.5);
    
    R = [cos(tr) -sin(tr) xr;sin(tr) cos(tr) yr; 0 0 1];
    xy = R * xy;
    
    if doNotErase
      hold on
      plot(xr,yr,'ro',xy(1,:),xy(2,:),'b.')
      hold off
    else
      plot(xr,yr,'ro',xy(1,:),xy(2,:),'b.')
      axis([-15 15 -30 15])
      drawnow
    end
end