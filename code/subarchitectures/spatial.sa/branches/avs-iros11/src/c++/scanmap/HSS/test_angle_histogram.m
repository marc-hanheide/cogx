%cleP*(Kar

%PKKscans = load('data/data_small_hospital_loop/scans.tdf');
%scans = load('data/data_dumbo/scans.tdf');
%scans = load('data/data_stage-cogx/scans.tdf');

max_range = 5.59;

if 1
    load ahistscans_hosp
    %load ahistscans_4room
    fov = 240;
else
    load ahistscans_dumbo
    fov = 180;
end

da = fov / (size(scans,2) - 1)
angle = (-(fov/2):da:(fov/2))*pi/180;
nbins = 180;
astep = pi / nbins;
width = ceil(2/da)

for s = 500:size(scans,1)

    range = scans(s,:);
    n = length(range);
    
    r = [];
    a = [];
    x = [];
    y = [];

    for k = 1:length(range)
        if range(k) > max_range
            continue;
        end
        r = [r range(k)];
        a = [a angle(k)];
        x = [x r(end)*cos(a(end))];
        y = [y r(end)*sin(a(end))];
    end

    if length(angle) == size(scans,2)
    else
        disp('Error vectors not of same size')
        break
    end

    ahist = zeros(1,nbins);

    D = [];
    
    for k = 1:(length(x)-width)
        % Direction between points
        dir = atan2(y(k+width)-y(k),x(k+width)-x(k));
        if (dir < 0)
            dir = dir + pi;
        end

        D = [D dir];
        
        % Angle bin that this falls in
        b = floor(dir / astep) + 1;
        if (b > nbins)
            b = 0;
        end
        
        ahist(b) = ahist(b) + r(k);
    end

    figure(2), subplot(2,1,1), bar(ahist)
    
    % smooth the histogram a bit
    ks = 1;
    ker = ones(1,2*ks+1) / ( 2*ks+1 );
    ahist = conv(ahist, ker);
    ahist = ahist((ks+1):(length(ahist)-ks));
        
    % Find the max  
    [maxv,maxb] = max(ahist);

    % Zero all bin that are not at least some % of the max
    ii = find(ahist < 0.05*maxv);
    if ~isempty(ii)
       ahist(ii) = 0*ahist(ii);
    end

    % Calculate the fraction of angles that are within some window of this
    % direction. If this fraction is high enough the scan contains
    % structures in mainly one direction.
    sumw = 25;
    sumindex = (maxb-sumw):(maxb+sumw);
    sumindex = rem(sumindex + nbins, nbins)+1;
    frac = sum(ahist(sumindex)) / sum(ahist);

    figure(1)
    plot(x,y,'.')
    hold on
    if frac > 0.75
        text(2,0,'CORRIDOR')
    end
    hold off
    axis([-6 6 -6 6])
    axis square

    % Normalise the histogram (for no reason really)
    ahist = ahist / sum(ahist);

    [junk,maxb] = max(ahist);

    disp(sprintf('Max angle is %fdeg with %f of points for scan %d',(maxb-1),frac,s))

    figure(2), subplot(2,1,2), bar(ahist)

    figure(3), plot(D,'.')
    
    pause
end