function [hue_selected, points_xy, ah, raw, mask] = getData( fname, generate )

% ah ... hue image
% ah_r  hue values in row

% fname = 'apple' ;
raw = imread([fname,'.bmp']) ;
mask = imread([fname,'_mask.bmp']) ;
a = rgb2hsv(raw) ;
ah = double(a(:,:,1)) ;
mask = mask(:,:,1) < 120*20 ;
% mask = mask(:,:,1) > 120 ;
m = mask(:,:,1) ;

if generate == 0 
    hue_selected = load([fname,'_H','.txt'])  ;
    points_xy =load([fname,'_P','.txt'])  ;
    return ;
end

m_r = reshape(m,1,size(m,1)*size(m,2)) ;
i = find(m_r > 0) ;
ids = randperm(length(i)) ;
sel = i(ids(1:min([length(i),generate]))) ;

m_r = m_r*0 ; m_r(sel) = 1 ;
m_sel = reshape(m_r,size(m,1),size(m,2)) ;

hue_selected = [] ;
points_xy = [] ;
for i = 1 : size(m,1)
    for j = 1 : size(m,2)
        if ( m_sel(i,j) > 0 )
            points_xy = [points_xy; [j,i]] ;
            hue_selected = [hue_selected, ah(i,j)] ;
        end
    end
end
r = randperm(rows(points_xy)) ;
points_xy(:,:) = points_xy(r,:) ;
hue_selected = hue_selected(r) ;

save([fname,'_H','.txt'],'hue_selected','-ascii') ;
save([fname,'_P','.txt'],'points_xy','-ascii') ;


