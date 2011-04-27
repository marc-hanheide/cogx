function B = getBoundingBox(M)

W = size(M,2) ;
H = size(M,1) ;
mm = sum(sum(M,1)) ;
if mm <= 0 mm = 1 ; end 
mY = sum( sum(M,2)'.*[1:H] / mm ) ;
mX = sum( sum(M,1).*[1:W] / mm ) ;

swaps = find(sum(M,1)>0) ;
i_min = min(swaps) ; i_max = max(swaps) ;
Bx1 = [i_min, i_max] ;
swaps = find(sum(M,2)>0) ; 
i_min = min(swaps) ; i_max = max(swaps) ;
By1 = [i_min, i_max] ;
B = [ Bx1; By1 ] ;