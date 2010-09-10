function img=showImg(varargin)
%SHOWIMG  Show image.
%   IMG = SHOWIMG(Xi) displays the image Xi. Xi can be a vector or a matrix
%   representing an image. Returned value IMG is the image matrix. 
%   The image vector is reshaped to the image of the equal height and width.
%
%   IMG = SHOWIMG(X,I) displays I-th column of the data matrix X as an image.
%   The image vector is reshaped to the image of the equal height and width.
%   
%   IMG = SHOWIMG(Xi,N,M) displays the image vector Xi as an image of width N 
%   and height M.
%
%   IMG = SHOWIMG(X,I,N,M) displays I-th column of the data matrix X as an image
%   of width N and height M.
%
%   See also SHOWIMGS, LOADIMG.

nin=nargin;
switch nin
  case 1 %=showImg(Xi) or showImg(Img);
    Xi=varargin{1}; 
    [n,m]=size(Xi);
    if m>1 %=showImg(Img);
       img=Xi;
    else %=showImg(Xi);
       n=sqrt(n);   
	    m=n;
		 img=reshape(Xi,n,m);
    end;   
 case 2 
    if size(varargin{2})==1 %=showImg(X,i);
	    Xi=varargin{1}(:,varargin{2}); 
   	 n=sqrt(size(Xi,1));   
	    m=n;   
       img=reshape(Xi,n,m);
    else %=showImg(Xi,[m,n]);    
	    Xi=varargin{1}; ;   
	    m=varargin{2}(1);
   	 n=varargin{2}(2);
       img=reshape(Xi,m,n);
    end;   
 case 3 %=showImg(Xi,n,m);
    Xi=varargin{1}; ;   
    n=varargin{2};
    m=varargin{3};
	 img=reshape(Xi,n,m);
  case 4 %=showImg(X,i,n,m); 
    Xi=varargin{1}(:,varargin{2}); 
    n=varargin{3};
    m=varargin{4};
	 img=reshape(Xi,n,m);
end   

imagesc(img);
