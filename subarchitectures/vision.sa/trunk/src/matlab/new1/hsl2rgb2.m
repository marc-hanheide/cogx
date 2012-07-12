function rgb=hsl2rgb2(hsl)

%Converts Hue-Saturation-Luminance Color value to Red-Green-Blue Color value
%
%Usage
%       RGB = hsl2rgb(HSL)
%
%   converts HSL, a M X 3 color matrix with values between 0 and 1
%   into RGB, a M X 3 color matrix with values between 0 and 1
%
%See also rgb2hsl, rgb2hsv, hsv2rgb

%Suresh E Joel, April 26,2003

if nargin<1,
    error('Too few arguements for hsl2rgb');
    return;
elseif nargin>1,
    error('Too many arguements for hsl2rgb');
    return;
end;

if max(max(hsl))>1 | min(min(hsl))<0,
    error('HSL values have to be between 0 and 1');
    return;
end;

[h w ~]=size(hsl);
rgb=zeros(h,w,3);

for i=1:h
    
    for j=1:w
        
        if hsl(i,j,2)==0,%when sat is 0
            rgb(i,j,1:3)=hsl(i,j,3);% all values are same as luminance
        end;
        if hsl(i,j,3)<0.5,
            temp2=hsl(i,j,3)*(1+hsl(i,j,2));
        else
            temp2=hsl(i,j,3)+hsl(i,j,2)-hsl(i,j,3)*hsl(i,j,2);
        end;
        temp1=2*hsl(i,j,3)-temp2;
        temp3(1)=hsl(i,j,1)+1/3;
        temp3(2)=hsl(i,j,1);
        temp3(3)=hsl(i,j,1)-1/3;
        for k=1:3,
            if temp3(k)>1,
                temp3(k)=temp3(k)-1;
            elseif temp3(k)<0,
                temp3(k)=temp3(k)+1;
            end;
            if 6*temp3(k)<1,
                rgb(i,j,k)=temp1+(temp2-temp1)*6*temp3(k);
            elseif 2*temp3(k)<1,
                rgb(i,j,k)=temp2;
            elseif 3*temp3(k)<2,
                rgb(i,j,k)=temp1+(temp2-temp1)*(2/3-temp3(k))*6;
            else
                rgb(i,j,k)=temp1;
            end;
        end;
    end
    
end;

rgb=round(rgb.*100000)./100000; %Sometimes the result is 1+eps instead of 1 or 0-eps instead of 0 ... so to get rid of this I am rounding to 5 decimal places)