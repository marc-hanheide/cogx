function hsl=rgb2hsl2(rgb)

%Converts Red-Green-Blue Color value to Hue-Saturation-Luminance Color value
%
%Usage
%       HSL = rgb2hsl(RGB)
%
%   converts RGB, a M X 3 color matrix with values between 0 and 1
%   into HSL, a M X 3 color matrix with values between 0 and 1
%
%See also hsl2rgb, rgb2hsv, hsv2rgb

%Suresh E Joel, April 26,2003

if nargin<1,
    error('Too few arguements for rgb2hsl');
    return;
elseif nargin>1,
    error('Too many arguements for rgb2hsl');
    return;
end;

if max(max(rgb))>1 | min(min(rgb))<0,
    error('RGB values have to be between 0 and 1');
    return;
end;

[h w ~]=size(rgb);
hsl=ones(h,w,3);

for i=1:h
    
    for j=1:w
        
        mx=max(rgb(i,j,:));%max of the 3 colors
        mn=min(rgb(i,j,:));%min of the 3 colors
        imx=find(rgb(i,j,:)==mx);%which color has the max
        hsl(i,j,3)=(mx+mn)/2;%luminance is half of max value + min value
        if(mx-mn)==0,%if all three colors have same value,
            hsl(i,j,2)=0;%then s=0 and
            hsl(i,j,1)=0;%h is undefined but for practical reasons 0
            
            % napaka v kodi - javljena na File Exchange
            %return;
            continue;
        end;
        if hsl(i,j,3)<0.5,
            hsl(i,j,2)=(mx-mn)/(mx+mn);
        else
            hsl(i,j,2)=(mx-mn)/(2-(mx+mn));
        end;
        switch(imx(1))%if two colors have same value and be the maximum, use the first color
            case 1 %Red is the max color
                hsl(i,j,1)=((rgb(i,j,2)-rgb(i,j,3))/(mx-mn))/6;
            case 2 %Green is the max color
                hsl(i,j,1)=(2+(rgb(i,j,3)-rgb(i,j,1))/(mx-mn))/6;
            case 3 %Blue is the max color
                hsl(i,j,1)=(4+(rgb(i,j,1)-rgb(i,j,2))/(mx-mn))/6;
        end;
        if hsl(i,j,1)<0,hsl(i,j,1)=hsl(i,j,1)+1;end;%if hue is negative, add 1 to get it within 0 and 1
    end;
end

hsl=round(hsl*100000)/100000; %Sometimes the result is 1+eps instead of 1 or 0-eps instead of 0 ... so to get rid of this I am rounding to 5 decimal places)