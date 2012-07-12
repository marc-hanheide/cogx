function VStestColorsHL(S,st)

global mC;
global Test;

if nargin<1
    S=.5;
    st=.1;
end

%st=.1;
H=0:st:1;
V=0:st:1;
%V=0.5;
wh=length(H);
HSV=zeros(wh,wh,3);

[HSV(:,:,1),HSV(:,:,3)]=meshgrid(H,V);
HSV(:,:,2)=repmat(S,wh,wh);

%RGB=hsv2rgb(HSV);
RGB=hsl2rgb2(HSV);
Test.RGB=RGB;

s1=.5;
s2=s1;
s3=s1;
REC=zeros(wh,wh);
PR=zeros(wh,wh);
for i=1:wh;
    for j=1:wh
        
        F=[squeeze(HSV(i,j,:));s1;s2;s3];
        % F(1)=F(1)/3*4;
        rslt = executeOperatorIKDEClsfr( mC{1}, 'input_data', F, 'classifyData', 'use_unknown_model',1   ) ;
        pcx=[[getc(mC,1,0,'name')';0] rslt.P];
        
        [pmax,pidx]=max(pcx(:,2));
        pname=pcx(pidx,1);
        
        REC(i,j)=pname;
        PR(i,j)=pmax;
        
    end
end

cmap=[[.93 0 .93];[1 0 0];[0 1 0];[0 0 1];[1 1 0];[0 0 0];[1 1 1];[1 .65 0];[1 .75 .8]];
numc=size(cmap,1)-1;

RECrgb=zeros(wh,wh,3);
RECrgb1=RECrgb(:,:,1);
RECrgb2=RECrgb(:,:,2);
RECrgb3=RECrgb(:,:,3);
for i=0:numc
    idxs=find(REC==i);
    RECrgb1(idxs)=cmap(i+1,1);
    RECrgb2(idxs)=cmap(i+1,2);
    RECrgb3(idxs)=cmap(i+1,3);
end
RECrgb=cat(3,RECrgb1,RECrgb2,RECrgb3);

dfigure(4.3,1.2,['VStest - Test colours HL; S=',num2str(S)]);
resizefigs(gcf,4.5,1.2);
Test.rgba=subplot(1,3,1);
%showimg(RGB);
imagesc(RGB);
set(gca,'YDir','normal');
xlabel('Hu');ylabel('Li');title('Input');
subplot(1,3,2);
imagesc(RECrgb);
set(gca,'YDir','normal');
xlabel('Hu');ylabel('Li');title('Recognized');
subplot(1,3,3);
imagesc(PR);
set(gca,'YDir','normal');
colormap gray;
xlabel('Hu');ylabel('Li');title('AP');
