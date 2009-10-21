%doRot2canImages

imgsOK=[1:400];

for i=1:length(imgsOK);

   sfile1=['img' num2str(imgsOK(i),'%03d') '.jpg'];
   sfile2=['msk' num2str(imgsOK(i),'%03d') '.jpg'];
   
   dfile1=['can\img' num2str(i,'%03d') '.jpg'];
   dfile2=['can\msk' num2str(i,'%03d') '.jpg'];
   
   x=imread(sfile1);
   b=imread(sfile2);

   [x1,b1]=rot2can(x,b);

   imwrite(x1,dfile1);
   imwrite(b1,dfile2);
   
end;   
