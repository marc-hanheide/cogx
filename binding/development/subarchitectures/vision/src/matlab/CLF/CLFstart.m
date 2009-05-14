function CLFstart(flag)

global avAcronyms avNames Fnames
global currState currMode
global mAV mDA mFS
global LRguiL LRguiR

avNames={'red';'green';'blue';'yellow';'square';'triangular';'circular'};
avAcronyms=['Rd';'Gr';'Bl';'Yl';'Sq';'Tr';'Cr'];
Fnames=extAPfeatures;

currMode=struct('learnMode',2,'THRs',[.999 .9995 .9999],'wT',1,'wYes',.75,'wPy',.25);

[mAV,mDA,mFS]=KDBFinit;

LRguiL=LRvisStart;

if flag==1
   set(LRguiL,'Visible','On');
end;

