function [lmAcr,lmName]=LMname

global LM

switch LM
   case 1
      lmAcr='TD';
      lmName='Tutor Driven';
   case 2
      lmAcr='TS';
      lmName='Tutor Supervised';
   case 3
      lmAcr='TA';
      lmName='Tutor Assisted';
   case 4
      lmAcr='TU';
      lmName='Tutor Unassisted';
   case 5
      lmAcr='oTD';
      lmName='Old Tutor Driven';
   case 6
      lmAcr='oTSc';
      lmName='Old Tutor Supervised Conservative';
   case 7
      lmAcr='oTSl';
      lmName='Old Tutor Supervised Liberal';
   case 8
      lmAcr='oEXc';
      lmName='Old Exploratory Conservative';
   case 9
      lmAcr='oEXl';
      lmName='Old Exploratory Liberal';
end

  