function data1=DSappendWord(data,datum)

global senMap

if (isempty(data) && datum<0) || (~isempty(data) && datum>0 && ~isempty(senMap{-data(1),2}))
   data1=[data datum];
else
   data1=data;
   beep;
end   