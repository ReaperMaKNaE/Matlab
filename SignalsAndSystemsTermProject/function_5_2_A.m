function h=function_5_2_A(timeconstant,voice)
a=1;
b=[1 timeconstant];
h=filter(a,b,voice);