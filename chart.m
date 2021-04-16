figure(1)
x = [,,0,];
p = pie(x);

pText = findobj(p,'Type','text');
percentValues = get(pText,'String'); 
txt = {'Module: ';'Weekly Prac: ';'Self assigned Project: '}; 
combinedtxt = strcat(txt,percentValues);

pText(1).String = combinedtxt(1);
pText(2).String = combinedtxt(2);
pText(3).String = combinedtxt(3);


figure(2)
x = [40,50,10];
p = pie(x);

pText = findobj(p,'Type','text');
percentValues = get(pText,'String'); 
txt = {'Module: ';'Weekly Prac: ';'Self assigned Project: '}; 
combinedtxt = strcat(txt,percentValues);

pText(1).String = combinedtxt(1);
pText(2).String = combinedtxt(2);
pText(3).String = combinedtxt(3);
