close all;
name = "input";
fid = fopen(sprintf("%s.dat",name));
A = (0.1*(0:17))';
for i = 1:2
  data = str2num(fgetl(fid));
  A = [A data'];
 end
%  A = [["#X" "Y" "Phi" "Xdot" "Ydot" "Phidot"]; A];
fig1 = figure(1);
xlabel("Time (s)");
hold on;
plot(A(:,1),A(:,2),'DisplayName','Thrust',"LineWidth",2);
plot(A(:,1),A(:,3),'DisplayName',"Roll Moment","LineWidth",2);
legend;
