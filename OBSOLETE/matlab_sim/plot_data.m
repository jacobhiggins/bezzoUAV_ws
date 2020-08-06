close all;
name = "data";
fid = fopen(sprintf("%s.dat",name));
A = (0.1*(0:18))';
for i = 1:6
  data = str2num(fgetl(fid));
  A = [A data'];
 end
%  A = [["#X" "Y" "Phi" "Xdot" "Ydot" "Phidot"]; A];
fig1 = figure(1);
plot(A(:,1),A(:,2),'DisplayName','Y position',"LineWidth",2);
xlabel("Time (s)");
ylabel("Distance (m)");
legend;
fig2 = figure(2);
plot(A(:,1),A(:,3),"DisplayName",'Z position',"LineWidth",2);
xlabel("Time (s)");
ylabel("Distance (m)");
legend;
fig3 = figure(3);
plot(A(:,2),A(:,3),"DisplayName","Path","LineWidth",2);
xlabel("Y Position (m)");
ylabel("Z Position (m)");
legend;
