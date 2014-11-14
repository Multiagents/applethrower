a0 = csvread('agent0.csv');
a1 = csvread('agent1.csv');
a2 = csvread('agent2.csv');
a3 = csvread('agent3.csv');
a4 = csvread('agent4.csv');
a5 = csvread('agent5.csv');
a6 = csvread('agent6.csv');
a7 = csvread('agent7.csv');
a = zeros(size(a0,1),16);
a(:,1:2) = a0(1:size(a0,1),2:3);
a(:,3:4) = a1(1:size(a1,1),2:3);
a(:,5:6) = a2(1:size(a2,1),2:3);
a(:,7:8) = a3(1:size(a3,1),2:3);
a(:,9:10) = a4(1:size(a4,1),2:3);
a(:,11:12) = a5(1:size(a5,1),2:3);
a(:,13:14) = a6(1:size(a6,1),2:3);
a(:,15:16) = a7(1:size(a7,1),2:3);


coordinates = zeros(8,2);
for i = 1:size(a0,1)
    for j = 1:size(coordinates,1)
        coordinates(j,1:2) = a(i,(j*2-1):(j*2));
    end
    map(coordinates)
    drawnow
end
