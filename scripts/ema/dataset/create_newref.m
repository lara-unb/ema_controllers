clear;

%% 1
% t = linspace(0,1,500);
% knee_ref1 = 50*t;
% knee_ref2 = 50*(flip(t));
% 
% knee_ref = [knee_ref1, knee_ref2];
% 
% save("ref50angle.mat");

%% 2
% t = linspace(0,1,500);
% ref1 = 45*t-5;
% ref2 = 40*ones(1,length(t));
% 
% knee_ref = [ref1, ref2, flip(ref1)];
% 
% plot(knee_ref);
% save("reference_for_thesis.mat");

%% 3
% t = linspace(0,1,500);
% ref1 = 40*t; %1s
% ref2 = 40*ones(1,length(t));
% ref3 = flip(ref1);
% ref4 = 0*ones(1,2*length(t));
% 
% knee_ref = [ref1, ref2, ref3, ref4];
% 
% plot(knee_ref);
% 
% save("reference_for_thesis_rest.mat");


%% 4
n = 100;
t = linspace(0,pi/2,n*2);
ref1 = 35*sin(t);
ref2 = flip(ref1);
t = linspace(0,pi/2,n*4);
ref3 = 45*sin(t); 
ref4 = flip(ref3);
ref5 = 0*ones(1,n);

knee_ref = [ref1, ref2, ref3, ref4 ref5]; 

plot(knee_ref);

save("senoides1300.mat");