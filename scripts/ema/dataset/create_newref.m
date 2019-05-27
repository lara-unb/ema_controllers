clear;


% t = linspace(0,1,500);
% knee_ref1 = 50*t;
% knee_ref2 = 50*(flip(t));
% 
% knee_ref = [knee_ref1, knee_ref2];
% 
% save("ref50angle.mat");


t = linspace(0,1,500);
ref1 = 70*t-10;
ref2 = 60*ones(1,length(t));

knee_ref = [ref1, ref2, flip(ref1)];

plot(knee_ref);

save("reference_for_thesis.mat");