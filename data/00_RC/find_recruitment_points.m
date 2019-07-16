look_PW = round(newPW*500);

%% Define the data to be fit
    i = 1;
    angle_y = zeros(1,11);
    pw_x = zeros(1,11);
    angle(1) = 0;
    pw_x(1) = 0;

    for pw_n = 50:50:500
        i = i + 1;
        pw_pts = find(look_PW==pw_n);
        angle_y(i) = mean(measAngle(pw_pts));
        pw_x(i) = pw_n;
    end

%% Define function that will be used to fit data
% pw_x = pw_x';
% f = @(F,x) F(1)/(1+exp(-F(2).*(x - F(3)))); %F(1): angle_max/ F(2) = -alpha/ F(3) = pw_mean
% F_fitted = nlinfit(pw_x, angle_y, f, [40 10^-6 200]);

p = polyfit(pw_x, angle_y, 3); % third order

%% Display fitted coefficients
% disp(['F = ',num2str(F_fitted)])
disp(['p = ',num2str(p)])

%% Create fit vector
    new_x = 0:1:500;
    y = polyval(p,new_x);

%% Plot the data and fit
figure()
plot(pw_x,angle_y,'*',new_x,y,'g');
legend('data','fit');

%% save
id_pw = new_x;
id_angle = y;

save('inverse_dynamics.mat', 'id_pw', 'id_angle');