
figure();
title(strcat("Recruitment points for the quadriceps muscles stimulated at ",num2str(i_applied)," mA."));

newPW2 = newPW;

n = find(newPW2==0);
newPW2(n) = [];
newAngle(n) = [];
newTime(n) = [];

subplot(2,1,1);
    plot(newPW2*500);
    ylabel("Pulse width [\mus]");
%     xlim([time_start,time_end]);
    ylim([0,500]);
    
subplot(2,1,2);
    plot(newAngle);
    ylabel("Angle [º]");
%     xlim([time_start,time_end]);
    ylim([-15,60]);
    
xlabel("Time [s]");