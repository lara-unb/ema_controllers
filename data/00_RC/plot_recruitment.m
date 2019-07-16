
figure();

subplot(2,1,1);
    plot(timePW,PW_values);
    ylabel("Pulse width [\mus]");
    xlim([time_start,time_end]);
    ylim([0,500]);
    
subplot(2,1,2);
    plot(timeAngle,measAngle);
    ylabel("Angle [º]");
    xlim([time_start,time_end]);
    ylim([-15,90]);
    
xlabel("Time [s]");

% suptitle(strcat("Recruitment curve for the quadriceps muscles stimulated at ",num2str(i_applied)," mA."));