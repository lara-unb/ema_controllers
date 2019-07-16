
%% find time offset
gui_lists = gui_lists(:,2:end);
time_offset = gui_lists(1,1);
gui_lists(1,:) = gui_lists(1,:) - time_offset;
% clear time_offset;

time_start = gui_lists(1,end-1);
time_end = gui_lists(1,end);

%% find reference
t = ref_lists{4};
t(t>1000) = 0;
angle_idx1 = find(t>time_start,1);
angle_idx2 = find(t>time_end,1);

timePW = t(angle_idx1:angle_idx2);
PW = ref_lists{3};
PW = PW(angle_idx1:angle_idx2);
PW_values = PW*500;

%% find current
i_applied = stim_lists(1,find(stim_lists(1,:)~=0,1)); %current

%% find angle and error
t = angle_err_lists(3,:);
angle_idx1 = find(t>time_start,1);
angle_idx2 = find(t>time_end,1);

timeAngle = t(angle_idx1:angle_idx2);
measAngle = angle_err_lists(1,angle_idx1:angle_idx2);

%% make vectors the same length
newTime = timeAngle;
newPW = interp1(timePW, PW, newTime);
newAngle = measAngle;
