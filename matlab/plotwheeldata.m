function  plotwheeldata(bagfilename)
%
%   plotwheeldata(bagfilename)
%
%   Plot the wheel command, desired, and actual for left/right wheels.
%   If 'bagfilename' is not given or given as 'latest', use the most
%   recent bag file.
%

% If no bagfile is specified, use the most recent.
if (~exist('bagfilename') || strcmp(bagfilename, 'latest'))
    bagfilename = latestbagfilename();
end

% Load the bag.
try
    bag = rosbag(bagfilename);
catch
    error(['Unable to open the bag file ''' bagfilename '''']);
end

% Grab the entire bag's start time.
tstart = bag.StartTime;

% Proceed with each wheel
plotwheel(1, tstart, bag, 'leftwheel');
plotwheel(2, tstart, bag, 'rightwheel');
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function  plotwheel(fig, tstart, bag, joint)
%
%   plotwheel(fig, tstart, bag, joint)
%
%   Plot the data in the name figure, starting at the given time, from
%   the given bag, for the named joint.
%

% Internal function to grab the data.  Make the time relative.
function [t, p, v, e, name] = data(topic)
    try
        [t, p, v, e, name] = jointstatedata(bagmsgs(bag, topic), joint);
        t = t - tstart;
    catch ex
        warning("ERROR: " + ex.message);
        % getReport(ex)
        [t, p, v, e, name] = deal([], [], [], [], {});
    end
end

% Read the data.
[tc, pc, vc, ec, name] = data('/wheel_command');
[td, pd, vd, ed, name] = data('/wheel_desired');
[ta, pa, va, ea, name] = data('/wheel_state');  

% Plot.
figure(fig);
clf;

% Plot.
ax(1) = subplot(2,1,1);
hold on;
if ~isempty(pc), plot(tc,pc,'bo--','LineWidth',2,'DisplayName','Command'); end
if ~isempty(pd), plot(td,pd,'ro:', 'LineWidth',2,'DisplayName','Desired'); end
if ~isempty(pa), plot(ta,pa,'gx-', 'LineWidth',2,'DisplayName','Actual');  end
grid on;
ylabel('Position (rad)');
title(['Data for ' joint]);
legend;

ax(2) = subplot(2,1,2);
hold on;
if ~isempty(vc), plot(tc,vc,'bo--','LineWidth',2,'DisplayName','Command'); end
if ~isempty(vd), plot(td,vd,'ro:', 'LineWidth',2,'DisplayName','Desired'); end
if ~isempty(va), plot(ta,va,'gx-', 'LineWidth',2,'DisplayName','Actual');  end
grid on;
ylabel('Velocity (rad/sec)');
xlabel('Time (sec)');

linkaxes(ax, 'x');

% Name the Figure and span the full 8.5x11 page.
set(gcf, 'Name',          'Joint Data');
set(gcf, 'PaperPosition', [0.25 0.25 8.00 10.50]);

% Return to the top subplot, so subsequent title()'s go here...
subplot(2,1,1);

end
