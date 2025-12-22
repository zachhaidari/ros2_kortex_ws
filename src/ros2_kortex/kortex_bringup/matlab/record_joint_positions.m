% record_joint_positions.m
% Subscribe to /matlab_joint_angles (trajectory_msgs/JointTrajectory)
% Record incoming joint positions over time and plot them live.
%
% Usage in MATLAB with ROS2 toolbox:
%   node = ros2node('/matlab_listener');
%   sub = ros2subscriber(node, '/matlab_joint_angles', 'trajectory_msgs/JointTrajectory', @callback);
%   % The script will run until you clear the subscriber or close MATLAB.
%
% If you prefer a blocking receive loop, run the function record_joint_positions_blocking()
%
function record_joint_positions()
    % Create node and subscriber
    node = ros2node('/matlab_listener');
    % Shared data stored in nested function scope via handles
    data.time = [];
    data.joints = zeros(0,6);
    data.count = 0;

    fig = figure('Name','EE Joint Positions','NumberTitle','off');
    ax = axes(fig);
    hold(ax,'on');
    colors = lines(6);
    h = gobjects(6,1);
    for i=1:6
        h(i) = plot(ax, nan, nan, 'Color', colors(i,:), 'LineWidth', 1.5);
    end
    xlabel(ax,'Sample #');
    ylabel(ax,'Joint Position (rad)');
    legend(ax,{'joint_1','joint_2','joint_3','joint_4','joint_5','joint_6'});
    grid(ax,'on');

    % Create subscriber with callback that updates data and plot
    sub = ros2subscriber(node, '/matlab_joint_angles', 'trajectory_msgs/JointTrajectory', @(src,msg)callback(src,msg));

    % Keep MATLAB responsive until figure is closed
    disp('Recording joint positions. Close figure window to stop.');
    uiwait(fig);

    % Clean up
    clear sub
    clear node

    function callback(~, msg)
        try
            if isempty(msg.Points)
                return;
            end
            pt = msg.Points(1);
            pos = double(pt.Positions);
            % Ensure length 6
            if length(pos) < 6
                pos = [pos, zeros(1,6-length(pos))];
            end
            data.count = data.count + 1;
            data.time(end+1,1) = data.count;
            data.joints(end+1, :) = pos(1:6);

            % Update plot
            n = size(data.joints,1);
            for k = 1:6
                set(h(k), 'XData', (1:n), 'YData', data.joints(:,k));
            end
            drawnow limitrate
        catch ME
            disp(['Callback error: ', ME.message]);
        end
    end
end

function record_joint_positions_blocking(duration_sec)
    % Blocking variant: collect messages for duration_sec then plot
    if nargin < 1
        duration_sec = 30;
    end
    node = ros2node('/matlab_listener_block');
    sub = ros2subscriber(node, '/matlab_joint_angles', 'trajectory_msgs/JointTrajectory');
    disp(['Collecting for ', num2str(duration_sec), 's...']);
    start_t = tic;
    times = [];
    joints = [];
    while toc(start_t) < duration_sec
        msg = receive(sub,1.0); % timeout 1s
        if isempty(msg)
            continue;
        end
        if isempty(msg.Points)
            continue;
        end
        pt = msg.Points(1);
        pos = double(pt.Positions);
        if length(pos) < 6
            pos = [pos, zeros(1,6-length(pos))];
        end
        times(end+1,1) = toc(start_t);
        joints(end+1,:) = pos(1:6);
    end
    figure; hold on; colors = lines(6);
    for k=1:6
        plot(times, joints(:,k), 'Color', colors(k,:), 'LineWidth', 1.5);
    end
    xlabel('Time (s)'); ylabel('Joint Position (rad)'); legend({'j1','j2','j3','j4','j5','j6'});
    grid on;
    clear sub; clear node;
end

function record_joint_positions_3d(mode, duration_sec, joint_idx)
    % 3D plotting helper
    % mode: 'trajectory' (plot joint_i vs joint_j vs joint_k)
    %       'surface'    (time x joint_index surface)
    % duration_sec: seconds to collect (default 20)
    % joint_idx: 1x3 vector of joint indices to plot for 'trajectory' (default [1 2 3])
    if nargin < 1 || isempty(mode)
        mode = 'trajectory';
    end
    if nargin < 2 || isempty(duration_sec)
        duration_sec = 20;
    end
    if nargin < 3 || isempty(joint_idx)
        joint_idx = [1 2 3];
    end
    mode = lower(mode);
    % Collect messages like the blocking variant
    node = ros2node('/matlab_listener_3d');
    sub = ros2subscriber(node, '/matlab_joint_angles', 'trajectory_msgs/JointTrajectory');
    disp(['Collecting for ', num2str(duration_sec), 's (mode=', mode, ')...']);
    start_t = tic;
    times = [];
    joints = [];
    while toc(start_t) < duration_sec
        msg = receive(sub,1.0);
        if isempty(msg) || isempty(msg.Points)
            continue;
        end
        pt = msg.Points(1);
        pos = double(pt.Positions);
        if length(pos) < 6
            pos = [pos, zeros(1,6-length(pos))];
        end
        times(end+1,1) = toc(start_t);
        joints(end+1,:) = pos(1:6);
    end
    % Plot depending on mode
    switch mode
        case 'trajectory'
            % plot 3D trajectory of three selected joints
            a = joint_idx(1); b = joint_idx(2); c = joint_idx(3);
            figure('Name','3D Joint Trajectory','NumberTitle','off');
            plot3(joints(:,a), joints(:,b), joints(:,c), '-o','LineWidth',1.2);
            xlabel(sprintf('joint_%d (rad)', a)); ylabel(sprintf('joint_%d (rad)', b)); zlabel(sprintf('joint_%d (rad)', c));
            grid on; title(sprintf('3D Trajectory: j%d vs j%d vs j%d',a,b,c));
        case 'surface'
            % time x joint_index surface
            figure('Name','Joint Positions Surface','NumberTitle','off');
            [T,J] = meshgrid(1:size(joints,1), 1:size(joints,2));
            surf(T, J, joints','EdgeColor','none');
            view(45,30);
            xlabel('Sample #'); ylabel('Joint Index'); zlabel('Joint Position (rad)');
            title('Joint Positions over Time (surface)'); colorbar; grid on;
        otherwise
            error('Unknown mode: %s', mode);
    end
    clear sub; clear node;
end

function record_joint_positions_3d_live(mode, joint_idx, buffer_len)
    % Live 3D plotting: updates while messages arrive until figure closed.
    % mode: 'trajectory' (3D joint_i vs joint_j vs joint_k) or 'surface' (time x joint_index)
    % joint_idx: for 'trajectory' a 1x3 vector of joints to plot (default [1 2 3])
    % buffer_len: number of recent samples to keep (default 500)
    if nargin < 1 || isempty(mode)
        mode = 'trajectory';
    end
    if nargin < 2 || isempty(joint_idx)
        joint_idx = [1 2 3];
    end
    if nargin < 3 || isempty(buffer_len)
        buffer_len = 500;
    end
    mode = lower(mode);

    node = ros2node('/matlab_listener_3d_live');
    % shared state
    state.times = [];
    state.joints = zeros(0,6);
    state.count = 0;

    % Create figure and initial plot objects
    fig = figure('Name','Live 3D Joint Plot','NumberTitle','off');
    ax = axes(fig);
    hold(ax,'on');

    switch mode
        case 'trajectory'
            a = joint_idx(1); b = joint_idx(2); c = joint_idx(3);
            hLine = plot3(ax, nan, nan, nan, '-o', 'LineWidth', 1.2, 'MarkerSize', 4);
            xlabel(ax, sprintf('joint_%d (rad)', a));
            ylabel(ax, sprintf('joint_%d (rad)', b));
            zlabel(ax, sprintf('joint_%d (rad)', c));
            title(ax, sprintf('Live 3D Trajectory: j%d vs j%d vs j%d', a, b, c));
            grid(ax,'on');
        case 'surface'
            % Use imagesc for efficient live updates (time x joint index)
            hImg = imagesc(ax, nan(buffer_len,6));
            colormap(ax,'jet'); colorbar(ax);
            xlabel(ax,'Sample (newest -> right)'); ylabel(ax,'Joint Index');
            title(ax,'Live Joint Positions (time x joint index)');
        otherwise
            error('Unknown mode: %s', mode);
    end

    % Subscriber with callback that updates state and plot
    sub = ros2subscriber(node, '/matlab_joint_angles', 'trajectory_msgs/JointTrajectory', @(~,msg)cb(msg));

    disp('Live 3D plotting started. Close figure to stop.');
    % keep running until figure closed
    waitfor(fig);

    % cleanup
    try
        clear sub
        clear node
    catch
    end

    function cb(msg)
        try
            if isempty(msg) || isempty(msg.Points)
                return;
            end
            pt = msg.Points(1);
            pos = double(pt.Positions);
            if length(pos) < 6
                pos = [pos, zeros(1,6-length(pos))];
            end
            % append
            state.count = state.count + 1;
            state.joints(end+1, :) = pos(1:6);
            if size(state.joints,1) > buffer_len
                state.joints(1,:) = []; % drop oldest row
            end

            % update plot depending on mode
            switch mode
                case 'trajectory'
                    a = joint_idx(1); b = joint_idx(2); c = joint_idx(3);
                    X = state.joints(:,a);
                    Y = state.joints(:,b);
                    Z = state.joints(:,c);
                    set(hLine, 'XData', X, 'YData', Y, 'ZData', Z);
                    drawnow limitrate
                case 'surface'
                    % show newest samples to the right
                    M = state.joints;
                    % pad to buffer_len with NaN on top if needed
                    rows = size(M,1);
                    if rows < buffer_len
                        pad = nan(buffer_len-rows, size(M,2));
                        Mdisp = [pad; M];
                    else
                        Mdisp = M(end-buffer_len+1:end, :);
                    end
                    % imagesc expects X by Y, convert to buffer_len x joints
                    set(hImg, 'CData', Mdisp);
                    drawnow limitrate
            end
        catch ME
            disp(['Live callback error: ', ME.message]);
        end
    end
end
