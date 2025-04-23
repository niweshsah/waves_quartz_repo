function quartz_clock_with_power()
    % Quartz Clock Simulation with Power Supply and Damping Compensation
    % Creates an interactive GUI to visualize damping and power compensation

    %% Default parameters
    params.f = 32768;            % Frequency (Hz)
    params.zeta = 0.001;         % Damping ratio
    params.A = 1;                % Target amplitude
    params.t_end = 0.003;        % Simulation duration (s)
    params.speed = 1;            % Animation speed multiplier
    params.powerOn = true;       % Power supply state
    params.powerMode = 1;        % 1: Continuous, 2: Pulsed
    params.powerEfficiency = 0.95;

    %% Figure and control panel
    fig = figure('Name', 'Quartz Clock Simulation', ...
                 'Color', 'white', 'Position', [100, 100, 1000, 800]);
    ctrl = uipanel(fig, 'Title', 'Controls', 'Position', [0.01, 0.01, 0.25, 0.35]);

    % Frequency slider
    uicontrol(ctrl, 'Style', 'text', 'Position', [10, 220, 230, 20], ...
              'Tag', 'freq_label', 'String', sprintf('Frequency: %d Hz', params.f));
    uicontrol(ctrl, 'Style', 'slider', 'Min', 100, 'Max', 1e5, 'Value', params.f, ...
              'Position', [10, 200, 230, 20], 'Callback', @(s,~) update_param(s, fig, 'f'));

    % Damping slider
    uicontrol(ctrl, 'Style', 'text', 'Position', [10, 180, 230, 20], ...
              'Tag', 'zeta_label', 'String', sprintf('Damping: %.5f', params.zeta));
    uicontrol(ctrl, 'Style', 'slider', 'Min', 1e-4, 'Max', 1e-2, 'Value', params.zeta, ...
              'Position', [10, 160, 230, 20], 'Callback', @(s,~) update_param(s, fig, 'zeta'));

    % Power efficiency slider
    uicontrol(ctrl, 'Style', 'text', 'Position', [10, 140, 230, 20], ...
              'Tag', 'eff_label', 'String', sprintf('Power Eff.: %.2f', params.powerEfficiency));
    uicontrol(ctrl, 'Style', 'slider', 'Min', 0, 'Max', 1, 'Value', params.powerEfficiency, ...
              'Position', [10, 120, 230, 20], 'Callback', @(s,~) update_param(s, fig, 'powerEfficiency'));

    % Target amplitude slider
    uicontrol(ctrl, 'Style', 'text', 'Position', [10, 100, 230, 20], ...
              'Tag', 'amp_label', 'String', sprintf('Amplitude: %.2f', params.A));
    uicontrol(ctrl, 'Style', 'slider', 'Min', 0.1, 'Max', 2, 'Value', params.A, ...
              'Position', [10, 80, 230, 20], 'Callback', @(s,~) update_param(s, fig, 'A'));

    % Animation speed slider
    uicontrol(ctrl, 'Style', 'text', 'Position', [10, 60, 230, 20], ...
              'Tag', 'speed_label', 'String', sprintf('Speed: %.1fx', params.speed));
    uicontrol(ctrl, 'Style', 'slider', 'Min', 0.1, 'Max', 10, 'Value', params.speed, ...
              'Position', [10, 40, 230, 20], 'Callback', @(s,~) update_param(s, fig, 'speed'));

    % Power on/off toggle
    uicontrol(ctrl, 'Style', 'togglebutton', 'String', 'Power ON', 'Value', params.powerOn, ...
              'Position', [10, 10, 100, 25], 'Tag', 'power_btn', ...
              'Callback', @(s,~) update_param(s, fig, 'powerOn'));

    % Power mode radio buttons
    bg = uibuttongroup(ctrl, 'Title', 'Mode', 'Position', [0.5, 0.01, 0.48, 0.15], ...
                      'SelectionChangedFcn', @(~,e) mode_changed(e, fig));
    uicontrol(bg, 'Style', 'radiobutton', 'String', 'Continuous', 'Tag', 'cont');
    uicontrol(bg, 'Style', 'radiobutton', 'String', 'Pulsed', 'Tag', 'pulse');

    % Restart button
    uicontrol(ctrl, 'Style', 'pushbutton', 'String', 'Restart', 'Position', [60, 260, 120, 30], ...
              'Callback', @(~,~) restart_sim(fig));

    % Store data and handlers
    setappdata(fig, 'params', params);
    setappdata(fig, 'isRunning', true);
    setappdata(fig, 'ctrl', ctrl);

    % Create axes for plots
    ax1 = subplot(3,1,1, 'Parent', fig); setappdata(fig, 'ax1', ax1);
    ax2 = subplot(3,1,2, 'Parent', fig); setappdata(fig, 'ax2', ax2);
    ax3 = subplot(3,1,3, 'Parent', fig); setappdata(fig, 'ax3', ax3);

    % Initialize and run
    initialize_plots(fig);
    animation_loop(fig);
end

%% Parameter update callback
function update_param(src, fig, field)
    params = getappdata(fig, 'params');
    val = get(src, 'Value');
    params.(field) = val;
    setappdata(fig, 'params', params);
    % Update label
    lbl = findobj(getappdata(fig,'ctrl'), 'Tag', [field '_label']);
    if isempty(lbl)
        lbl = findobj(getappdata(fig,'ctrl'), 'Tag', [field(1) '_label']);
    end
    set(lbl, 'String', sprintf('%s: %.3g', field, val));
    restart_sim(fig);
end

function mode_changed(event, fig)
    params = getappdata(fig, 'params');
    params.powerMode = strcmp(event.NewValue.Tag, 'cont');
    setappdata(fig, 'params', params);
    restart_sim(fig);
end

function restart_sim(fig)
    setappdata(fig, 'isRunning', false);
    pause(0.05);
    setappdata(fig, 'isRunning', true);
    initialize_plots(fig);
    animation_loop(fig);
end

function initialize_plots(fig)
    params = getappdata(fig,'params');
    ax1 = getappdata(fig,'ax1'); ax2 = getappdata(fig,'ax2'); ax3 = getappdata(fig,'ax3');
    cla(ax1); cla(ax2); cla(ax3);

    % Time vectors
    dt = 1/(params.f*50);
    t = 0:dt:params.t_end;
    omega = 2*pi*params.f;
    omega_d = omega*sqrt(1-params.zeta^2);

    % Preallocate
    x = zeros(size(t)); x_und = zeros(size(t)); pwr = zeros(size(t));
    x(1)=params.A; x_und(1)=params.A;

    if params.powerOn
        if params.powerMode % Continuous
            for i=2:length(t)
                xn = params.A*exp(-params.zeta*omega*t(i))*cos(omega_d*t(i));
                cmp = (params.A*(1-exp(-params.zeta*omega*t(i)))) * params.powerEfficiency;
                x(i)=xn + cmp*cos(omega_d*t(i));
                pwr(i)=cmp*params.zeta*omega;
                x_und(i)=params.A*cos(omega*t(i));
            end
        else % Pulsed
            th = 0.95*params.A;
            for i=2:length(t)
                xn = x(i-1)*exp(-params.zeta*omega*dt)*cos(omega_d*dt);
                if abs(x(i-1)) < th
                    pf = params.powerEfficiency * 0.1 * params.A;
                    x(i) = xn + sign(xn)*pf;
                    pwr(i) = pf;
                else
                    x(i) = xn;
                    pwr(i) = 0;
                end
                x_und(i)=params.A*cos(omega*t(i));
                x(i)=sign(x(i))*min(abs(x(i)), params.A);
            end
        end
    else % No power
        for i=2:length(t)
            x(i)=params.A*exp(-params.zeta*omega*t(i))*cos(omega_d*t(i));
            x_und(i)=params.A*cos(omega*t(i));
        end
    end

    % Store data
    setappdata(fig,'t',t);
    setappdata(fig,'x',x);
    setappdata(fig,'x_und',x_und);
    setappdata(fig,'pwr',pwr);
    setappdata(fig,'idx',1);

    % Plot oscillation
    axes(ax1);
    plot(t, x, 'r-', 'LineWidth',2); hold on;
    plot(t, x_und, 'g-');
    plot(t, params.A*exp(-params.zeta*omega*t), 'b--');
    plot(t, -params.A*exp(-params.zeta*omega*t), 'b--');
    hold off; grid on;
    xlabel('Time (s)'); ylabel('Displacement');
    title('Oscillation with Compensation');
    legend('Actual','Undamped','Envelope');

    % Plot power
    axes(ax2);
    plot(t, pwr, 'b-', 'LineWidth',2); grid on;
    xlabel('Time (s)'); ylabel('Power');
    title('Power Input');

    % Crystal visualization
    axes(ax3); axis equal off; hold on;
    w = 0.5; h = 2;
    theta = linspace(0,2*pi,50);
    r0 = w/2;
    fill([-w/2 w/2 w/2 -w/2],[-h/2 -h/2 h/2 h/2],[0.9 0.9 1]);
    h_deform = plot(r0*cos(theta), r0*sin(theta), 'b-', 'LineWidth',2);
    setappdata(fig,'h_deform',h_deform);
    setappdata(fig,'theta',theta);
    setappdata(fig,'r0',r0);
    title('Crystal'); hold off;
end

function animation_loop(fig)
    if ~getappdata(fig,'isRunning'), return; end
    t = getappdata(fig,'t');
    x = getappdata(fig,'x');
    pwr = getappdata(fig,'pwr');
    idx = getappdata(fig,'idx');
    h_deform = getappdata(fig,'h_deform');
    theta = getappdata(fig,'theta');
    r0 = getappdata(fig,'r0');
    params = getappdata(fig,'params');

    % Update index
    idx = idx + round(params.speed);
    if idx > numel(t), idx = 1; end

    % Compute deformation
    deform = 0.2 * x(idx);
    a = r0 * (1 + deform);
    b = r0 * (1 - deform*0.5);

    % Update crystal shape
    set(h_deform, 'XData', a*cos(theta), 'YData', b*sin(theta));

    % Increment and store
    setappdata(fig,'idx',idx);

    drawnow limitrate;
    pause(0.001);
    animation_loop(fig);
end