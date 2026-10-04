function results = gimbal_lqr_eso_demo(outputDir)
% Teaching-only sampled simulation; all plant parameters below are synthetic.
% NEVER copy these gains/current limits to the vehicle.
% Base MATLAB only. Analytic continuous CARE for a 2-state diagonal-Q plant.
if nargin<1, outputDir = fullfile(fileparts(mfilename('fullpath')),'results','demo'); end
if ~exist(outputDir,'dir'), mkdir(outputDir); end
rng(7);
J = 0.02; viscous = 0.04; Ts = 0.001;
Q = diag([16,1]); R = 4;
K = [sqrt(Q(1,1)/R), -viscous+sqrt(viscous^2+2*J*sqrt(Q(1,1)/R)+Q(2,2)/R)];
A = [0,1;0,-viscous/J]; Bu = [0;1/J];
discrete = expm([A,Bu;zeros(1,3)]*Ts);
Ad = discrete(1:2,1:2); Bd = discrete(1:2,3);
assert(all(abs(eig(Ad-Bd*K))<1),'Sampled feedback is unstable.');
observerW = 50; damping = viscous/J;
% Exact repeated observer poles at -observerW, including known damping.
L = [3*observerW-damping; ...
    3*observerW^2-damping*(3*observerW-damping); observerW^3];
Ae = [0,1,0;0,-damping,1;0,0,0]; Be = [0;1/J;0]; C = [1,0,0];
assert(all(abs(eig(eye(3)+Ts*(Ae-L*C)))<1),'Euler observer is unstable.');
t = (0:Ts:5)'; n = numel(t); f = 1;
amplitude = deg2rad(5);
ref = amplitude*sin(2*pi*f*t);
refW = amplitude*2*pi*f*cos(2*pi*f*t);
refA = -amplitude*(2*pi*f)^2*sin(2*pi*f*t);
disturbance = 0.06*(t>=1.5 & t<3.5);
noise = deg2rad(0.003)*randn(n,1);
limit = 0.6; slewRate = 6;
names = ["LQR feedback","LQR + feedforward","LQR + feedforward + ESO"];
colors = lines(3);
angles = zeros(n,3); commands = zeros(n,3); estimates = zeros(n,3);
rows = repmat(struct,3,1);
for scenario = 1:3
    x = [0;0]; z = [0;0;0]; lastCommand = 0;
    for k = 1:n
        angles(k,scenario) = x(1);
        % ESO uses previous FINAL command, never the unsaturated request.
        measuredAngle = x(1)+noise(k);
        z = z+Ts*(Ae*z+Be*lastCommand+L*(measuredAngle-C*z));
        rawCommand = -K*([measuredAngle;x(2)]-[ref(k);refW(k)]);
        if scenario>=2, rawCommand = rawCommand+J*refA(k)+viscous*refW(k); end
        compensation = min(0.2,max(-0.2,-J*z(3)));
        if scenario==3, rawCommand = rawCommand+compensation; end
        command = min(limit,max(-limit,rawCommand));
        command = min(lastCommand+slewRate*Ts,max(lastCommand-slewRate*Ts,command));
        commands(k,scenario) = command;
        estimates(k,scenario) = J*z(3);
        lastCommand = command;
        if k<n, x = Ad*x+Bd*(command+disturbance(k)); end
    end
    errorDeg = rad2deg(angles(:,scenario)-ref);
    afterStartup = t>=0.5;
    disturbanceInterval = t>=2 & t<3.5;
    rows(scenario).scenario = names(scenario);
    rows(scenario).rmse_deg = sqrt(mean(errorDeg(afterStartup).^2));
    rows(scenario).disturbance_rmse_deg = sqrt(mean(errorDeg(disturbanceInterval).^2));
    rows(scenario).peak_torque_nm = max(abs(commands(:,scenario)));
    rows(scenario).simulated_only = true;
end
results = struct2table(rows);
writetable(results,fullfile(outputDir,'demo_metrics.csv'));
writetable(table(t,ref,angles(:,1),angles(:,2),angles(:,3),disturbance, ...
    'VariableNames',{'t_s','theta_ref_rad','lqr_rad','lqr_ff_rad','lqr_ff_eso_rad','disturbance_nm'}), ...
    fullfile(outputDir,'demo_traces.csv'));
fig = figure('Visible','off','Color','w','Position',[100 100 1200 900]);
cleanupFig = onCleanup(@() close(fig));
tiledlayout(4,1);
nexttile; plot(t,rad2deg(ref),'k--'); hold on;
for scenario = 1:3
    plot(t,rad2deg(angles(:,scenario)),'Color',colors(scenario,:));
end
ylabel('Angle (deg)'); legend(["Reference",names],'Location','eastoutside'); grid on;
title('SYNTHETIC TEACHING SIMULATION - no measured vehicle data');
nexttile; colororder(colors); plot(t,rad2deg(angles-ref)); ylabel('Error (deg)'); grid on;
nexttile; colororder(colors); plot(t,commands); ylabel('Torque (N m)'); grid on;
nexttile; plot(t,disturbance,'k--',t,estimates(:,3));
ylabel('Disturbance (N m)'); xlabel('Time (s)'); legend('Actual','ESO estimate'); grid on;
exportgraphics(fig,fullfile(outputDir,'demo.png'),'Resolution',160);
fprintf('SYNTHETIC ONLY: J=%g, B=%g, K=[%g %g], Ts=%g\n',J,viscous,K,Ts);
disp(results);
end
