function gimbal_lab_selftest(outputDir)
% Meaningful offline checks: exporter units, missing data, time validation,
% integral model identification and single-frequency phase estimation.
if nargin<1, outputDir = fullfile(fileparts(mfilename('fullpath')),'results','selftest'); end
if ~exist(outputDir,'dir'), mkdir(outputDir); end
t = (0:0.001:8)';
% Deterministic independently excited rate; derive the required input.
w = 0.3*sin(2*pi*0.7*t)+0.2*sin(2*pi*1.9*t);
wdot = 0.3*2*pi*0.7*cos(2*pi*0.7*t)+0.2*2*pi*1.9*cos(2*pi*1.9*t);
gain = 4; damping = 1.5; friction = 0.12; bias = 0.03;
u = (wdot+damping*w+friction*tanh(w/0.05)-bias)/gain;
theta = cumtrapz(t,w); thetaRef = theta+deg2rad(0.5);
T = table(t,theta,thetaRef,w,u,'VariableNames', ...
    {'t_s','theta_rad','theta_ref_rad','omega_rad_s','u_cmd'});
normalized = fullfile(outputDir,'synthetic.csv'); writetable(T,normalized);
report = gimbal_analyze_csv(normalized,fullfile(outputDir,'analysis'));
assert(abs(report.rmse_deg-0.5)<1e-8,'Angle RMSE failed.');
fit = gimbal_fit_axis(normalized,struct('torquePerCommandUnit',0.08));
assert(abs(fit.gain_rad_s2_per_unit-gain)/gain<0.02,'Gain recovery failed.');
assert(abs(fit.damping_per_s-damping)/damping<0.02,'Damping recovery failed.');
assert(abs(fit.J_effective_kg_m2-0.02)<0.0004,'Inertia conversion failed.');
% Match EmberProbe 0.8 archive export time format and typed headers.
stamp = datetime(2026,10,4,'TimeZone','UTC')+seconds(t(1:20));
stamp.Format = "yyyy-MM-dd'T'HH:mm:ss.SSS'Z'";
raw = table(string(stamp),rad2deg(theta(1:20)), ...
    'VariableNames',{'time','angle:f32'});
raw{4,2} = NaN;
rawFile = fullfile(outputDir,'emberprobe_shape.csv'); writetable(raw,rawFile);
mapping.timeColumn = 'time'; mapping.timeUnit = 'iso8601';
mapping.signals.theta_rad = struct('column','angle:f32','scale',pi/180);
imported = gimbal_import_csv(rawFile,mapping);
assert(max(abs(imported.t_s-t(1:20)))<1e-9,'ISO time conversion failed.');
assert(isnan(imported.theta_rad(4)),'Missing sample must remain missing.');
assert(abs(imported.theta_rad(end)-theta(20))<1e-9,'Degree conversion failed.');
raw.time(5) = raw.time(4); writetable(raw,rawFile);
rejected = false;
try, gimbal_import_csv(rawFile,mapping); catch, rejected = true; end
assert(rejected,'Duplicate timestamp must be rejected.');
phase = T; phase.theta_ref_rad = 0.1*sin(2*pi*t);
phase.theta_rad = 0.08*sin(2*pi*t-deg2rad(20));
phaseFile = fullfile(outputDir,'sine.csv'); writetable(phase,phaseFile);
phaseReport = gimbal_analyze_csv(phaseFile,fullfile(outputDir,'phase'),struct('frequencyHz',1));
assert(abs(phaseReport.sine_phase_lag_deg-20)<1e-6,'Phase estimate failed.');
assert(abs(phaseReport.sine_amplitude_ratio-0.8)<1e-6,'Amplitude estimate failed.');
demo = gimbal_lqr_eso_demo(fullfile(outputDir,'demo'));
assert(all(isfinite(demo.rmse_deg)),'Demo must remain finite.');
assert(demo.disturbance_rmse_deg(3)<demo.disturbance_rmse_deg(2), ...
    'ESO should improve this synthetic disturbance case.');
fprintf('PASS: import, time rejection, missing data, metrics, fit, phase, sampled simulation.\n');
end
