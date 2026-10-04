function report = gimbal_analyze_csv(csvFile, outputDir, options)
% Analyze a NORMALIZED CSV. Required columns: t_s, theta_rad, theta_ref_rad.
% Optional: omega_rad_s, u_cmd, saturated (actual MCU limiter flag).
% options.frequencyHz: known single sine frequency; omit for arbitrary motion.
% options.commandLimit: verified command-unit limit (no GM6020 default assumed).
if nargin < 3, options = struct; end
T = readtable(csvFile, 'VariableNamingRule','preserve');
required = {'t_s','theta_rad','theta_ref_rad'};
assert(all(ismember(required,T.Properties.VariableNames)), 'Missing required columns.');
t = T.t_s;
assert(numel(t)>=10 && all(isfinite(t)) && all(diff(t)>0), ...
    'At least 10 rows and finite, strictly increasing time are required.');
dt = diff(t);
errorRad = T.theta_rad - T.theta_ref_rad;
valid = isfinite(errorRad);
assert(nnz(valid)>=10, 'Too few valid paired angle samples.');
report = struct;
report.rows = height(T);
report.valid_angle_rows = nnz(valid);
report.duration_s = t(end)-t(1);
report.median_sample_hz = 1/median(dt);
report.p95_interval_s = localPercentile(dt,95);
report.max_interval_s = max(dt);
report.large_gaps = nnz(dt>3*median(dt));
report.interval_jitter_ratio = std(dt)/mean(dt);
report.rmse_deg = rad2deg(sqrt(mean(errorRad(valid).^2)));
report.p95_abs_error_deg = rad2deg(localPercentile(abs(errorRad(valid)),95));
report.max_abs_error_deg = rad2deg(max(abs(errorRad(valid))));
report.mean_error_deg = rad2deg(mean(errorRad(valid)));
report.note = 'Host timestamps and non-atomic SWD reads limit fast-dynamics inference.';
if ismember('u_cmd',T.Properties.VariableNames)
    u = T.u_cmd;
    report.command_peak = max(abs(u),[],'omitnan');
    report.command_rms = sqrt(mean(u.^2,'omitnan'));
    if isfield(options,'commandLimit')
        assert(options.commandLimit>0 && isfinite(options.commandLimit), ...
            'commandLimit must be positive in the SAME units as u_cmd.');
        finiteU = isfinite(u);
        report.near_command_limit_fraction = mean(abs(u(finiteU))>=0.98*options.commandLimit);
    end
end
if ismember('saturated',T.Properties.VariableNames)
    flag = T.saturated;
    report.saturated_fraction = mean(flag(isfinite(flag))~=0);
end
if isfield(options,'frequencyHz')
    f = options.frequencyHz;
    assert(isscalar(f) && isfinite(f) && f>0, 'frequencyHz must be positive.');
    if f >= report.median_sample_hz/10 || f*report.duration_s<3 || report.large_gaps>0
        report.sine_note = 'Insufficient samples/cycles or gaps: no sine phase estimate.';
    else
        tv = t(valid);
        basis = [sin(2*pi*f*tv),cos(2*pi*f*tv),ones(size(tv))];
        refFit = basis\T.theta_ref_rad(valid);
        outFit = basis\T.theta_rad(valid);
        amplitude = hypot(refFit(1),refFit(2));
        if amplitude>1e-8
            phaseRef = atan2(refFit(2),refFit(1));
            phaseOut = atan2(outFit(2),outFit(1));
            lag = atan2(sin(phaseRef-phaseOut),cos(phaseRef-phaseOut));
            report.sine_amplitude_ratio = hypot(outFit(1),outFit(2))/amplitude;
            report.sine_phase_lag_deg = rad2deg(lag);
            report.sine_equivalent_lag_ms = 1000*lag/(2*pi*f);
            report.sine_note = 'Single-frequency phase lag modulo one period; not pure transport delay.';
        end
    end
end
if ~exist(outputDir,'dir'), mkdir(outputDir); end
fid = fopen(fullfile(outputDir,'metrics.json'),'w');
assert(fid>=0,'Cannot write metrics.json');
cleanup = onCleanup(@() fclose(fid));
fprintf(fid,'%s\n',jsonencode(report,'PrettyPrint',true));
clear cleanup
fig = figure('Visible','off','Color','w','Position',[100 100 1100 800]);
cleanupFig = onCleanup(@() close(fig));
tiledlayout(3,1);
nexttile; plot(t,rad2deg(T.theta_ref_rad),'--',t,rad2deg(T.theta_rad));
ylabel('Angle (deg)'); legend('Reference','Measured'); grid on;
nexttile; plot(t,rad2deg(errorRad)); ylabel('Error (deg)'); grid on;
nexttile;
if ismember('u_cmd',T.Properties.VariableNames)
    plot(t,T.u_cmd); ylabel('Command (declared units)');
else
    plot(t(2:end),1000*dt); ylabel('Sample interval (ms)');
end
xlabel('Time (s)'); grid on;
exportgraphics(fig,fullfile(outputDir,'tracking.png'),'Resolution',160);
disp(report);
end

function value = localPercentile(x,p)
x = sort(x(isfinite(x)));
position = 1 + (numel(x)-1)*p/100;
lo = floor(position); hi = ceil(position);
value = x(lo) + (position-lo)*(x(hi)-x(lo));
end
