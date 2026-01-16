%% UNB FSAE | Wheel Loads, Roll, and Frequency Analysis
% Created by: Cameron Farrell, Last Edit: 2025-11-14 (Ref. Milliken, 1995)
clear; clc;
close all

%% Vehicle Parameters 
g   = 9.81;           
mu_lat = 1.4; %lateral friction coefficient estimate

%Acceleration sweeps
ax_vec = linspace(-0.8,  1.1, 100) * g; %longitudinal accel (- = braking)
ay_vec = linspace(-1.4,  1.4, 100) * g; %lateral accel (+ = left turn)

Ms = 250; %sprung mass (kg)
Mu = 50; %unsprung mass (kg)
m = Ms + Mu; %total mass (kg) 
wf = 0.45; %front fraction of sprung weight    
wr = 1 - wf; %rear fraction of sprung weight

Msf = Ms * wf; %sprung mass at front (kg)
Msr = Ms * wr; %sprung mass at rear (kg)

Muf = Mu * .45; %unsprung mass at front (kg)
Mur = Mu * .55; %unsprung mass at rear (kg)

kt = 120; %tire rate (N/mm) this is fine to estimate (change by 20% only changes total stiffness 4%)

W = m * g; %weight of car (N)      

L = 1.55; %wheelbase (m)
h = 0.325; %CG height (m)        

%Critical Warning: You must ensure the shock has 45mm of usable stroke before it hits the bump rubber.
%Most shocks listed as "57mm stroke" often have a 10-15mm bump rubber, leaving only ~45mm of clean travel.
MRf = 45 / 60; %front motion ratio = shock travel / wheel travel CC TIGON 210x55 (assuming 10mm bumpstop) 
MRr = 45 / 60; %rear motion ratio = shock travel / wheel travel CC TIGON 210x55 (assuming 10mm bumpstop)

ksf = 43.78; %front spring rate (N/mm) 
ksr = 43.78; %rear spring rate (N/mm) 

tf = 1.250; %front track width (m) 
tr = 1.200; %rear track width (m)   

b = wf * L; %CG to rear axle (m)      
a = L - b; %CG to front ale (m)     
c = tr / 2;
d = tr / 2;
 
Zrf = 0.0254; %front RCH (m)    
Zrr = 0.062; %rear RCH (m)          

RCH = Zrf + (a/L)*(Zrr - Zrf); %RCH at true COG (linear interpolation along wheelbase) 
H = h - RCH; %CG above roll axis (m)     

Kwf = MRf^2 * ksf; %front wheel rate (N/mm)
Kwr = MRr^2 * ksr; %rear wheel rate (N/mm)

Krf = Kwf * kt / (Kwf + kt); %front ride rate (N/mm)
Krr = Kwr * kt / (Kwr + kt); %rear ride rate (N/mm)

Kphi_f = (Krf*1000 * tf^2)/2; %front roll stiffness (Nm/rad)
Kphi_r = (Krr*1000 * tr^2)/2; %rear roll stiffness (Nm/rad)

phi_grad = (W * H) / (Kphi_f + Kphi_r); %roll gradient (rad/g)
phi_grad_deg = phi_grad * 180 / pi; %roll angle (deg/g)

dy = 0.001; %lateral CG offset (m)
bump_mult = 1.00; %bump mulitplier (1.00 = no bump, 1.10 = +10% load from bump)

%% Print Example 
ax_print = 1.1 * g; % + forward accel, - braking
ay_print = 1.4 * g; % + cornering left

C = wheelLoads_components(ax_print, ay_print, ...
    W, wf, wr, L, h, tf, tr, a, b, Zrf, Zrr, H, Kphi_f, Kphi_r, dy, bump_mult, g);

%Contribution breakdown at the print point 

%assemble rows: [Static, Longitudinal, Elastic, Geometric, Offset]
FL_terms = [C.Wf_static/2, -C.dW_L/2, -C.dW_F_el, -C.dW_F_geo, C.dW_F_stat];
FR_terms = [C.Wf_static/2, -C.dW_L/2, C.dW_F_el, C.dW_F_geo, -C.dW_F_stat];
RL_terms = [C.Wr_static/2, C.dW_L/2, -C.dW_R_el, -C.dW_R_geo, C.dW_R_stat];
RR_terms = [C.Wr_static/2, C.dW_L/2, C.dW_R_el, C.dW_R_geo, -C.dW_R_stat];

labels = {'Static','Longitudinal','Elastic lat','Geom lat','CG offset'};

%pretty print
fprintf('\nWHEEL LOAD BREAKDOWN @ ax=%.2f g (+: forward), ay=%.2f g (+: left turn)\n', ax_print/g, ay_print/g);
fprintf('%-18s  %10s  %10s  %10s  %10s\n', '','FL','FR','RL','RR');
fmt = '%-18s  %10.1f  %10.1f  %10.1f  %10.1f\n';
for k = 1:numel(labels)
    fprintf(fmt, labels{k}, FL_terms(k), FR_terms(k), RL_terms(k), RR_terms(k));
end

Totals = [sum(FL_terms), sum(FR_terms), sum(RL_terms), sum(RR_terms)];
fprintf('%-18s  %10.1f  %10.1f  %10.1f  %10.1f\n', 'TOTAL (calc)', Totals(1), Totals(2), Totals(3), Totals(4));
fprintf('%-18s  %10.1f  %10.1f  %10.1f  %10.1f\n', 'TOTAL (C.*) ', C.W_FL, C.W_FR, C.W_RL, C.W_RR);

fprintf('Sum of wheels: %.1f N\n', C.W_sum);

%lateral tire forces
Fy_FL = C.W_FL * mu_lat;
Fy_FR = C.W_FR * mu_lat;
Fy_RL = C.W_RL * mu_lat;
Fy_RR = C.W_RR * mu_lat;

fprintf('\nLATERAL TIRE FORCES @ ax=%.2f g (+: forward), ay=%.2f g (+: left turn)\n', ax_print/g, ay_print/g);
fprintf('FL: %.2f N   FR: %.2f N   RL: %.2f N   RR: %.2f N\n', Fy_FL, Fy_FR, Fy_RL, Fy_RR);

fprintf('\nROLL:\n');
fprintf('Roll gradient: %.2f deg/g \n', phi_grad_deg);
fprintf('Front roll stiffness" %.2f Nm/rad \n', Kphi_f);
fprintf('Rear roll stiffness" %.2f Nm/rad \n', Kphi_r);
fprintf('\nFREQUENCIES\n');
[fu_f, fu_r, fs_f, fs_r] = frequencies(kt, Krf, Krr, Kwf, Kwr, Msf, Msr, Muf, Mur);
fprintf('Front sprung ride frequency: %.2f Hz \n', fs_f);
fprintf('Rear sprung ride frequency: %.2f Hz \n', fs_r);
fprintf('Front unsprung ride frequency: %.2f Hz \n', fu_f);
fprintf('Rear unsprung ride frequency: %.2f Hz \n', fu_r);


%% Plots 
[AX, AY] = meshgrid(ax_vec, ay_vec);

[W_FL, W_FR, W_RL, W_RR] = wheelLoads(AX, AY, W, wf, wr, L, h, tf, tr, a, b, ...
    Zrf, Zrr, H, Kphi_f, Kphi_r, dy, bump_mult, g);

% Store in cell array for looping
WheelData = {W_FL, 'Front Left'; W_FR, 'Front Right'; W_RL, 'Rear Left'; W_RR, 'Rear Right'};

figure('Name', 'Wheel Loads & Lift Check', 'Color', 'w'); 
tiledlayout(2,2, 'Padding', 'compact', 'TileSpacing', 'compact');

for i = 1:4
    nexttile;
    % CONTOUR PLOT
    contourf(AX/g, AY/g, WheelData{i,1}, 20, 'LineStyle', 'none'); 
    hold on;
    %Plot the "Wheel Lift" boundary (0 Newtons) in thick black
    [C, h_cont] = contour(AX/g, AY/g, WheelData{i,1}, [0 0], 'k', 'LineWidth', 2);
    clabel(C, h_cont, 'FontSize', 8, 'Color', 'w', 'FontWeight', 'bold');
    
    colorbar; grid on;
    title(WheelData{i,2}); 
    xlabel('Longitudinal (g)'); ylabel('Lateral (g) ''+'' = left turn');
end
sgtitle('Wheel Loads (N)');
%% Functions 
function [fu_f, fu_r, fs_f, fs_r] = frequencies(Kt, Krf, Krr, Kwf, Kwr, Msf, Msr, Muf, Mur)

    fs_f = sqrt(Krf*1000 / (0.5*Msf)) / (2*pi); %front sprung undamped natural frequency (ride freq.)
    fs_r = sqrt(Krr*1000 / (0.5*Msr)) / (2*pi); %rear sprung undamped natural frequency (ride freq.)

    fu_f = sqrt((Kwf + Kt)*1000 / (0.5*Muf)) / (2*pi); %front unsprung undamped natural frequency (ride freq.)
    fu_r = sqrt((Kwr + Kt)*1000 / (0.5*Mur)) / (2*pi); %rear unsprung undamped natural frequency (ride freq.)

end

function [W_FL, W_FR, W_RL, W_RR] = wheelLoads(ax, ay, ...
    W, wf, wr, L, h, tf, tr, a, b, Zrf, Zrr, H, Kphi_f, Kphi_r, dy, bump_mult, g)

    Wf_stat = (W * wf) * bump_mult;   
    Wr_stat = (W * wr) * bump_mult;   
    dW_F_stat = Wf_stat .* (dy ./ tf); %accounting for CG offset
    dW_R_stat = Wr_stat .* (dy ./ tr);  

    % +ax = forward accel (rear gains, front loses)
    dW_L = (W * h / (g * L)) .* ax;    

    %roll moment and elastic distribution
    M_phi = (W/g) .* ay .* H;  % (Nm)      
    frac_f = Kphi_f / (Kphi_f + Kphi_r); %fraction of roll moment on front axle (ref. Milliken)
    frac_r = Kphi_r / (Kphi_f + Kphi_r); %fraction of roll moment on rear axle

    dW_F_el = (frac_f .* M_phi) ./ tf;   
    dW_R_el = (frac_r .* M_phi) ./ tr;   

    %geometric weight transfer (RC)
    dW_F_geo = (W/g) .* ay .* (b/L) .* (Zrf./tf);
    dW_R_geo = (W/g) .* ay .* (a/L) .* (Zrr./tr);

    %total lateral per axle
    dW_F = dW_F_el + dW_F_geo; %+ adds to right, sub from left
    dW_R = dW_R_el + dW_R_geo;   

    %total longitudinal per axle
    Wf_total = Wf_stat - dW_L;   
    Wr_total = Wr_stat + dW_L;   

    %split to each wheel
    W_FL = (Wf_total/2) - dW_F + dW_F_stat;   
    W_FR = (Wf_total/2) + dW_F - dW_F_stat;   
    W_RL = (Wr_total/2) - dW_R + dW_R_stat;   
    W_RR = (Wr_total/2) + dW_R - dW_R_stat;   

end

%This is the same as function above just using scalar values to work for
%print output instead of a vector range of acceleration values
function C = wheelLoads_components(ax, ay, W, wf, wr, L, h, tf, tr,...
    a, b, Zrf, Zrr, H, Kphi_f, Kphi_r, dy, bump_mult, g)

    %Static loads (with bump)
    C.Wf_static = (W * wf) * bump_mult;   
    C.Wr_static = (W * wr) * bump_mult;  

    %Load transfer from CG offset
    C.dW_F_stat = C.Wf_static * (dy / tf); 
    C.dW_R_stat = C.Wr_static * (dy / tr);     
    
    %Load transfer from Longitudinal accel
    C.dW_L = (W * h / (g * L)) * ax;    
   
    %Laterl components (geometric and elastic)
    C.M_phi = (W/g) * ay * H;             
    frac_f = Kphi_f / (Kphi_f + Kphi_r);
    frac_r = Kphi_r / (Kphi_f + Kphi_r);

    C.dW_F_el = (frac_f * C.M_phi) / tf;  
    C.dW_R_el = (frac_r * C.M_phi) / tr;  

    C.dW_F_geo = (W/g) * ay * (b/L) * (Zrf/tf); 
    C.dW_R_geo = (W/g) * ay * (a/L) * (Zrr/tr); 

    %Total load each wheel
    C.W_FL = C.Wf_static/2 - C.dW_L/2 - C.dW_F_el - C.dW_F_geo + C.dW_F_stat;
    C.W_FR = C.Wf_static/2 - C.dW_L/2 + C.dW_F_el + C.dW_F_geo - C.dW_F_stat;
    C.W_RL = C.Wr_static/2 + C.dW_L/2 - C.dW_R_el - C.dW_R_geo + C.dW_R_stat;
    C.W_RR = C.Wr_static/2 + C.dW_L/2 + C.dW_R_el + C.dW_R_geo - C.dW_R_stat;

    C.W_sum = C.W_FL + C.W_FR + C.W_RL + C.W_RR;
end

function bumpDS = createBumpScenario(amplitude, t_start, width, speed, wheelbase)
% CREATEBUMPSCENARIO Generates FSAE bump signals for Signal Editor
%
% Inputs:
%   amplitude - Height of the bump (meters)
%   t_start   - Time when front wheels hit the bump (seconds)
%   width     - Duration of the bump event (seconds) [Length / Speed]
%   speed     - Car speed (m/s)
%   wheelbase - Distance between front and rear axles (m)
%
% Output:
%   bumpDS    - A Simulink.SimulationData.Dataset object compatible 
%               with Signal Editor

    % 1. Simulation Setup
    T_total = t_start + width + (wheelbase/speed) + 4; % Add buffer time
    dt = 0.01; % 1 kHz sample rate
    time = 0:dt:T_total;
    
    % 2. Calculate Delay for Rear Wheels (Time = Distance / Speed)
    t_delay = wheelbase / speed;
    t_start_rear = t_start + t_delay;
    
    % 3. Define the Helper Function (The Half-Sine Logic)
    % y = A * sin(pi/width * (t - t0))
    % We use pi (not 2pi) because we only want the first half-hump (0 to pi)
    getBump = @(t, t0) (amplitude * sin((pi/width) * (t - t0))) .* ...
                       (t >= t0 & t <= (t0 + width));

    % 4. Generate Data
    % Front Wheels (Zg1 = Left, Zg2 = Right)
    zg_front_data = getBump(time, t_start);
    
    % Rear Wheels (Zg3 = Left, Zg4 = Right) - Delayed
    zg_rear_data = getBump(time, t_start_rear);
    
    % 5. Create Timeseries Objects
    % Assuming a "Heave" bump (both left and right hit simultaneously)
    ts_Zg1 = timeseries(zg_front_data, time, 'Name', 'Zg1');
    ts_Zg2 = timeseries(zg_front_data, time, 'Name', 'Zg2');
    ts_Zg3 = timeseries(zg_rear_data,  time, 'Name', 'Zg3');
    ts_Zg4 = timeseries(zg_rear_data,  time, 'Name', 'Zg4');

    % 6. Pack into a Dataset (Best format for Signal Editor)
    bumpDS = Simulink.SimulationData.Dataset;
    bumpDS = bumpDS.addElement(ts_Zg1);
    bumpDS = bumpDS.addElement(ts_Zg2);
    bumpDS = bumpDS.addElement(ts_Zg3);
    bumpDS = bumpDS.addElement(ts_Zg4);
    
end

% amplitude=0.05m, start=1s, width=0.2s, speed=10m/s, wheelbase
myBumpData = createBumpScenario(0.25, 1, 0.2, 10, L);