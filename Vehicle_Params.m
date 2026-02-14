
function VP = Vehicle_Params()

    %% INPUTS (Subject to change)
    VP.g = 9.81;
    VP.BB = 0.6;        % 60% front Brake Bias

    % Mass & Inertia
    VP.Ms = 265;           % Sprung Mass (kg)
    VP.Mu = 55;            % Total Unsprung Mass (kg)
    VP.wf = 0.45;          % Front Weight Distribution (decimal)
    
    % Geometry
    VP.L = 1.55;       % Wheelbase (m)          
    VP.h = 0.325;      % CG Height (m)
    VP.tf = 1.262;      % Front Track (m)
    VP.tr = 1.263;      % Rear Track (m)
    
    VP.Zrf = 0.024;     % Front Roll Center Height (m)
    VP.Zrr = 0.078;      % Rear Roll Center Height (m)
    VP.dy = 0.001;      % Lateral CG Offset (m)

    % Stiffness (N/mm * 1000 = N/m)
    VP.kt = 120 * 1000; % Tire Stiffness      
    VP.ksf = 52.5 * 1000; % Front Spring Stiffness300lb/in
    VP.ksr = 61.3 * 1000; % Rear Spring Stiffness 350lb/in
    
    %CANE CREEK COIL-IL G2
    VP.MRf = 45 / 66;   % Motion Ratio Front
    VP.MRr = 50 / 62;   % Motion Ratio Rear

    %% CALCULATED VALUES

    % Derived Geometry
    VP.wr = 1 - VP.wf;
    VP.b = VP.wf * VP.L;    % CG to Rear
    VP.a = VP.L - VP.b;     % CG to Front
    
    % Derived Mass
    VP.W = (VP.Ms + VP.Mu) * VP.g;
    VP.Msf = VP.Ms * VP.wf;
    VP.Msr = VP.Ms * VP.wr;
    VP.Muf = (VP.Mu * 0.5);
    VP.Mur = (VP.Mu * 0.5);

    % Derived Stiffness (Wheel Rates & Ride Rates)
    VP.Kwf = VP.MRf^2 * VP.ksf;    % Wheel Rate Front (N/m)
    VP.Kwr = VP.MRr^2 * VP.ksr;    % Wheel Rate Rear (N/m)
    
    VP.Krf = (VP.Kwf * VP.kt) / (VP.Kwf + VP.kt); % Ride Rate Front
    VP.Krr = (VP.Kwr * VP.kt) / (VP.Kwr + VP.kt); % Ride Rate Rear

    VP.Wheel_Sag_f = 0.5 * VP.Msf * 9.81 / (VP.Kwf / 1000); %(mm)
    VP.Wheel_Sag_r = 0.5 * VP.Msr * 9.81 / (VP.Kwr / 1000); %(mm)

    % Roll Stiffness (Nm/rad)
    % Note: 0.5 * K_ride * Track^2 is the approx. for stiff independent suspension
    VP.Kphi_f = 0.5 * VP.Krf * VP.tf^2; 
    VP.Kphi_r = 0.5 * VP.Krr * VP.tr^2;
    
    % Geometric Roll Constants
    % Interpolate RC at CG
    VP.RCH = VP.Zrf + (VP.a / VP.L) * (VP.Zrr - VP.Zrf); 
    VP.H = VP.h - VP.RCH; % Roll Moment Arm
    
    % Roll Gradient
    VP.phi_grad = (VP.W * VP.H) / (VP.Kphi_f + VP.Kphi_r);  % (rad/g)
    VP.phi_grad_deg = VP.phi_grad * 180 / pi;               % (deg/g)

    VP.fs_f = sqrt(VP.Krf / (0.5*VP.Msf)) / (2*pi); %front sprung undamped natural frequency (ride freq.)
    VP.fs_r = sqrt(VP.Krr / (0.5*VP.Msr)) / (2*pi); %rear sprung undamped natural frequency (ride freq.)

    VP.fu_f = sqrt((VP.Kwf + VP.kt) / (0.5*VP.Muf)) / (2*pi); %front unsprung undamped natural frequency (ride freq.)
    VP.fu_r = sqrt((VP.Kwr + VP.kt) / (0.5*VP.Mur)) / (2*pi); %rear unsprung undamped natural frequency (ride freq.)
end