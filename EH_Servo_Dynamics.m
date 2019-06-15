%%% Filename: EH_Servo_Dynamics.m
function zdot = EH_Servo_Dynamics(t,z)
global D_p w_shaft p_max 
global Q_rv x_vmax p_r c_v w_n tau_a K_a K_t i_v x_v 
global beta V_hose_pv V_hose_va V_hose_vb A_a A_b l_cyl m_p c_p m_l F_load
global Q_p Q_v Q_r 
global p_p p_a p_b p_t
global y_d ydot_d y ydot K_fb K_p K_i K_d u_i i_cmd
T_vmax = 10;
p_p = z(1) ;
i_v = z(2); 
x_v = z(3) ; 
xdot_v = z(4) ; 
p_a = z(5) ;
p_b = z(6) ; 
y = z(7) ;
ydot = z(8) ;
zdot=zeros(8,1) ;
% ODEs....
% Pump flow rate and pressure, and relief-valve model (instantaneously % acting)
Q_p = D_p * w_shaft ;
if p_p < p_max 
    zdot(1)= (beta/V_hose_pv)*(Q_p - Q_v) ; 
    Q_r = 0.0 ; 
else
    zdot(1) = 0.0 ; 
    Q_r = Q_p - Q_v ; 
end
% Amplifier and servo valve electrical dynamics
zdot(2) = K_a * (1/tau_a)* (- z(2) + i_cmd) ; 
T_v = K_t * z(2) ;
% Servo valve stage two spool position and torque relationship
if ((x_v/x_vmax) <= 0.4) 
    w_n = 2*pi*75 ; % rad/sec 
else
    w_n = 2*pi* (75 - ( ((x_v/x_vmax) - 0.4) * ((75-25)/(1.0-0.4)) )); % rad/sec
end
zdot(3) = z(4) ; 
zdot(4) = (- 2 * c_v * w_n * z(4) - w_n^2 * z(3)) + (x_vmax/T_vmax) * w_n^2 * T_v ;
if x_v >= 0.0
    Q_pa = Q_rv * abs(x_v/x_vmax) * ((p_p - p_a)/p_r)^0.5 ;
    Q_bt = Q_rv * abs(x_v/x_vmax) * ((p_b - p_t)/p_r)^0.5 ; 
    Q_v = Q_pa ; 
    zdot(5) = (beta/(V_hose_va + y * A_a))*(Q_pa - ydot * A_a) ; 
    zdot(6) = (beta/(V_hose_vb + (l_cyl - y) * A_b))*(-Q_bt + ydot * A_b); 
else
    Q_at = Q_rv * abs(x_v/x_vmax) * ((p_a - p_t)/p_r)^0.5 ;
    Q_pb = Q_rv * abs(x_v/x_vmax) * ((p_p - p_b)/p_r)^0.5 ;
    Q_v = Q_pb ;
    zdot(5) = (beta/(V_hose_va + y * A_a))*(-Q_at - ydot * A_a) ; 
    zdot(6) = (beta/(V_hose_vb + (l_cyl - y) * A_b))*(Q_pb + ydot * A_b); 
end

% Piston and load inertia dynamics
zdot(7) = z(8); 
zdot(8) = (1/(m_p+m_l))*(-c_p * z(8) + p_a * A_a - p_b * A_b - F_load) ;
return; 
