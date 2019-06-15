
% Filename: EH_Servo_Sim.m
global D_p w_shaft p_max 
global Q_rv x_vmax T_vmax p_r c_v w_n tau_a K_a K_t i_v x_v 
global beta V_hose_pv V_hose_va V_hose_vb A_a A_b l_cyl m_p c_p m_l F_load
global Q_p Q_v Q_r 
global p_p p_a p_b p_t
global y_d ydot_d y ydot K_fb K_p K_i K_d u_i i_cmd
% Parameters of the components of the EH hydraulic circuit
D_p = 20 * 10^-6 ; % mˆ3/rev 
p_max = 20.685 *10^6 ; % [N/mˆ2] = Pa 
x_vmax = 10 * 10^-3 ; % m
c_v = 0.60 ; % damping ratio of the valve spool motion
tau_a = 0.01 ; % sec 
K_a = 1.0 ; % current amp gain 
K_t = 1.0 ; % current to torque gain 
Q_rv = 1.0 * 10^-3 ; % mˆ3/sec 
p_r = 6.895 *10^6 ; % Pa = N/mˆ2
beta = 1.5 *(10^9) ; % Bulk modulus 
V_hose_pv = 0.001 ; % [mˆ3] 
V_hose_va = 0.001 ; % [mˆ3]
V_hose_vb = 0.001 ; % [mˆ3]
A_a = 0.001 ;
A_b = 0.0005 ; 
l_cyl = 1.0 ; % [m] 
m_p = 10 ; % kg 
m_l = 990 ; % kg 
c_p = 100.0 ;
% PID controller parameters and initialization
K_fb = 1.0 ; 
K_p = 3.0 ; 
K_i = 0.0 ;
K_d = 0.0 ; 
u_i = 0.0 ;
% Input conditions
w_shaft = 3000/60 ; % rev/sec 
F_load = (m_p + m_l) * 9.81 ; % kg m/secˆ2 = N
% Initial conditions on head and rod-end volume of cylinder.
i_v = 0.0 ; 
x_v = 0.0 ;
xdot_v = 0.0 ;
y = 0.2 ; % Initial cylinder position 
ydot = 0.0 ; % Initial cylinder velocity 
y_d = 0.2 ; 
ydot_d = 0.0 ; % Initial pressures, atmospheric pressure: 101 kPa 
p_p = 101 * 10^3; % Pump pressure: Pa 
p_a = F_load/A_a ; 
p_b = 101 * 10^3 ;
p_t = 101 *10^3 ; % kPa Tank pressure 
Q_r = 0.0 ;
Q_v = 0.0 ;
% Simulation over a time period: [t_0, t_f]
t_0 =0.0; 
t_f =5.0 ;
t_sample = 0.001 ;
z=zeros(8,1);
z(1) = p_p ;
z(2) = i_v ; 
z(3) = x_v ;
z(4) = xdot_v ; 
z(5) = p_a ; 
z(6) = p_b ; 
z(7) = y; 
z(8) = ydot ;
%y_out = [ Q_p Q_v Q_r p_p p_a p_b i_cmd x_v*10^3 y_d y ]; 
y_out = [ 0.0 0.0 0.0 p_p p_a p_b 0.0 0.0 0.2 0.2 ];
for (t=t_0: t_sample:t_f)
% Solve ODEs...
t_span=[t,t+t_sample] ; 
i_cmd = EH_Servo_PID_Controller(t) ; 
[T,z1] = ode45('EH_Servo_Dynamics',t_span, z); 
[m,n]=size(z1); 
z(:)=[z1(m,:)] ; 
t;
y_out=[y_out ; Q_p Q_v Q_r z(1) z(5) z(6) i_cmd z(3)*100 y_d z(7) ]; 
end
[m,n]=size(y_out);
t_inc = (t_f-t_0)/m ; 
tout=t_0:t_inc:t_f-t_inc; 
tout = tout' ;
figure(1) ;
subplot(2,2,1) ; plot(tout, y_out(:,1), 'k',tout, y_out(:,2), 'b',tout, y_out(:,3), 'm'); xlabel('Time (sec)') ; ylabel('Flow rate (m^3/sec)') ; legend('Q_p','Q_v', 'Q_r');
subplot(2,2,2) ; plot(tout, y_out(:,4), 'k',tout, y_out(:,5), 'b',tout, y_out(:,6), 'g'); xlabel('Time (sec)') ; ylabel('Pressure [Pa]') ; legend('p_p','p_a','p_b');
subplot(2,2,3) ; plot(tout, y_out(:,7), 'k',tout, y_out(:,8), 'b'); xlabel('Time (sec)') ; ylabel('Valve currentcmd and spool position') ; legend('i_cmd','x_v');
subplot(2,2,4) ; plot(tout, y_out(:,9), 'k',tout, y_out(:,10),'b'); xlabel('Time (sec)') ; ylabel('Cylinder position: cmd and mea.') ; legend('y_d','y');
