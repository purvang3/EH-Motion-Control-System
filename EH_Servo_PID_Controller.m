% Filename: EH_Servo_PID_Controller.m
function i_cmd = EH_Servo_PID_Controller(t)
global y_d ydot_d y ydot K_fb K_p K_i K_d u_i
% Desired cylinder position command to PID controller
if t<1.0 
    y_d = 0.2 ; 
    ydot_d = 0.0 ; 
elseif (t>= 1.0 && t<=2.0) 
    y_d= 0.2 + 0.5 *(t-1.0) ;
    ydot_d = 0.5 ;
elseif (t> 2.0 && t<=3.0) 
    y_d = 0.5 ; 
    ydot_d = 0.0 ;
elseif (t> 3.0 && t<=4) 
    y_d = 0.7 - 0.5 * (t - 3.0) ; 
    ydot_d = - 0.5 ;
else
    y_d = 0.2 ; 
    ydot_d = 0.0 ;
end
% Cylinder position sensor reading; feedback signal.
y_fb = K_fb * y;
ydot_fb = K_fb * ydot ;
% PID control algorithm
u_i = u_i + (y_d - y_fb) ;
i_cmd = K_p *(y_d - y_fb) + K_i * u_i + K_d * (ydot_d - ydot_fb) ;
return ;