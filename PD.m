%  COMPUTED-TORQUE CONTROLLER

function u = control_djc ( t, x, q_des, qd_des, qdd_des, L_mod, m_mod, mu_mod, g, Kp, Kv )

q(1:2) = x(1:2) ;
qd(1:2) = x(3:4) ;
c1 = cos(q(1)) ;
c2 = cos(q(2)) ;
s2 = sin(q(2)) ;
c12 = cos(q(1)+q(2)) ;
	
M_mod(1,1) = m_mod(1)*L_mod(1)^2;
M_mod(1,2) = 0 ;
M_mod(2,1) = M_mod(1,2) ;
M_mod(2,2) = m_mod(2)*L_mod(2)^2 ;
	
G_mod(1) = (m_mod(1)+m_mod(2))*g*L_mod(1)*c1 + m_mod(2)*g*L_mod(2)*c12 ;
G_mod(2) = m_mod(2)*g*L_mod(2)*c12 ;
V_mod(1) = 0;
V_mod(2) = 0 ;

f_mod(1) = 0 ;
f_mod(2) = 0 ;

q_error = q_des - q ;
qd_error = qd_des - qd ;

fprime = qdd_des' + Kv * qd_error' + Kp * q_error' ;
	
ucol = M_mod*fprime + V_mod' + G_mod' + f_mod';
u = ucol' ;

