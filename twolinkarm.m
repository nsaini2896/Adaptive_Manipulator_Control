%    THIS PROGRAM SIMULATES THE TWO-LINK ARM USING A
%    DISCRETE-TIME CONTROLLER

Tint = 0.1 ;						% integration period
Tcon = 0.1 ;						% control period
Tplot = 0.1 ;						% plotting period
Ncon = round(Tcon/Tint) ;
Nplot = round(Tplot/Tint) ;
Ttotal = 5.0-Tcon ;
1
% ACTUAL PARAMETERS
L = [0.5; 0.5];  % link length (m)
m = [1; 1] ;  % link mass (kg)
mu = [0; 0] ;  % viscous friction coefficient (N-m/rad/sec)
g = 9.8 ;  % gravitational acceleration (m/2^2)

% MODEL PARAMETERS
L_mod = [0.5; 0.5] ;
m_mod = [1; 1] ;
mu_mod = [0; 0] ;

% SET THE CONTROL GAINS (Exer 10.3)
Kp = [40 0; 0 30] ;
Kv = [2 0; 0 20] ;

%  INITIALIZE TIME, STATE, AND CONTROL

t = 0.0 ;
x = [0 0 0.0 0.0] ;
u = [0.0 0.0] ;
q = x(1:2);
qd = x(3:4);
q_des(1) = pi/2;
q_des(2) = 0;
qd_des(1) = 0;
qd_des(2) = 0;
qdd_des(1) = 0;
qdd_des(2) = 0;

	  	
%  INITIALIZE COUNTERS FOR PLOTTING, INTEGRATION, AND CONTROL

i = 1 ;								% plot counter
cycle = 0 ;							% integration cycles
rkcounter = 1 ;						% integration counter

%  ENTER CONTROLLER TO BE USED
CONTROL = input('Enter a controller (0=none, 1=DJC, 2=IJC): ');
switch CONTROL
    case 0
        disp('No Control')
    case 1
        disp('DJC Control')
    case 2
        disp('IJC Control')
    otherwise
        warning('Unexpected control type')
end

%  RUN INTEGRATION

while t < Ttotal

    fprintf('Time: %f \n',t);
	
	% Update the Trajectory
	
%	[q_des,qd_des,qdd_des] = trajectory(t) ;

	% Sample the State
	
	xsampled = x ;
	
    switch CONTROL
        case 0
            u = [0 0];
        case 1
            u = control_djc( t, xsampled, q_des, qd_des, qdd_des, L_mod, m_mod, mu_mod, g, Kp, Kv);
        case 2
            u = control_ijc( t, xsampled, q_des, qd_des, qdd_des, L_mod, m_mod, mu_mod, g, Kp, Kv);
        otherwise
            warning('Unexpected control type')
    end

	% Integrate over Ncon*Tint seconds
	
	while rkcounter <= Ncon
		
	% Update the plotted variables (if it's time)
	
		if ( round(cycle/Nplot)*Nplot == cycle )
			time(i) = t ;
			state(i,:) = x ;
			control(i,:) = u ;
			error(i,:) = q_des - xsampled(1:2) ;
            q = x(1:2) ;
            position(i,:) = fwdkin(q,L) ;
			i = i + 1 ;
		end
	
	% Call Runge-Kutta Integrator
		
		[x,u] = RK4_discrete(Tint,t,q_des,qd_des,qdd_des,u,x,L,m,mu,g) ;
		t = t + Tint ;
		rkcounter = rkcounter + 1 ;
		
	% Update time counter
	
		cycle = cycle + 1 ;
		
	end
	
	rkcounter = 1 ;
	
end


% PLOT DATA

figure(1)
clf
plot(time,state(:,1:2)) ;
hold on
xlabel('time (sec)') ;
ylabel('q (rad)') ;
title('Joint Position') ;

figure(2)
clf
plot(time,error) ;
hold on
xlabel('time (sec)') ;
ylabel('q_error (rad)') ;
title('Joint Position Error') ;

figure(3)
clf
plot(time,control) ;
hold on
xlabel('time (sec)') ;
ylabel('T (N-m)') ;
title('Joint Torque') ;
                                    
figure(4)
clf
plot(position(:,1),position(:,2)) ;
hold on
xlabel('x (m)') ;
ylabel('y (m)') ;
title('Tool Position') ;
                                    
figure(5)
tic
for k=1:500
    tic
    x0=[0,position(k,3),position(k,1)];
    x1=[0,position(k,4),position(k,2)];
    plot(x0,x1,'o-')
    axis([-1,1,-1,1])
    drawnow
end

