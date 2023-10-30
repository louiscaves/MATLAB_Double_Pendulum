clearvars

g=9.81;
%define system of equations with the following variables.
% theta1 and theta2 are functions of time
syms theta1(t) theta2(t) l1 l2 m1 m2

%express x,y positions and velocities in terms of generalized coordinates
x1 = l1*sin(theta1);
y1 = -l1*cos(theta1);
x2 = x1+l2*sin(theta2);
y2=y1-l2*cos(theta2);
vx1=diff(x1);
vy1=diff(y1);
vx2=diff(x2);
vy2=diff(y2);

%find kinetic/potential energies and lagrangian
T=1/2*m1*(vx1^2+vy1^2)+1/2*m2*(vx2^2+vy2^2);
U=(m1*g*y1)+(m2*g*(y2));
L=T-U;

%lagrangian derivatives
dL_theta1=diff(L,theta1);
dL_theta2=diff(L,theta2);
dL_theta1Dot=diff(L,diff(theta1(t),t));
dL_theta2Dot=diff(L,diff(theta2(t),t));
dt_dL_theta1Dot=diff(dL_theta1Dot);
dt_dL_theta2Dot=diff(dL_theta2Dot);

%----Equations of motion----%
EqofMot_theta1= dL_theta1==dt_dL_theta1Dot;
EqofMot_theta2= dL_theta2==dt_dL_theta2Dot;

%sub values for l1 l2 m1 m2 into equations of motion
l1=1;
l2=1;
m1=1;
m2=1;
EqRed_theta1=subs(EqofMot_theta1);
EqRed_theta2=subs(EqofMot_theta2);

%convert system of 2nd order ODEs to first order using function
%odeToVectorField
% V=column vector of 1st oder ODEs where V=dS/dt
% S=column vector [theta2, Dtheta2, theta1, Dtheta1]
[V,S]=odeToVectorField(EqRed_theta1,EqRed_theta2);

%convert to matlab function
M=matlabFunction(V,'vars',{'t','Y'});

%define initial conditions
theta1_0=pi;
Dtheta1_0=0;
theta2_0=pi;
Dtheta2_0=0;
initConds=[theta2_0 Dtheta2_0 theta1_0 Dtheta1_0];

%solve system of equations on timescale [0,tspan]
tspan=15;
sols=ode45(M,[0,tspan],initConds);
[t,y]=ode45(M,[0 ,tspan],initConds);

%Redeifne x,y coordinates in terms of solution vector Y
x1=l1*sin(sols.y(3,:));
y1=-l1*cos(sols.y(3,:));
x2=x1+l2*sin(sols.y(1,:));
y2=y1-l2*cos(sols.y(1,:));

x_1=l1*sin(y(:,3));
y_1=-l1*cos(y(:,3));
x_2=x_1+l2*sin(y(:,1));
y_2=y_1-l2*cos(y(:,1));

%test plot
figure(1)
plot(sols.x,sols.y)
legend('\theta_2','d\theta_2/dt','\theta_1','d\theta_1/dt')
title('Solutions of State Variables')
xlabel('Time (s)')
ylabel('Solutions (rad or rad/s)')

figure(2)
plot(t,y)
legend('\theta_2','d\theta_2/dt','\theta_1','d\theta_1/dt')
title('Solutions of State Variables')
xlabel('Time (s)')
ylabel('Solutions (rad or rad/s)')

   figure(3)
   plot(x1,y1,'linewidth',2)
   hold on
   plot(x2,y2,'r','linewidth',2)
   h=gca; 
   get(h,'fontSize') 
   set(h,'fontSize',14)
   xlabel('X','fontSize',14);
   ylabel('Y','fontSize',14);
   title('Chaotic Double Pendulum','fontsize',14)
   fh = figure(3);
   set(fh, 'color', 'white'); 

   figure(4)
   plot(x_1,y_1,'linewidth',2)
   hold on
   plot(x_2,y_2,'r','linewidth',2)
   h=gca; 
   get(h,'fontSize') 
   set(h,'fontSize',14)
   xlabel('X','fontSize',14);
   ylabel('Y','fontSize',14);
   title('Chaotic Double Pendulum','fontsize',14)
   fh = figure(4);
   set(fh, 'color', 'white'); 


%animate the system
 % 
 % figure(5)
 %   Ncount=0;
 %   fram=0;
 % 
 %     for i=1:length(sols.x)
 %         Ncount=Ncount+1;
 %         fram=fram+1;
 %         plot(0, 0,'.','markersize',20);
 %         hold on
 %         plot(x1(i),y1(i),'.','markersize',20);
 %         plot(x2(i),y2(i),'.','markersize',20);
 %         hold off
 %         line([0 x1(i)], [0 y1(i)],'Linewidth',2);
 %         axis([-(l1+l2) l1+l2 -(l1+l2) l1+l2]);
 %         line([x1(i) x2(i)], [y1(i) y2(i)],'linewidth',2);
 %         h=gca; 
 %         get(h,'fontSize') 
 %         set(h,'fontSize',12)
 %         xlabel('X','fontSize',12);
 %         ylabel('Y','fontSize',12);
 %         title('Chaotic Motion','fontsize',14)
 %         fh = figure(5);
 %         set(fh, 'color', 'white'); 
 %         F=getframe;
 %      end
 % 
 %      movie(F,fram,20)

 figure(6)
   Ncount2=0;
   fram2=0;

     for j=1:length(y)
         Ncount2=Ncount2+1;
         fram2=fram2+1;
         plot(0, 0,'.','markersize',20);
         hold on
         plot(x_1(j),y_1(j),'.','markersize',20);
         plot(x_2(j),y_2(j),'.','markersize',20);
         hold off
         line([0 x_1(j)], [0 y_1(j)],'Linewidth',2);
         axis([-(l1+l2) l1+l2 -(l1+l2) l1+l2]);
         line([x_1(j) x_2(j)], [y_1(j) y_2(j)],'linewidth',2);
         h2=gca; 
         get(h2,'fontSize') 
         set(h2,'fontSize',12)
         xlabel('X','fontSize',12);
         ylabel('Y','fontSize',12);
         title('Double Pendulum System','fontsize',14)
         fh = figure(6);
         set(fh, 'color', 'white'); 
         F2=getframe;
      end

      movie(F2,fram2,20)