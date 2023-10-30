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
%5 different initial conditions for 5 different solutions
initConds1=[pi 0 pi+.00001 0];
initConds2=[pi 0 pi+.00002 0];
initConds3=[pi 0 pi+.00003 0];
initConds4=[pi 0 pi+.00004 0];
initConds5=[pi 0 pi+.00005 0];

%solve system of equations on timescale [0,tspan]
tspan=10;
%sols=ode45(M,[0,tspan],initConds);
[t1,y1]=ode45(M,[0 ,tspan],initConds1);
[t2,y2]=ode45(M,[0,tspan], initConds2);
[t3,y3]=ode45(M,[0,tspan], initConds3);
[t4,y4]=ode45(M,[0,tspan], initConds4);
[t5,y5]=ode45(M,[0,tspan], initConds5);

%redefine x/y positions in terms of solution set
x1_1=l1*sin(y1(:,3));
y1_1=-l1*cos(y1(:,3));
x1_2=x1_1+l2*sin(y1(:,1));
y1_2=y1_1-l2*cos(y1(:,1));

x2_1=l1*sin(y2(:,3));
y2_1=-l1*cos(y2(:,3));
x2_2=x2_1+l2*sin(y2(:,1));
y2_2=y2_1-l2*cos(y2(:,1));

x3_1=l1*sin(y3(:,3));
y3_1=-l1*cos(y3(:,3));
x3_2=x3_1+l2*sin(y3(:,1));
y3_2=y3_1-l2*cos(y3(:,1));

x4_1=l1*sin(y4(:,3));
y4_1=-l1*cos(y4(:,3));
x4_2=x4_1+l2*sin(y4(:,1));
y4_2=y4_1-l2*cos(y4(:,1));

x5_1=l1*sin(y5(:,3));
y5_1=-l1*cos(y5(:,3));
x5_2=x5_1+l2*sin(y5(:,1));
y5_2=y5_1-l2*cos(y5(:,1));

%animate syste of 5 pendulums
 figure(1)
   Ncount2=0;
   fram2=0;

     for j=1:length(y1)
         Ncount2=Ncount2+1;
         fram2=fram2+1;
         plot(0, 0,'.','markersize',20);
         hold on
         plot(x1_1(j),y1_1(j),'.','markersize',20);
         plot(x1_2(j),y1_2(j),'.','markersize',20);
         plot(x2_1(j),y2_1(j),'.','markersize',20);
         plot(x2_2(j),y2_2(j),'.','markersize',20);
         plot(x3_1(j),y3_1(j),'.','markersize',20);
         plot(x3_2(j),y3_2(j),'.','markersize',20);
         plot(x4_1(j),y4_1(j),'.','markersize',20);
         plot(x4_2(j),y4_2(j),'.','markersize',20);
         plot(x5_1(j),y5_1(j),'.','markersize',20);
         plot(x5_2(j),y5_2(j),'.','markersize',20);
         hold off
         line([0 x1_1(j)], [0 y1_1(j)],'Linewidth',2);
         axis([-(l1+l2) l1+l2 -(l1+l2) l1+l2]);
         line([x1_1(j) x1_2(j)], [y1_1(j) y1_2(j)],'linewidth',2);
         line([0 x2_1(j)], [0 y2_1(j)],'Linewidth',2);
         line([x2_1(j) x2_2(j)], [y2_1(j) y2_2(j)],'linewidth',2);
         line([0 x3_1(j)], [0 y3_1(j)],'Linewidth',2);
         line([x3_1(j) x3_2(j)], [y3_1(j) y3_2(j)],'linewidth',2);
         line([0 x4_1(j)], [0 y4_1(j)],'Linewidth',2);
         line([x4_1(j) x4_2(j)], [y4_1(j) y4_2(j)],'linewidth',2);
         line([0 x5_1(j)], [0 y5_1(j)],'Linewidth',2);
         line([x5_1(j) x5_2(j)], [y5_1(j) y5_2(j)],'linewidth',2);
         h2=gca; 
         get(h2,'fontSize') 
         set(h2,'fontSize',12)
         xlabel('X','fontSize',12);
         ylabel('Y','fontSize',12);
         title('Chaotic Motion','fontsize',14)
         fh = figure(1);
         set(fh, 'color', 'white'); 
         F2=getframe;
      end

      movie(F2,fram2,20)
