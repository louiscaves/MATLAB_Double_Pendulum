clearvars
g=9.81;

syms a1(t) a2(t) a3(t) a4(t) l1 l2 l3 l4 m1 m2 m3 m4

% xAr=[x1 x2 x3 x4];
% yAr=[y1 y2 y3 y4];
% lAr=[l1 l2 l3 l4];
% mAr=[m1 m2 m3 m4];
% aAr=[a1 a2 a3 a4];
% for i=1:length(xAr)
%     xAr(i)=xAr(i-1)+lAr(i)*sin(aAr(i))
% end
x1=l1*sin(a1);
y1=-l1*cos(a1);
x2=x1+l2*sin(a2);
y2=y1-l2*cos(a2);
x3=x2+l3*sin(a3);
y3=y2-l3*cos(a3);
x4=x3+l4*sin(a4);
y4=y3-l4*cos(a4);
vx1=diff(x1);
vy1=diff(y1);
vx2=diff(x2);
vy2=diff(y2);
vx3=diff(x3);
vy3=diff(y3);
vx4=diff(x4);
vy4=diff(y4);
T1=1/2*m1*(vx1^2+vy1^2);
T2=1/2*m2*(vx2^2+vy2^2);
T3=1/2*m3*(vx3^2+vy3^2);
T4=1/2*m4*(vx4^2+vy4^2);
T=T1+T2+T3+T4;
U1=m1*g*y1;
U2=m2*g*y2;
U3=m3*g*y3;
U4=m4*g*y4;
U=U1+U2+U3+U4;
L=T-U;
dla1=diff(L,a1);
dla2=diff(L,a2);
dla3=diff(L,a3);
dla4=diff(L,a4);
dla1d=diff(L,diff(a1(t),t));
dla2d=diff(L,diff(a2(t),t));
dla3d=diff(L,diff(a3(t),t));
dla4d=diff(L,diff(a4(t),t));
dtdla1d=diff(dla1d);
dtdla2d=diff(dla2d);
dtdla3d=diff(dla3d);
dtdla4d=diff(dla4d);
EOM1=dla1==dtdla1d;
EOM2=dla2==dtdla2d;
EOM3=dla3==dtdla3d;
EOM4=dla4==dtdla4d;

l1=1;
l2=1;
l3=1;
l4=1;
m1=1;
m2=1;
m3=1;
m4=1;
EOM1r=subs(EOM1);
EOM2r=subs(EOM2);
EOM3r=subs(EOM3);
EOM4r=subs(EOM4);

[V,S]=odeToVectorField(EOM1r,EOM2r,EOM3r,EOM4r);
M=matlabFunction(V,'vars',{'t','Y'});

initConds=[3 0 2.5 0 2 0 1.5 0];
tspan=10;
sols=ode45(M,[0 tspan],initConds);

x1=l1*sin(sols.y(3,:));
y1=-l1*cos(sols.y(3,:));
x2=x1+l2*sin(sols.y(1,:));
y2=y1-l2*cos(sols.y(1,:));
x3=x2+l3*sin(sols.y(5,:));
y3=y2-l3*cos(sols.y(5,:));
x4=x3+l4*sin(sols.y(7,:));
y4=y3-l4*cos(sols.y(7,:));

figure(1)
plot(sols.x,sols.y)
legend('1','2','3','4','5','6','7','8')

figure(2)
Ncount=0;
fram=0;
for i=1:length(sols.x)
    Ncount=Ncount+1;
    fram=fram+1;
    plot(0,0,'.','markersize',20);
    hold on
    plot(x1(i),y1(i),'.','markersize',20);
    plot(x2(i),y2(i),'.','markersize',20);
    plot(x3(i),y3(i),'.','markersize',20);
    plot(x4(i),y4(i),'.','markersize',20);
    hold off
    line([0 x1(i)], [0 y1(i)],'Linewidth',2);
    line([x1(i) x2(i)], [y1(i) y2(i)],'Linewidth',2);
    line([x2(i) x3(i)], [y2(i) y3(i)],'Linewidth',2);
    line([x3(i) x4(i)], [y3(i) y4(i)],'Linewidth',2);
    axis([-(l1+l2+l3+l4) (l1+l2+l3+l4) -(l1+l2+l3+l4) (l1+l2+l3+l4)]);
    h=gca;
    get(h,'fontSize')
    set(h,'fontSize',12)
    xlabel('X','fontSize',12)
    ylabel('Y','fontSize',12)
    title('Quadruple Pendulum System','FontSize',14)
    fh=figure(2);
    set(fh,'color','white');
    F=getframe;
end
movie(F,fram,20)



