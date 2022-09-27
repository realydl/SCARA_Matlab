clc,clear;
t1=input('Enter the t');
P1=input('Enter the P:');
dP0=input('Enter the dP0:');
dPn=input('ENter the dPn:');
t=0:0.01:t1(end);
n=length(t);
for i=1:n
    x(i)=S_spline(t(i),t1,P1(1,:),dP0(1),dPn(1));
    y(i)=S_spline(t(i),t1,P1(2,:),dP0(2),dPn(2));
    z(i)=S_spline(t(i),t1,P1(3,:),dP0(3),dPn(3));
end
P=[x;y;z];
figure('Name','三次样条轨迹');
plot3(P1(1,:),P1(2,:),P1(3,:),'ro','MarkerFaceColor','r');
hold on
plot3(P(1,:),P(2,:),P(3,:),'b-')
grid;
xlabel('X');ylabel('Y');zlabel('Z');
figure('Name','位置坐标随时间变化的曲线');
subplot(1,3,1);
plot(t,x);
grid;
xlabel('时间（s）');ylabel('Py');
subplot(1,3,2);
plot(t,y);
grid;
xlabel('时间（s）');ylabel('Pz');
subplot(1,3,3);
plot(t,z);
grid;



