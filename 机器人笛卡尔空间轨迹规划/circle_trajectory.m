P1=input('Enter the P1:');
P2=input('Enter the P2:');
P3=input('Enter the P3:');
ts=input('Enter the t:');

t=0:0.01:ts;
A=[P1(1) P1(2) P1(3) 1;P2(1) P2(2) P2(3) 1;P3(1) P3(2) P3(3) 1];
B=[2*(P1(1)-P2(1)) 2*(P1(2)-P2(2)) 2*(P1(3)-P2(3));2*(P2(1)-P3(1)) 2*(P2(2)-P3(2)) 2*(P2(3)-P3(3));det(A(:,2:4)) -det(A(:,[1,3:4])) det(A(:,[1,2,4]))];
C=[dot(P1,P1)-dot(P2,P2); dot(P2,P2)-dot(P3,P3); det(A(:,1:3))];
O=inv(B)*C;
R=sqrt(dot(P1-O,P1-O));
n=cross(P2-P1,P3-P2);
%%求圆心角theta
n1=cross(P1-O,P3-P2);
H=dot(n,n1);
if H>=0
    theta=2*asin(sqrt(dot(P3-P1,P3-P1))/(2*R));
else
    theta=2*pi-2*asin(sqrt(dot(P3-P1,P3-P1))/(2*R));
end
N=length(t)-2;
delta=theta/(N+1);
ds=delta*R;
E=ds/(R*sqrt(dot(n,n)));
G=R/sqrt(R^2+ds^2);
%起点P1赋值给P(:,1)
x(1)=P1(1);
y(1)=P1(2);
z(1)=P1(3);
P(:,1)=[x(1) y(1) z(1)]';
%圆弧插补
for i=1:N+1
    m(:,i)=cross(n,P(:,i)-O);
    %各插补点坐标值
    x(i+1)=O(1)+G*(x(i)+E*m(1,i)-O(1));
    y(i+1)=O(2)+G*(y(i)+E*m(2,i)-O(2));
    z(i+1)=O(3)+G*(z(i)+E*m(3,i)-O(3));
    P(:,i+1)=[x(i+1) y(i+1) z(i+1)]';
end
figure('Name','圆弧轨迹');
plot3(P1(1),P1(2),P1(3),'bo','MarkerFaceColor','b')%P1
hold on
plot3(P2(1),P2(2),P2(3),'bv','MarkerFaceColor','b')%P2
hold on
plot3(P3(1),P3(2),P3(3),'bs','MarkerFaceColor','b')%P3
hold on
plot3(O(1),O(2),O(3),'mo','MarkerFaceColor','m')%圆心O点
hold on
plot3(P(1,:),P(2,:),P(3,:),'r')
legend('P1','P2','P3','圆心O','圆弧轨迹')
grid on
xlabel('x');ylabel('y');zlabel('Z');
figure('Name','位置坐标随时间变化曲线');
subplot(1,3,1)
plot(t,P(1,:));
grid on
xlim([0 t(end)]);
xlabel('时间（s）');ylabel('Px');
subplot(1,3,2)
plot(t,P(2,:));
grid on
xlim([0 t(end)]);
xlabel('时间（s）');ylabel('Py');
subplot(1,3,3)
plot(t,P(3,:));
grid on
xlim([0 t(end)]);
xlabel('时间（s）');ylabel('Pz');
