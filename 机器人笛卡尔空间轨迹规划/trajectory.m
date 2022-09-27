clc,clear;
P0=input('Enter the P0:');
Pf=input('Enter the Pf:');
ts=input('Enter the t:');
t=0:0.01:ts;
l=[0.475 0.32];
RPY0=P0(4:6);
%%初始位置
%逆运动学求解
%求解θ1
T0=transl(P0(1:3))*trotz(RPY0(3))*troty(RPY0(2))*trotx(RPY0(1));
A_0=(l(1)^2-l(2)^2+P0(1)^2+P0(2)^2)/(2*l(1)*sqrt(P0(1)^2+P0(2)^2));
phi_0=atan2(P0(1),P0(2));
theta1_0=atan2(A_0,sqrt(1-A_0^2))-phi_0;
%求解θ2
r_0=sqrt(P0(1)^2+P0(2)^2);
theta2_0=atan2(r_0*cos(theta1_0+phi_0),(r_0*sin(theta1_0+phi_0)-l(1)));
%求解d3
d3_0=-P0(3);
%求解θ4
theta4_0=theta2_0-asin(-sin(theta1_0)*T0(1,1)+cos(theta1_0)*T0(2,1));
q0=[theta1_0 theta2_0 d3_0 theta4_0];
%%终止位置Pf
RPYf=Pf(4:6);
Tf=transl(Pf(1:3))*trotz(RPYf(3))*troty(RPYf(2))*trotx(RPYf(1));
%逆运动学求解
%求解θ1
A_f=(l(1)^2-l(2)^2+Pf(1)^2+Pf(2)^2)/(2*l(1)*sqrt(Pf(1)^2+Pf(2)^2));
phi_f=atan2(Pf(1),Pf(2));
theta1_f=atan2(A_f,sqrt(1-A_f^2))-phi_f;
%求解θ2
r_f=sqrt(Pf(1)^2+Pf(2)^2);
theta2_f=atan2(r_f*cos(theta1_f+phi_f),sqrt(r_f*sin(theta1_f+phi_f)-l(1)));
%求解d3
d3_f=-Pf(3);
%求解θ4
theta4_f=theta2_f-asin(-sin(theta1_f)*Tf(1,1)+cos(theta1_f)*Tf(2,1));
qf=[theta1_f theta2_f d3_f theta4_f];
%%插补
N=length(t);
%位置直线插补
P(:,1)=P0(1:3)';%位置的初始值
dP=(Pf(1:3)'-P0(1:3)')/(N-1);%位置的增量
for i=2:N
    P(:,i)=P0(1:3)'-(i-1)*dP;%插补各点位置值
end
%RPY角三次多项式插补
for i=1:N
    RPY(i,:)=RPY0+(3*t(i)^2/tf^2-2*t(i)^3/tf^3)*(RPYf-RPY0);
end
%求解插补点的位姿矩阵T和关节变量q
for i=1:N
    T(:,:,i)=transl(P(:,i))*trotz(RPY(i,3))*troty(RPY(i,2))*trotx(RPY(i,1));
    if i==1
        theta1(i)=q0(1);
        theta2(i)=q0(2);
        d3(i)=q0(3);
        theta4(i)=q0(4);
    else
        %逆运动学求解(解的选取)
        %求解θ1
        A=(l(1)^2-l(2)^2+P(1)^2+P(2)^2)/(2*l(1)*sqrt(P(1)^2+P(2)^2));
        phi=atan2(P(1,i),P(2,i));
        B=atan2(A,sqrt(1-A^2))-phi;
        C=atan2(A,-sqrt(1-A^2))-phi;
        D=min([abs(B-theta1(i-1)),abs(C-theta1(i-1))]);
        if D==abs(B-theta1(i-1))
            theta1(i)=B;
        else
            theta1(i)=C;
        end
        %求解θ2
        r0=sqrt(P(1,i)^2+P(2,i)^2);
        theta2(i)=atan2(r0*cos(theta1(i)+phi),sqrt(r0*sin(theta1(i)+phi)-l(1)));
        %求解d3
        d3(i)=-P(3,i);
        %求解θ4
        theta4(i)=theta2(i)-asin(-sin(theta1(i))*Tf(1,1,i)+cos(theta1(i))*Tf(2,1,i));
    end
q(i,:)=[theta1(i) theta2(i) d3(i) theta4(i)];
end
%%
%绘图
figure('Name','SCARA机器人末端直线运动轨迹');
plot3(P0(1),P0(2),P0(3),'ro','MarkerFaceColor','r');
hold on
plot3(Pf(1),Pf(2),Pf(3),'rv','MarkerFaceColor','r');
hold on
x=P(1,:);
y=P(2,:);
z=P(3,:);
plot3(x,y,z,'b-');
legend('初始位置','终止位置','直线轨迹');
grid;
xlabel('X');ylabel('Y');zlabel('Z');
figure('Name','SCARA机器人关节位移曲线');
subplot(1,4,1);
plot(t,q(:,1));
grid on
xlim([0,t(end)]);
xlabel('时间（s）');ylabel('关节1转角（rad）');
subplot(1,4,2);
plot(t,q(:,2));
grid on
xlim([0,t(end)]);
xlabel('时间（s）');ylabel('关节2转角（rad）');
subplot(1,4,3);
plot(t,q(:,3));
grid on
xlim([0,t(end)]);
xlabel('时间（s）');ylabel('关节3位移（m）');
subplot(1,4,4);
plot(t,q(:,4));
grid on
xlim([0,t(end)]);
xlabel('时间（s）');ylabel('关节4转角（rad）');
