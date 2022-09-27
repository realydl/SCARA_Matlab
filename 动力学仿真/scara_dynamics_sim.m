%%
qA = input('Enter the A:');
qB = input('Enter the B:');
t=0:0.05:2;
[q,qd,qdd]=jtraj(qA,qB,t);

%%
m1=15;m2=10;m3=2.5;m4=2.5;%连杆质量
L1=0.475;L2=0.325;r0=0.01;%连杆参数
g=9.8;%重力加速度
for i=length(t)%求各时间点关节驱动力矩矢量
    %求解Dij
    D11=(m1*L1^2)/3+m2*((L1^2)/3+L2^2+L1*L2*cos(q(i,2)))+...
    (m3+m4)*(L1^2+L2^2+2*L1*L2*cos(q(i,2)))+(m4*r0^2)/2;
    D22=(m2*L2^2)/3+m3*L2^2+m4*L2^2+(m4*r0^2)/2;
    D33=m3+m4;
    D44=(m4*r0^2)/2;
    D12=(m2*L2^2)/3+((m2*L1*L2*cos(q(i,2)))/2)+m3*L2^2+m3*L1*L2*cos(q(i,2))+...
    m4*L2^2+m4*L1*L2*cos(q(i,2))+(m4*r0^2)/2;
    D13=0;
    D14=-(m4*r0^2)/2;
    D21=D12;
    D23=0;
    D24=-(m4*r0^2)/2;
    D31=D13;
    D32=D23;
    D34=0;
    D41=D14;
    D42=D24;
    D43=D34;
    %求解Dijk
    D112=-(m2/2+m3+m4)*L1*L2*sin(q(i,2));
    D121=D112;
    D122=-(m2/2+m3+m4)*L1*L2*cos(q(i,2));%cos or sin???
    D211=-D122;
    %求解关节力矩矢量
    tau(:,i)=[D11 D12 D13 D14;D21 D22 D23 D24;D31 D32 D33 D34;D41 D42 D43 D44]*qdd(i,:)'+...
    [2*D112*qd(i,1)*qd(i,2)+D122*qd(i,2)^2;D211*qd(i,2)^2;-(m3+m4)*g;0];
end
%%    
figure('Name','SCARA机器人关节力矩-关节变量的关系曲线');
subplot(1,4,1);
plot(q(:,1),tau(1,:));
grid on
xlabel('theta_1(rad)');ylabel('关节1力矩（N.M）');
subplot(1,4,2);
plot(q(:,2),tau(2,:));
grid on
xlabel('theta_2(rad)');ylabel('关节2力矩（N.M）');
subplot(1,4,3);
plot(q(:,3),tau(3,:));
grid on
xlabel('d_3(m)');ylabel('关节3力（N）');
subplot(1,4,4);
plot(q(:,4),tau(4,:));
grid on
xlabel('theta_4(rad)');ylabel('关节4力矩（N.M）');