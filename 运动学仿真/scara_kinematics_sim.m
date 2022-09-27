clc,clear
l=[0.475 0.325];
L(1)=Link('d',0,'a',l(1),'alpha',0,'standard','qlim',[-130 130]*pi/180);
L(2)=Link('d',0,'a',l(2),'alpha',pi,'standard','qlim',[-145 145]*pi/180);
L(3)=Link('theta',0,'a',0,'alpha',0,'standard','qlim',[0 0.3]);
L(4)=Link('d',0,'a',0,'alpha',0,'standard','qlim',[-360 360]*pi/180);
scara=SerialLink(L,'name','SCARA Robot');
q0=[0 0 0 0];
scara.plot(q0);
scara.teach(q0);
%%
qL=scara.qlim;
N=10000;
ws_q=zeros(N,4);
a=rand(size(ws_q));

for j=1:N
    for i=1:4
        ws_q(j,i)=qL(i,1)+(qL(i,2)-qL(i,1))*a(j,i);
    end
end
%�������λ������
ws_x=l(1)*cos(ws_q(:,1))+l(2)*cos(ws_q(:,1)+ws_q(:,2));
ws_y=l(1)*sin(ws_q(:,1))+l(2)*sin(ws_q(:,1)+ws_q(:,2));
ws_z=-ws_q(:,3);
figure('Name','Scara�����˹����ռ�');
plot3(ws_x,ws_y,ws_z,'r.');
grid;
xlabel('X');ylabel('Y');zlabel('Z');
%%
% clc,clear;
qA=input('Enter the A:');
qB=input('Enter the B:');
t=0:0.05:2;
[q qd qdd]=jtraj(qA,qB,t);

T=double(scara.fkine(q));
[x y z]=transl(T);
figure('Name','SCARA_Robot���������˶�ѧ��ʾ');
plot(scara,q);
figure('Name','������ĩ���˶��켣');
plot3(x,y,z,'r-o','MarkerFaceColor','r');
grid;
xlabel('X');xlabel('Y');xlabel('Z');
figure('Name','���ؽ�λ���ٶȼ��ٶ�����');
subplot(3,4,1);
plot(t,q(:,1));
title('�ؽ�1');
grid;
ylabel('λ�ƣ�rad��');
subplot(3,4,5);
plot(t,qd(:,1));
grid;
ylabel('�ٶȣ�rad/s��');
subplot(3,4,9);
plot(t,qdd(:,1));
grid;
xlabel('ʱ�䣨s��');ylabel('���ٶȣ�rad/s^2��');
subplot(3,4,2);
plot(t,q(:,2));
title('�ؽ�2');
grid;
subplot(3,4,6);
plot(t,qd(:,2));
grid;
subplot(3,4,10);
plot(t,qdd(:,2));
grid;
xlabel('ʱ�䣨s��');
subplot(3,4,3);
plot(t,q(:,3));
title('�ؽ�3');
grid;
subplot(3,4,7);
plot(t,qd(:,3));
grid;
subplot(3,4,11);
plot(t,qdd(:,3));
grid;
xlabel('ʱ�䣨s��');
subplot(3,4,4);
plot(t,q(:,4));
title('�ؽ�4');
grid;
subplot(3,4,8)
plot(t,qd(:,4));
grid;
subplot(3,4,12);
plot(t,qdd(:,4));
grid;
xlabel('ʱ�䣨s��');

