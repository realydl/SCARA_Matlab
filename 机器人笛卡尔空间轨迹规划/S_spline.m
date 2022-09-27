function y=S_spline(t,t1,y1,dy0,dyn)
%���������켣����
%"y1"Ϊ������·���㣬"t1"Ϊ��·�����Ӧ��ʱ��
%"dy0"Ϊ��ʼλ���ٶȣ�"dyn"Ϊ��ֹλ���ٶ�
%���������̡��ˡ�h��d
n=length(t1)-1;
mu=zeros(1,n);
lambda=zeros(1,n);
h=zeros(1,n);
d=zeros(1,n+1);
%��ʱ����h
for i=1:n
    h(i)=t1(i+1)-t1(i);
end
%���
for i=1:n-1
    mu(i)=h(i)/(h(i)+h(i+1));
end
%���
for i=2:n
    lambda(i)=h(i)/(h(i-1)+h(i));
end
lambda(1)=1;
%��d
for i=2:n
    d(i)=6/(h(i-1)+h(i))*((y1(i+1)-y1(i))/h(i)-(y1(i)-y1(i-1))/h(i-1));
end
d(1)=6/h(1)*((y1(2)-y1(1))/h(1)-dy0);
d(n+1)=6/h(n)*(dyn-(y1(n+1)-y1(n))/h(n));
A=diag(lambda);
B=diag(mu);
C=2*eye(n+1);
D=C+[zeros(n,1) A;zeros(1,n+1)]+[zeros(1,n+1);B zeros(n,1)];
M=inv(D)*d';
%��������ʱ��t����Ӧ��·����m
for i=1:n+1
    ti=sum(t1(1:i));
    if t1(i)>t
        break;
    end
end
m=i;
%�������ʱ��t��Ӧ�����ֵy
y=(t1(m)-t)^3/(6*h(m-1))*M(m-1)+(t-t1(m-1))^3/(6*h(m-1))*M(m)+(y1(m-1)-(M(m-1)*h(m-1)^2)/6)*(t1(m)-t)/h(m-1)+(y1(m)-(M(m)*h(m-1)^2)/6)*(t-t1(m-1))/h(m-1);

