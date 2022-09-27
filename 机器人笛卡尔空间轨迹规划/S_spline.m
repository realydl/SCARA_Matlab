function y=S_spline(t,t1,y1,dy0,dyn)
%三次样条轨迹函数
%"y1"为给定的路径点，"t1"为各路径点对应的时刻
%"dy0"为初始位置速度，"dyn"为终止位置速度
%定义向量μ、λ、h和d
n=length(t1)-1;
mu=zeros(1,n);
lambda=zeros(1,n);
h=zeros(1,n);
d=zeros(1,n+1);
%求时间间隔h
for i=1:n
    h(i)=t1(i+1)-t1(i);
end
%求μ
for i=1:n-1
    mu(i)=h(i)/(h(i)+h(i+1));
end
%求λ
for i=2:n
    lambda(i)=h(i)/(h(i-1)+h(i));
end
lambda(1)=1;
%求d
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
%计算输入时间t所对应的路径段m
for i=1:n+1
    ti=sum(t1(1:i));
    if t1(i)>t
        break;
    end
end
m=i;
%计算输出时间t对应的输出值y
y=(t1(m)-t)^3/(6*h(m-1))*M(m-1)+(t-t1(m-1))^3/(6*h(m-1))*M(m)+(y1(m-1)-(M(m-1)*h(m-1)^2)/6)*(t1(m)-t)/h(m-1)+(y1(m)-(M(m)*h(m-1)^2)/6)*(t-t1(m-1))/h(m-1);

