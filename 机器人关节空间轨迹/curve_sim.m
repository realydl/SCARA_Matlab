y0=input('Enter the y0:');
dt=input('Enter the dt:');
a=input('Enter the a:');
ts=sum(dt);
t=0:0.01:ts;
n=length(t);
for i=1:n
    [y(i),dy,ddy,tp,t1]=para_curve(t(i),y0,a,dt);
end
figure('Name','带有抛物线过渡的线性差值曲线');
for i=1:length(y0)
    t0(i)=sum(dt(1:i-1));
end
t0(1)=0.5*tp(1);
t0(end)=t0(end)-0.5*tp(end);

plot([0 t0 ts],[y0(1) y0 y0(end)],'r-o','MarkerFaceColor','r')
hold on
plot(t,y,'b')
grid on
xlabel('Time(s)');ylabel('theta(deg)');
xlim([1 ts])