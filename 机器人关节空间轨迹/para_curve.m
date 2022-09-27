function [y,dy,ddy,tp,tl] = para_curve(t,y0,a,dt)
%tp:抛物线拟合区的时间间隔
%tl:直线段的时间间隔
%y0为路径点,dt:相邻两路径之间的时间间隔
%a为拟合区加速度绝对值，y为t对应的输出值
n=length(dt);
if n==1%表示路径无中点
    ddy(1)=sign(y0(2)-y0(1))*a(1);%tp(1)段抛物线拟合区加速度
    ddy(2)=sign(y0(1)-y0(2))*a(2);%tp(2)段抛物线拟合区加速度
    tp(1)=dt/2-sqrt(ddy(1)^2*dt^2-4*ddy(1)*(y0(2)-y0(1)))/(2*ddy(1));
    tp(2)=tp(1);
    t_l=dt-tp(1)-tp(2);%直线段时间
    dy=ddy(1)*tp(1);%直线段速度
    if t<=tp(1)%抛物线拟合区tp(1)段
        y=y0(1)+ddy(1)/2*t^2;%tp(1)段抛物线段关节值
    elseif t<=dt-tp(2)%直线tl段
        y=dy*(t-tp(1))+y0(1)+ddy(1)/2*tp(1)^2;%直线段关节值
    else%抛物线拟合区tp(2)段
        y=y0(2)+ddy(2)/2*(t-dt)^2;%tp(2)段抛物线段关节值
    end
else %n>1,表示有中间点，求解抛物线拟合区加速度ddy、抛物线拟合区持续时间tp，直线段速度dy
    for i=1:n
        if i==1 %n=1，表示第一个路径段
            ddy(i)=sign(y0(i+1)-y0(i))*a(i);%抛物线拟合区加速度ddy
            tp(i)=dt(i)-sqrt(dt(i)^2-2*(y0(i+1)-y0(i))/ddy(i));%抛物线拟合区持续时间tp
            dy(i)=(y0(i+1)-y0(i))/(dt(i)-0.5*tp(i));%直线段速度dy
        elseif i<n%1<i<n,表示中间路径段
            ddy(i)=sign(y0(i+1)-y0(i))*a(i);%抛物线拟合区加速度
            dy(i)=(y0(i+1)-y0(i))/dt(i);%直线段速度dy
            tp(i)=(dy(i)-dy(i-1))/ddy(i);%抛物线拟合区持续时间tp
        else %i=n,最后一个路径段
            ddy(i)=sign(y0(i+1)-y0(i))*a(i);%抛物线拟合区加速度ddy
            ddy(i+1)=sign(y0(i)-y0(i+1))*a(i+1);
            tp(i+1)=dt(i)-sqrt(dt(i)^2+2*(y0(i+1)-y0(i))/ddy(i+1));%抛物线拟合区持续时间tp
            dy(i)=(y0(i+1)-y0(i))/(dt(i)-0.5*tp(i+1));%直线段速度dy
            tp(i)=(dy(i)-dy(i-1))/ddy(i);%抛物线拟合区持续时间tp
        end
    end
    %tl:直线段的时间间隔
    %求解各个直线段的持续时间tl
    for i=1:n
        if i==1%n=1，表示第一个路径段
            tl(i)=dt(i)-tp(i)-0.5*tp(i+1);
        elseif i<n%1<i<n,表示中间路径段
            tl(i)=dt(i)-0.5*tp(i)-0.5*tp(i+1);
        else%i=n,最后一个路径段
            tl(i)=dt(i)-tp(i+1)-0.5*tp(i);
        end
    end
    for i=1:n
        %计算输入时间对应的y时，tl(i)+tp(i+1)算做一段
        %第一段：tp(1)+tl(1)+tp(2),
        %最后一段：tl(n)+tp(n+1),
        A=sum(dt(1,i))+0.5*tp(i+1);%前i段时间总长
        if t<A%判断输入时间t是否小于A，目的是为了得出t所在的路径段
            break;
        end
    end
    m=i-1;%路径段i-1赋值给m
    if m==0%第一段：tp(1)+tl(1)+tp(2),
        if t<tp(1)
            y1=y0(1);%起始点值
            y2=dy(1)*(tp(1)-0.5*tp(1))+y0(1);%终止点值
            t_l=0;%起点时刻
            t2=tp(1);%终止时刻
            a=(t2+t_l-2*(y2-y1)/(ddy(1)*(t2-t_l)))/2;%抛物线顶点时刻
            b=y1-ddy(1)/2*(t_l-a)^2;%抛物线顶点值
            y=ddy(1)/2*(t-a)^2+b;%抛物线方程
        elseif t<=tp(1)+tl(1)
            y=dy(1)*(t-0.5*tp(1))+y0(1);%直线段tl(1)段直线方程
        else
            y1=dy(1)*(tp(1)+t(1)-0.5*tp(1))+y0(1);%起始点值
            y2=dy(2)*(dt(1)+0.5*tp(2)-dt(1))+y0(2);%终止点值
            t_l=tp(1)+tl(1);%起点时刻
            t2=dt(1)+0.5*tp(2);%终止时刻
            a=(t2+t_l-2*(y2-y1)/(ddy(2))*(t2-t_l));%抛物线顶点时刻
            b=y1-ddy(2)/2*(t_l-a)^2;%%抛物线顶点值
            y=ddy(2)/2*(t-a)^2+b;%抛物线方程
        end
    else %m>0
        B=sum(dt(1:m))+0.5*tp(m+1);%前m段时间总长，即前m-1段时间总长
        if m<n-1%表示tl(i)+tp(i+1)段
            if t<=B+tl(m+1)%直线段tl(i),(i>1)直线方程
                y=dy(m+1)*(t-(B-0.5*tp(m+1)))+y0(m+1);
            else%表示tp(i+1)段
                y1=dy(m+1)*(B+tl(m+1)-(B-0.5*tp(m+1)))+y0(m+1);%起始点值
                y2=dy(m+2)*0.5*tp(m+2)+y0(m+2);%终止点值
                t_l=B+tl(m+1);%起点时刻
                t2=B+tl(m+1)+tp(m+2);%终止时刻
                a=(t2+t_l-2*(y2-y1)/(ddy(m+2)*(t2-t_l)))/2;%抛物线顶点时刻
                b=y1-ddy(m+2)/2*(t_l-a)^2;%抛物线顶点值
                y=ddy(m+2)/2*(t-a)^2+b;%抛物线方程
            end
        else%m=n-1，%表示tl(n)+tp(n+1)段
            if t<=B+tl(m+1)%表示tl(n)段直线方程
                y=dy(m+1)*(t-(B-0.5*tp(m+1)))+y0(m+1);
            else
                y1=dy(m+1)*(B+tl(m+1)-(B-0.5*tp(m+1)))+y0(m+1);%起始点值
                y2=y0(m+2);%终止点值
                t_l=B+tl(m+1);%起点时刻
                t2=B+tl(m+1)+tp(m+2);%终止时刻
                a=(t2+t_l-2*(y2-y1)/(ddy(m+2)*(t2-t_l)))/2;%抛物线顶点时刻
                b=y1-ddy(m+2)/2*(t_l-a)^2;%抛物线顶点值
                y=ddy(m+2)/2*(t-a)^2+b;%抛物线方程
            end
        end
    end
end