clc,clear;
%delete(instrfindall);
S=Bluetooth('JDY-31-SPP',1);
fopen(S);


a=fread(S,[1,3],'uint8');
if a(1,1)~=1
    [n,m]=find(a==1);
    m=m-1;
    while m>0
        fread(S,[1,1],'uint8');
        m=m-1;
    end
end
% a=fread(S,[1,3],'uint8');
Data1=zeros(1,2);
Data2=zeros(1,2);
X=[0 1];
t=0;
for j=1:2
        a=fread(S,[1,3],'uint8');
        if a(1,1)==1
            Data1(1,j)=a(1,2);
            Data2(1,j)=a(1,3);
        end
end
for i=1:1000
    a=fread(S,[1,3],'uint8');
    if a(1,1)==1
            Data1(1,1)=Data1(1,2);
            Data2(1,1)=Data2(1,2);
            Data1(1,2)=a(1,2);
            Data2(1,2)=a(1,3);
    end
    plot(X,Data1,'-r',X,Data2,'-b');%红线当前值 蓝线期望值
    axis([0 i+1 0 120]);
    hold on;
    X=X+1;
    t=t+1;
    pause(0.01)
end

