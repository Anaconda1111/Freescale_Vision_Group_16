clc,clear;
delete(instrfindall);
S=Bluetooth('JDY-31-SPP',1);
fopen(S);



a=fread(S,[1,9],'uint8');
if a(1,1)~=1
    [n,m]=find(a==1);
    m=m-1;
    while m>0
        fread(S,[1,1],'uint8');
        m=m-1;
    end
end

a=fread(S,[1,9],'uint8');
Data=zeros(1,5);
Data(1,1)=a(1,1);
for i=1:4
    Data(1,i+1)=uint16(bitor(bitshift(a(1,2*i),8),a(1,2*i+1)));
    if bitget(Data(1,i+1),16)==1
        Data(1,i+1)=bitxor(Data(1,i+1),65535);
        Data(1,i+1)=(Data(1,i+1)*-1)-1;
    end
end

Data1=zeros(1,2);
Data2=zeros(1,2);
X=[0 1];
t=0;
for j=1:2
        a=fread(S,[1,9],'uint8');
        Data=zeros(1,5);
        Data(1,1)=a(1,1);
        for i=1:4
            Data(1,i+1)=uint16(bitor(bitshift(a(1,2*i),8),a(1,2*i+1)));
            if bitget(Data(1,i+1),16)==1
                Data(1,i+1)=bitxor(Data(1,i+1),65535);
                Data(1,i+1)=(Data(1,i+1)*-1)-1;
            end
        end   
        Data1(1,j)=Data(1,2);
        Data2(1,j)=Data(1,3);

end


for i=1:10000
    a=fread(S,[1,9],'uint8');
    Data=zeros(1,5);
    Data(1,1)=a(1,1);
    for j=1:4
        Data(1,j+1)=uint16(bitor(bitshift(a(1,2*j),8),a(1,2*j+1)));
        if bitget(Data(1,j+1),16)==1
            Data(1,j+1)=bitxor(Data(1,j+1),65535);
            Data(1,j+1)=(Data(1,j+1)*-1)-1;
        end
    end
    Data1(1,1)=Data1(1,2);
    Data2(1,1)=Data2(1,2);
    Data1(1,2)=Data(1,2);
     Data2(1,2)=Data(1,3);
    plot(X,Data1,'-r');%红线当前值 蓝线期望值
    axis([i-20 i+20 -200 200]);
    hold on;
    X=X+1;
    t=t+1;
    pause(0.05)
end

