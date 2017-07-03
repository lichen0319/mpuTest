%%  
function callback(s,BytesAvailable)  
    
obj=surf(peaks);
axis([0,100,0,100,-50,50])

title('MPU9250')
xlabel('X axis')
ylabel('Y axis')
zlabel('Z axis')

    out = fscanf(s);%读取串口数据，转换为num数据存储在data中
    data = str2num(out);

    roll=data(1,1);
    pitch=data(1,2);
    yaw=data(1,3);

    rotate(obj,[1 0 0],roll)
    rotate(obj,[0 1 0],pitch)
    rotate(obj,[0 0 1],yaw)
    
end