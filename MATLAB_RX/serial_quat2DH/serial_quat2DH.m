clc;
clear;

% open start_stop_button
start_stop_button
global start_stop_state close_state serial_state

% serial state
serial_num = 3;

%init string to num
max_num_data = 20;
temp = zeros(max_num_data,1);

%%%%%%%%%%%%%%%%
% add settings %
%%%%%%%%%%%%%%%%

%data init
R = zeros(4,4,4);
theta = zeros(7,1);
theta2 = zeros(7,1);

% graph
x = zeros(8,1);
y = zeros(8,1);
z = zeros(8,1);
graph = plot3(x,y,z,'LineWidth',6);
axis([-20 20 -20 20 -25 5])

temp2 = zeros(max_num_data,1);
temp3 = zeros(max_num_data,1);



while close_state == 0
    % wait for pushing start button
    while start_stop_state == 0 && close_state == 0
        % for start_stop_button
        pause(0.01)
    end
    
    % open serial
    if close_state == 0
        port = seriallist
        if strcmp(port(serial_num),"/dev/ttyS0")
            close_state = 1;
        end
        serial_state = serial(port(serial_num),'BaudRate',115200);
        set(serial_state,'Terminator','CR/LF');
        fopen(serial_state);
    end
    
    % read serial
    while start_stop_state == 1 && close_state == 0
        % for start_stop_button
        pause(0.00001)

        % read until terminator
        out = fscanf(serial_state)
    
        % translate string to num
        count = 0;
        start_index = 1;
        end_index = 1;
        str_leng = length(out) - 1; % CR/LF \r\n => -2 + 1
        if(str_leng > 0)
            out(str_leng) = ',';
            for i = 1:1:str_leng
                if out(i) == ','
                    count = count +1;
                    end_index = i - 1;
                    temp(count) = str2double(out(start_index:end_index));
                    start_index = i + 1;
                    if count >= max_num_data
                        break
                    end
                end
            end
        end
        
        % for start_stop_button
        pause(0.00001)
        
%         % if data is delayed because of full of buffer, flush data for real time
%         if serial_state.BytesAvailable > 300
%             flushinput(serial_state);
%         end
        
        %%%%%%%%%%%%
        % add code %
        %%%%%%%%%%%%

        % quaternion to matrix
        if(count == 5 && temp(count) == 1)
            q = temp(1:(count-1))/20000;
            R(:,:,1) = quat2matrix(q);
        end
        if(count == 5 && temp(count) == 2)
            q = temp(1:(count-1))/20000;
            R(:,:,2) = quat2matrix(q);
        end
        if(count == 5 && temp(count) == 3)
            q = temp(1:(count-1))/20000;
            R(:,:,3) = quat2matrix(q);
        end
        if(count == 5 && temp(count) == 4)
            q = temp(1:(count-1))/20000;
            R(:,:,4) = quat2matrix(q);
        end
        
        % sensor to DH parameter
        R1 = R(:,:,1) * roty(-pi/2) * rotx(-pi/2);
        R4 = R(:,:,2) * roty(-pi/2) * rotx(pi/2);
        R5 = R(:,:,3) * roty(pi/2) * rotz(pi);
        R7 = R(:,:,4);
        
        R14 = transpose(R1) * R4;
        R45 = transpose(R4) * R5;
        R57 = transpose(R5) * R7;
        
        theta(1) = atan2(R1(1,3) ,R1(1,1));
        theta(2) = atan2(R14(2,2),R14(1,2));
        theta(3) = acos(-R14(3,2));
        theta(4) = atan2(R14(3,3),R14(3,1));
        theta(5) = atan2(R45(1,3),R45(1,1));
        theta(6) = atan2(R57(1,2),-R57(2,2));
        theta(7) = atan2(R57(3,1),-R57(3,3));
        theta = real(theta);
        theta * 57.3
        
        % DH parameter graph
        d = [2 -2 0 10 0 -10 0];
        l = [0 0 0 0 0 0 4];
        kk = eye(4);
        for i=1:1:7
            kk = kk*dh(theta(i),d(i),l(i),pi/2);
            x(i+1) = (kk(1,4));
            y(i+1) = (kk(2,4));
            z(i+1) = (kk(3,4));
        end
        set(graph,'XData',x,'YData',y,'ZData',z);
    end
    
    % close serial
    fclose(serial_state);
end

% close graph
close all;

% close start_stop_button
close(start_stop_button);

function matrix = quat2matrix(q)
    matrix = [1 - 2*( q(2)*q(2) + q(3)*q(3) ), 2*( q(1)*q(2) - q(3)*q(4) ), 2*( q(1)*q(3) + q(2)*q(4) ), 0;
              2*( q(1)*q(2) + q(3)*q(4) ), 1 - 2*( q(1)*q(1) + q(3)*q(3) ), 2*( q(2)*q(3) - q(1)*q(4) ), 0;
              2*( q(1)*q(3) - q(2)*q(4) ), 2*( q(2)*q(3) + q(1)*q(4) ), 1 - 2*( q(1)*q(1) + q(2)*q(2) ), 0;
              0, 0, 0, 1];
end

function R=dh(a,d,l,b)
    R=rotz(a)*trans(l,0,d)*rotx(b);
end

function t=trans(x,y,z)
t=[1 0 0 x;
    0 1 0 y;
    0 0 1 z;
    0 0 0 1];
end

function zm=rotz(z)
zm = [cos(z) -sin(z) 0 0;
    sin(z) cos(z) 0 0;
    0 0 1 0;
    0 0 0 1];
end

function ym=roty(y)
ym = [cos(y) 0 sin(y) 0;
    0 1 0 0;
    -sin(y) 0 cos(y) 0;
    0 0 0 1];
end

function xm=rotx(x)
xm = [1 0 0 0;
    0 cos(x) -sin(x) 0;
    0 sin(x) cos(x) 0;
    0 0 0 1];
end
