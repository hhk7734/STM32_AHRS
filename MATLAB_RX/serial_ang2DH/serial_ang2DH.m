clc;
clear;

% open start_stop_button
start_stop_button
global start_stop_state close_state serial_state

% serial state
serial_num = 7;

%init string to num
max_num_data = 20;
temp = zeros(max_num_data,1);

%%%%%%%%%%%%%%%%
% add settings %
%%%%%%%%%%%%%%%%

%data init
R = zeros(4,4,4);
theta = zeros(7,1);

% graph
x = zeros(8,1);
y = zeros(8,1);
z = zeros(8,1);
graph = plot3(x,y,z,'LineWidth',6);
axis([-25 25 -25 25 -25 25])
grid on;

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
        out = fscanf(serial_state);
    
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

        if count == 12
            theta(1:7) = temp(1:7)/1000;
        end
        
        theta * 57.3
%         temp(8:12)
        
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
