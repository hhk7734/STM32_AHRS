clc;
clear;

% open start_stop_button
start_stop_button
global start_stop_state close_state serial_state

% serial state
serial_num = 3;

%init string to num
max_num_data = 15;
temp = zeros(max_num_data,1);

%%%%%%%%%%%%%%%%
% add settings %
%%%%%%%%%%%%%%%%

%data init
sensor_num = 2;


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
        
        % quaternion to matrix
        if(count == 5 && temp(count) == sensor_num)
            q = temp(1:(count-1))/10000;
            quat2matrix(q)
        end
    end
    
    % close serial
    fclose(serial_state);
end

% close start_stop_button
close(start_stop_button);

function matrix = quat2matrix(q)
    matrix = [1 - 2*( q(2)*q(2) + q(3)*q(3) ), 2*( q(1)*q(2) - q(3)*q(4) ), 2*( q(1)*q(3) + q(2)*q(4) );
              2*( q(1)*q(2) + q(3)*q(4) ), 1 - 2*( q(1)*q(1) + q(3)*q(3) ), 2*( q(2)*q(3) - q(1)*q(4) );
              2*( q(1)*q(3) - q(2)*q(4) ), 2*( q(2)*q(3) + q(1)*q(4) ), 1 - 2*( q(1)*q(1) + q(2)*q(2) )];
end
