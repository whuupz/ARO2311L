clear;
close all;
clc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ARO2311 Team 4 Project Code
% Cal Poly Pomona 2023
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% Set up the serial port

uno = arduinoUno('/dev/cu.usbmodem1414201',9600);

try
    % Open the serial port
    fopen(uno);

    % Read and display data from the Arduino
    disp('Reading IMU data from Arduino...');
    
    while true
        % Read a line from the serial port
        data = fgetl(uno);

        if (data(11) == 'G')

            [angularVelocity.x,angularVelocity.y,angularVelocity.z] = extractDataValues(data);
            disp('Angular Velocity:');
            disp(angularVelocity);

        elseif (data(11) == 'A')

            [acceleration.x,acceleration.y,acceleration.z] = extractDataValues(data);
            disp('Acceleration:');
            disp(acceleration);

        elseif (data(11) == 'T')

          tempurature = extractDataValues(data);
          disp('Tempurature:');
          disp(((9/5) * (tempurature + 32)));

          
        elseif (data(11) == 'L')

            [linearVelocity.x,linearVelocity.y,linearVelocity.z] = extractDataValues(data);
            disp('Linear Velocity:');
            disp(linearVelocity);

        elseif (data(11) == 'O')

            [orientation.x,orientation.y,orientation.z] = extractDataValues(data);
            disp('Orientation:');
            disp(orientation);
        
        elseif (data(11) == 'P')

           pressure = extractDataValues(data);
           disp('Pressure:');
           disp(pressure * 0.00986923);
          
        elseif (data(11) == 'M')

            [magneticField.x,magneticField.y,magneticField.z] = extractDataValues(data);
            disp('Magnetic Field:');
            disp(magneticField);
          
        elseif (data(11) == 'Q')

            [quat.x,quat.y,quat.z,quat.w] = extractDataValues2(data);
            disp('Quaternion:');
            disp(quat);
            euler = quat2eul(quat);
            disp('Euler');
            disp(euler);

        end




        % Check if the data contains IMU information
        % Display the IMU data in the MATLAB command window
        % Add a delay if needed to avoid overwhelming the serial port
        pause(0.01);
        clear data;
    end

catch ME
    % Handle any errors that may occur
    disp(ME.message);
end

% Close the serial port when done
fclose(uno);
delete(uno);
clear uno;
disp('Serial port closed.');

function serialObject = arduinoUno(port,baud)

    serialObject = serialport(port,baud,'Timeout',5);

end

function [xVal,yVal,zVal] = extractDataValues(data)

    dataArray = strsplit(data, '=');

    dataX = char(dataArray(2));
    dataY = char(dataArray(3));
    dataZ = char(dataArray(4));

    dataXsplit = strsplit(dataX, ' ');
    dataYsplit = strsplit(dataY, ' ');
    dataZsplit = strsplit(dataZ, ' ');

    dataZsplit2 = strsplit(char(dataZsplit(1)), '.');
    dataZsplit3 = [char(string(dataZsplit2(1))),'.',char(string(dataZsplit2(2)))];

    dataZsplit4 = strsplit((dataZsplit3), '¿');

    dataZsplit5 = strsplit(char(string(dataZsplit4(1))), ' ');
    dataZsplit6 = strsplit(char(string(dataZsplit5(1))), '');
    dataZsplit7 = strsplit(char(string(dataZsplit6(1))), 'z');

    xVal = double(string(char(dataXsplit(1))));
    yVal = double(string(char(dataYsplit(1))));
    zVal = round(double(string(dataZsplit7(1))),2);

end

function [xVal,yVal,zVal,wVal] = extractDataValues2(data)

    dataArray = strsplit(data, '=');

    dataX = char(dataArray(2));
    dataY = char(dataArray(3));
    dataZ = char(dataArray(4));
    dataW = char(dataArray(5));

    dataXsplit = strsplit(dataX, ' ');
    dataYsplit = strsplit(dataY, ' ');
    dataZsplit = strsplit(dataZ, ' ');
    dataWsplit = strsplit(dataW, ' ');

    dataWsplit2 = strsplit(char(dataWsplit(1)), '.');
    dataWsplit3 = [char(string(dataWsplit2(1))),'.',char(string(dataWsplit2(2)))];

    dataWsplit4 = strsplit((dataWsplit3), '¿');

    dataWsplit5 = strsplit(char(string(dataWsplit4(1))), ' ');
    dataWsplit6 = strsplit(char(string(dataWsplit5(1))), '');

    xVal = double(string(char(dataXsplit(1))));
    yVal = double(string(char(dataYsplit(1))));
    zVal = double(string(char(dataZsplit(1))));
    wVal = round(double(string(dataWsplit6(1))),2);

end
