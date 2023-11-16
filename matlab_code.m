clear;
close all;
clc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ARO2311 Team 4 Project Code
% Cal Poly Pomona 2023
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

angularVelocity = [];
acceleration = [];
tempurature = [];
linearVelocity = [];
orientation = [];
pressure = [];
magneticField = [];
quat = [];

uno = arduinoUno('/dev/cu.usbmodem1414201',9600); % Create serial port object for the Uno

fopen(uno); % Open the serial port
    
while true
        
    data = fgetl(uno); % Read a line from the serial port

    if ~isempty(data)

        [acceleration,gyro,mag,orrientation,quat,temp,pressure] = getData(data);

    end

    disp('Acceleration:');
    disp(acceleration);
    disp('Tempurature:');
    disp(((9/5) * (temp + 32)));
    disp('Orientation:');
    disp(orrientation);
    disp('Pressure:');
    disp(pressure * 0.00986923);
    disp('Magnetic Field:');
    disp(mag);
    disp('Gyro:');
    disp(gyro);
    disp('Quaternion:');
    disp(quat);

    %sim('Artificial_Horizon_2022.slx', 0.01);

end

function serialObject = arduinoUno(port,baud)

    serialObject = serialport(port,baud,'Timeout',5);

end

function [acceleration1,gyro1,mag1,orrientation1,quat1,temp,pressure] = getData(data)

    dataArray = strsplit(data, '|');

    quat = dataArray(1);
    quat = strsplit(char(string(quat(1))), ':');
    quat = strsplit(char(string(quat(3))), ',');
    quat1.x = double(string(char(quat(1)))); 
    quat1.y = double(string(char(quat(2)))); 
    quat1.z = double(string(char(quat(3)))); 
    quat1.w = double(string(char(quat(4))));

    acceleration = dataArray(2);
    acceleration = strsplit(char(string(acceleration(1))), ':');
    acceleration = strsplit(char(string(acceleration(2))), ',');
    acceleration1.x = double(string(char(acceleration(1)))); 
    acceleration1.y = double(string(char(acceleration(2)))); 
    acceleration1.z = double(string(char(acceleration(3)))); 

    mag = dataArray(3);
    mag = strsplit(char(string(mag(1))), ':');
    mag = strsplit(char(string(mag(2))), ',');
    mag1.x = double(string(char(mag(1)))); 
    mag1.y = double(string(char(mag(2)))); 
    mag1.z = double(string(char(mag(3)))); 

    gyro = dataArray(4);
    gyro = strsplit(char(string(gyro(1))), ':');
    gyro = strsplit(char(string(gyro(2))), ',');
    gyro1.x = double(string(char(gyro(1)))); 
    gyro1.y = double(string(char(gyro(2)))); 
    gyro1.z = double(string(char(gyro(3)))); 

    orrientation = dataArray(5);
    orrientation = strsplit(char(string(orrientation(1))), ':');
    orrientation = strsplit(char(string(orrientation(2))), ',');
    orrientation1.x = double(string(char(orrientation(1)))); 
    orrientation1.y = double(string(char(orrientation(2)))); 
    orrientation1.z = double(string(char(orrientation(3)))); 

    temp = dataArray(6);
    temp = strsplit(char(string(temp(1))), ':');
    temp = double(string(char(temp(2))));

    pressure = dataArray(7);
    pressure = strsplit(char(string(pressure(1))), ':');
    pressure = double(string(char(pressure(2))));

end
