function magMeas = readMag(trueOrient,magField)
    % 
    % readMag() simulates magnetometer measurements by rotating the WMM 
    %   vector with the exact spacecraft orientation, and adding noise.
    %
    % Returns:
    % magMeas, a simulated magnetometer measurement
    
    
    % DO ALL MAG STUFF IN Tesla
    
    % Figure out how to find mag_sd
    % mag_sd is the magnetometer standard deviation. The paper says "Note 
    %   that the actual magnetic field errors have systematic components,
    %   but these are not relevant to the present filter comparisons."
    % mag_sd is currently in Tesla per measurement (not seconds)
    mag_sd = 50e-9;
    
    noise = mag_sd*randn(3,1);
    
    % rotate from the world frame into the sensor frame
    % do the inverse of quatRotate
    worldQuat = [magField; 0];
    
    idealMagMeas = multQuat(multQuat(invQuat(trueOrient),...
        worldQuat), trueOrient);
    idealMagMeas = idealMagMeas(1:3,1);
    
    magMeas = idealMagMeas + noise;
end
    
    