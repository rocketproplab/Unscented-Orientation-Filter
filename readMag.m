function magMeas = readMag(currentQuat,F)
    % 
    % readMag()
    %
    % returns a simulated magnetometer value
    % this magnetometer value is modeled to be the WMM vector rotated with 
    %   the exact spacecraft orientation, with noise added to each
    %   component
    % At each time step, read the exact orientation quaternion from the 
    %   idealPath function, and rotate the WMM vector from the readWMM 
    %   function, by this quaternion. Then add the white noise to this 
    %   vector to get the magnetometer measurement.
    
    % DO ALL MAG STUFF IN Tesla
    
    % TODO: Figure out how to find this
    % mag_sd is the magnetometer standard deviation. The paper says "Note 
    %   that the actual magnetic field errors have systematic components,
    %   but these are not relevant to the present filter comparisons."
    % mag_sd is currently in Tesla per measurement (not seconds)
    mag_sd = 50e-9;
    
    noise = mag_sd*randn(3,1);
    
    idealMagMeas = quatRotate(currentQuat,F);
    
    magMeas = idealMagMeas + noise;
end
    
    