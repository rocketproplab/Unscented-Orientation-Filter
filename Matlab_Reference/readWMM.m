function magField = readWMM()
    %
    % readWMM()
    %
    % This function simulates the expected magnetic field measurement, 
    %   based on the world magnetic model. If you are simulating an orbit,
    %   and you know the position of the spacecraft at each time, consider
    %   making this time-dependent.
    %
    % Returns:
    % magField, the magnetic field at the spacecraft's location
    %
    % DO ALL MAG STUFF IN Tesla
    % Consider making this simulation take a lat, long, alt, and date 
    %   value, and return a WMM reading. Currently, this returns the
    %   magnetic field at the ROC Lucerne Dry Lake launch site 
    %   (34.497129° N, 116.958371° W) at 50km above sea level and on 
    %   2020-09-05.
    
    % [+N|-S;+E|-W;+Up|-Down]
    magField = [22819.5e-9;4638.9e-9;-39574.9e-9];
end