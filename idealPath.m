function [idealAngV,currentQuat] = idealPath(rotationTime,rotVec,gyroDt)
    %
    % idealPath(rotationTime,rotVec,gyroDt)
    % 
    % simulates the flight of the spacecraft, simulating true angular 
    %   velocity and ideal orientation
    %
    % parameters:
    % rotationTime is the time it takes the spacecraft to complete a
    %   full rotation in seconds
    % rotVec is the vector the spacecraft is rotating around
    % gyroDt is the gyroscope measuring interval
    %
    % results:
    % idealAngV is true angular velocity of the spacecraft
    % currentQuat is the true orientation quaternion of the spacecraft
    
    
    % Keep rates in rad/s
    idealAngV = (2*pi/(rotationTime*norm(rotVec)))*rotVec;
    
    persistent orientationQuat
    if isempty(orientationQuat)
        % The true orientation is different from the estimated orientation,
        %   if errors to the initial estimate are introduced for testing.
        orientationQuat = [0;0;0;1];
    end
    % update the true orientation with the latest rotation
    orientationQuat = gyroIntegrate(orientationQuat,idealAngV,gyroDt);
    
    currentQuat = orientationQuat;
end