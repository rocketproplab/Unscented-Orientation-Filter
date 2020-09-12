function [idealAngV,currentQuat] = idealPath(rotationTime,gyroDt)

    % A full rotation every 90 minutes
    %
    % Keep rates in rad/s
    % idealAngV = (2/sqrt(27))*[1,1,1] deg/s
    idealAngV = (2*pi/(rotationTime*sqrt(3)))*[1;1;1]; % rad/s
    
    persistent orientationQuat
    if isempty(orientationQuat)
        % The true orientation is different from the estimated orientation,
        %   if errors to the initial estimate are introduced for testing.
        orientationQuat = [0;0;0;1];
    end
    orientationQuat = gyroIntegrate(orientationQuat,idealAngV,gyroDt);
    
    currentQuat = orientationQuat;
end