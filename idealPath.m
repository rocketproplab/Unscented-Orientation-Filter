function [idealAngV,currentQuat] = idealPath(rotationTime,rotVec,gyroDt)
    %
    %
    % Keep rates in rad/s
    idealAngV = (2*pi/(rotationTime*norm(rotVec)))*rotVec; % rad/s
    
    persistent orientationQuat
    if isempty(orientationQuat)
        % The true orientation is different from the estimated orientation,
        %   if errors to the initial estimate are introduced for testing.
        orientationQuat = [0;0;0;1];
    end
    orientationQuat = gyroIntegrate(orientationQuat,idealAngV,gyroDt);
    
    currentQuat = orientationQuat;
end