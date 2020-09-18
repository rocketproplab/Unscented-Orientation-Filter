function worldVec = quatRotate(orientationQuat, referenceVec)
    % quatRotate(orientationQuat, referenceVec)
    %
    % Finds the reference vector of the rocket in the world frame
    % orientationQuat is the current orientation of the rocket
    % referenceVec is the acceleration or mag vector output by the sensor
    % cosine term 4th
    
    bodyQuat = [referenceVec; 0];
    
    worldVec = multQuat(multQuat(orientationQuat, bodyQuat),...
        invQuat(orientationQuat));
    worldVec = worldVec(1:3,1);
end