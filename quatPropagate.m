function possNewQuats = quatPropagate(possQuats,possAngV,gyroDt)
    %
    % quatPropogate(possQuats,possAngV,gyroDt)
    % finds the possible orientations of the spacecraft after the possible
    %   rotations of the spacecraft
    %
    % Parameters:
    % possQuats is the array of pre-propogation sigma
    %   quaternions
    % possAngV is the array of estimated sigma angular
    %   velocities (measured minus Chi bias sigma vectors)
    % gyroDt is the sampling interval in the gyro
    %
    % Results:
    % possNewQuats is the array of propagated sigma quaternions
    
    possNewQuats = zeros(4,size(possAngV,2));
    for i=1:size(possAngV,2)
        
        % Eq. (29)
        psiK = ((sin(0.5*norm(possAngV(:,i))*gyroDt))*possAngV(:,i))/...
                norm(possAngV(:,i));
        Omega = [cos(0.5*norm(possAngV(:,i))*gyroDt)*eye(3)-crossMatrix(...
            psiK), psiK; -psiK', cos(0.5*norm(possAngV(:,i))*gyroDt)];
        
        possNewQuats(:,i) = Omega*possQuats(:,i);
    end
end