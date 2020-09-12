function NewQuats = quatPropagate(OldQuats,EstAngV,dt)
    %
    % function quatPropogate
    % OldQuats is the array of pre-propogation sigma
    %   quaternions
    % EstAngV is the array of estimated sigma angular
    %   velocities (measured minus Chi bias sigma vectors)
    % dt is the sampling interval in the gyro
    % NewQuats is the array of propogated sigma quaternions
    
    NewQuats = zeros(4,size(EstAngV,2));
    for i=1:size(EstAngV,2)
        
        % Eq. (29)
        psiK = ((sin(0.5*norm(EstAngV(:,i))*dt))*EstAngV(:,i))/...
                norm(EstAngV(:,i));
        Omega = [cos(0.5*norm(EstAngV(:,i))*dt)*eye(3)-crossMatrix(...
            psiK), psiK; -psiK', cos(0.5*norm(EstAngV(:,i))*dt)];
        
        NewQuats(:,i) = Omega*OldQuats(:,i);
    end
end