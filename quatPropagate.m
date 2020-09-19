function NewQuats = quatPropagate(OldQuats,EstAngV,gyroDt,n)
    %
    % function quatPropogate
    % OldQuats is the array of pre-propogation sigma
    %   quaternions
    % EstAngV is the array of estimated sigma angular
    %   velocities (measured minus Chi bias sigma vectors)
    % dt is the sampling interval in the gyro
    % NewQuats is the array of propogated sigma quaternions
    
    NewQuats = zeros(4,2*n+1);
    for i=1:(2*n+1)
        
        % Eq. (29)
        psiK = ((sin(0.5*norm(EstAngV(:,i))*gyroDt))*EstAngV(:,i))/...
                norm(EstAngV(:,i));
        Omega = [cos(0.5*norm(EstAngV(:,i))*gyroDt)*eye(3)-crossMatrix(...
            psiK), psiK; -transpose(psiK), ...
            cos(0.5*norm(EstAngV(:,i))*gyroDt)];
        
        NewQuats(:,i) = Omega*OldQuats(:,i);
    end
end