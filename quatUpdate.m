function attitudeQuat = quatUpdate(error,f,a,possNewQuats,n)
    %
    % quatUpdate(error,f,a,possNewQuats,n)
    % updates the orientation quaternion by removing the estimated error
    %   from the propogated previous orientation.
    % 
    % Parameters:
    % error is the approximated error vector of this orientation
    %   calculation
    % possNewQuats are the possible propagated orientation quaternions
    % f and a are fine tuning constants
    % n is a constant
    %
    % Results:
    % attitudeQuat is the best estimate of the current spacecraft 
    %   orientation.
    % 
    % Eq. (45a)
	% Find the scalar component of the quaternion update
	dqplusK1 = [0;0;0;0];
	dqplusK1(4,1) = (-a*(norm(error(1:3,1)))^2 + f*sqrt(f^2+(1-a^2)...
        *(norm(error(1:3,1)))^2))/(f^2+(norm(error(1:3,1)))^2);
        
	% Eq. (45b)
	% Find the vector component of the quaternion update
	dqplusK1(1:3,1) = ((a+dqplusK1(4,1))/f)*error(1:3,1);
        
        
	% Eq. (44)
	% Hooray we made it
	% Update the attitude quaternion to get the postupdate propagated
	%   attitude quaternion for this cycle (what we were trying to get
	%   the whole time)
	% This is one of the important results of each loop, and will be
	% used in the next loop.
	attitudeQuat = kalmanArrayMult(dqplusK1,possNewQuats(:,2*n+1));
end