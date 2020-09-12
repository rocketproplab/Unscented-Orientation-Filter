function attitudeQuat = quatUpdate(meanPlus,f,a,qK1,n)
    % Eq. (45a)
	% Find the scalar component of the quaternion update
	dqplusK1(4,1) = (-a*(norm(meanPlus(1:3,1)))^2 + f*sqrt(f^2+(1-a^2)...
        *(norm(meanPlus(1:3,1)))^2))/(f^2+(norm(meanPlus(1:3,1)))^2);
        
	% Eq. (45b)
	% Find the vector component of the quaternion update
	dqplusK1(1:3,1) = ((a+dqplusK1(4,1))/f)*meanPlus(1:3,1);
        
        
	% Eq. (44)
	% Hooray we made it
	% Update the attitude quaternion to get the postupdate propagated
	%   attitude quaternion for this cycle (what we were trying to get
	%   the whole time)
	% This is one of the important results of each loop, and will be
	% used in the next loop.
	attitudeQuat = kalmanArrayMult(dqplusK1,qK1(:,2*n+1));
end