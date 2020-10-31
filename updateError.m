function [meanPlus,covariance] = updateError(magMeas,yK1,PK1xy,PK1vv,...
    meanMinus,PminusK1)
    %
    % Eq. (3)
	% Find the innovation (measurement minus expected measurement)
	vK1 = magMeas - yK1;
        
	% Eq. (4)
	% Find the gain
	KK1 = PK1xy/(PK1vv);
    PminusK1
    PK1vv
    vK1
    meanMinus
        
	% Eq. (2a)
	% Find the updated, propagated state (mean error vector)
	meanPlus = meanMinus + KK1*vK1;
        
	% Eq. (2b)
	% Find the updated, propagated covariance
	% This is one of the important results of each loop, and will be
	%   used in the next loop.
	covariance = PminusK1 - KK1*PK1vv*transpose(KK1);
end