function PK1xy = crossCorr(ChiK1,meanMinus,gammaK1,yK1,lambda,n)
    % Eq. (13)
	% Find the cross-correlation matrix
        
	% P_in is an intermediate step, used also for Eq. (13)
	P_in = ChiK1(:,1:(2*n+1)) - repmat(meanMinus,1,2*n+1);
        
	% Eq. (11) intermediate step, used also for Eq. (13)
	P_in2 = gammaK1(:,:) - repmat(yK1,1,2*n+1);
        
	PK1xy = (lambda*P_in(:,2*n+1)*(P_in2(:,2*n+1)') + ...
        0.5*sum(P_in(:,1:(2*n))*(P_in2(:,1:(2*n))'),2))/(n+lambda);
end