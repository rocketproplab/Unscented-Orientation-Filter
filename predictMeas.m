function yK1 = predictMeas(lambda,gammaK1,n)
    % Eq. (9)
	% Find the mean expected observation vector
	yK1 = (lambda*gammaK1(:,2*n+1)+0.5*sum(gammaK1(:,1:(2*n)),2))/...
        (n+lambda);
end