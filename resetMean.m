function meanPlus = resetMean(meanPlus)
    % Instruction immediately after Eq. (45b)
	% Reset dpplusK1 (the first three components of meanPlus) to zero 
	%   for the next propagation.
	% This is one of the important results of each loop, and will be
	% used in the next loop.
	meanPlus(1:3,1) = 0;
end