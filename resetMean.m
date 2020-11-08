function error = resetMean(error)
    % 
    % resetMean(error)
    % resets the attitude component of the error best estimate to 0
    %
    % error is the best estimate of the error in orientation of the rocket
    %
    %
    % Instruction immediately after Eq. (45b)
	% Reset dpplusK1 (the first three components of meanPlus) to zero 
	%   for the next propagation.
	% This is one of the important results of each loop, and will be
	% used in the next loop.
	error(1:3,1) = 0;
end