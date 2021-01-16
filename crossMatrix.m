function matrix = crossMatrix(vec)
    %
    % crossMatrix returns the cross product matrix of the given vector
    % vec is a 3-vector
    
    matrix = [0, -vec(3,1), vec(2,1); vec(3,1), 0, -vec(1,1); -vec(2,1),...
		vec(1,1), 0];
end