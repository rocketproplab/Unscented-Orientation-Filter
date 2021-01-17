% This function tests a filter that simulates a satellite and tests 
%	orientation tracking.
function testFilter()
	%
	%
	% HOW to test: run this function.
	%
	% You can change the estimated initial orientation, covariance, and
	%	gyro bias in this function. You can change the true rotation 
	%	simulation parameters, simulation run time, gyro and mag sampling
	%	interval, and estimated gyro and mag noise and bias parameters in 
	%	runFilter.m. You can change the true gyro and mag bias and noise
	%	parameters in readGyro.m and readMag.m. If you want more complex
	%	flight sensor simulations, you should edit readMag.m, readGyro.m,
	%	and idealPath.m.
	
	clear all;
	clc; % Clear command window.
	% Let's time this function to see how long it takes to run.
	startTime = tic;
	fprintf('Beginning to run %s.m.\nPlease wait about 20 seconds...\n',...
		mfilename);
	
	% Define initial attitude estimate (actual attitude in idealPath).
	attitudeQuat = [0;0;0;1];
	
	% Define attitude error and bias error components of covariance.
	att_err = (pi/360)^2;
	bias_err = (pi/(3.24e6))^2;
	covariance = diag([att_err,att_err,att_err,bias_err,bias_err,...
		bias_err]);
	
	% Define gyro bias estimate.
	gyrobias = (pi/6.48)*(1e-5)*[1;1;1];
	
	%----------------------------------------------------------------------
	% HERE IS THE MAIN PART OF THE FUNCTION!
	% Run Simulation
	[v2errQs,v1errQs] = runFilter(attitudeQuat,covariance,gyrobias);
	%----------------------------------------------------------------------
	
	% Graph results in degrees for easy reading.
	v1theta = zeros(1,size(v1errQs,2));
	for i=1:size(v1errQs,2)
		v1theta(1,i) = 2*atand(norm(v1errQs(1:3,i))/v1errQs(4,i));
		%v1theta(1,i) = abs(v1err(4,i));
	end
	
	% Graph results in degrees for easy reading.
	v2theta = zeros(1,size(v2errQs,2));
	for i=1:size(v2errQs,2)
		v2theta(1,i) = 2*atand(norm(v2errQs(1:3,i))/v2errQs(4,i));
	end
	
	% Find the max number of iterations that runFilter did.
	numIterations = max(size(v1errQs,2),size(v2errQs,2));
	% Set up the time (x) axis for plotting. t will be in minutes.
	t = linspace(1,10*numIterations,numIterations) / 60;
	
	%----------------------------------------------------------------------
	% Plot the error from naive quaternion integration.
	figure;
	subplot(2, 1, 1);
	plot(t,v1theta);
	grid on;
	fontSize = 20;
	title('Naive Integration', 'FontSize', fontSize);
	xlabel('Time [minutes]', 'FontSize', fontSize);
	ylabel('Error in Degrees', 'FontSize', fontSize);
	
	% Plot the error from the Unscented Quaternion Estimator.
	subplot(2, 1, 2);
	plot(t,v2theta);
	grid on;
	title('USQUE', 'FontSize', fontSize);
	xlabel('Time [minutes]', 'FontSize', fontSize);
	ylabel('Error in Degrees', 'FontSize', fontSize);
	%legend('justQuats','USQUE');
	
	% Maximize and title the figure.
	g = gcf;
	g.WindowState = 'maximized';
	g.NumberTitle = 'off';
	g.Name = 'Simulation by Matthew Mikhailov';
	
	% Record and print run time.
	elapsedSeconds = toc(startTime);
	message = sprintf('Done %s.m.\nIt took %.1f seconds to run.\n', ...
		mfilename, elapsedSeconds);
	fprintf('%s\n', message);
	uiwait(helpdlg(message));
	
	% Allow for saving the graph.
	promptMessage = sprintf(...
		'Do you want to save this figure as an image?');
	titleBarCaption = 'Save?';
	buttonText = questdlg(promptMessage, titleBarCaption, 'Yes', 'No', ...
		'Yes');
	if contains(buttonText, 'Yes', 'IgnoreCase', true)
		
		% Get the name of the file that the user wants to save.
		% Note, if you're saving an image you can use imsave() instead of 
		%	uiputfile().
		startingFolder = pwd; % Or "pwd" or wherever you want.
		defaultFileName = fullfile(startingFolder, 'testFilter.png');
		[baseFileName, folder] = uiputfile(defaultFileName, ...
			'Specify a file');
		if baseFileName == 0
			
			% User clicked the Cancel button.
			return;
		end
		fullFileName = fullfile(folder, baseFileName);
		exportgraphics(gcf, fullFileName);
		if ispc
			
			% If it's a PC, open the image in the default image viewer for 
			%	Windows.
			winopen(fullFileName);
		end
	end
	
	
end

