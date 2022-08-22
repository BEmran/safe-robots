%% Read imu data
read_imu();

%% Using the |magcal| Function
% calibrated = (raw - b) * A
% To find the |A| and |b| parameters
[A1, b1, mfs1]  = magcal(mag);
E1 = ResidualError(mag, A1, b1, mfs1);

%% Full Hard and Soft Iron Compensation
% To force the |magcal| function to solve for an arbitrary ellipsoid and
% produce a dense, symmetric |A| matrix, call the function as:
[A2, b2, mfs2] = magcal(mag,'diag');
E2 = ResidualError(mag, A2, b2, mfs2);

%% Comparing the results
Missalignement = A1;
Bias = b1;
if (E2 < E1)
    Missalignement = A2;
    Bias = b2;
end
disp("Magnitometer calibration\n calibrated = M * (raw - b):");
disp("Missalignement");
disp(Missalignement);
disp("Bias");
disp(Bias);

de = HelperDrawEllipsoid;
de.compareBest(A1, b1, mfs1, A2, b2, mfs2, mag);

%%
function E = ResidualError(mag, A, b, expmfs)
% The residual error is the sum of the distances between the calibrated data and a sphere of radius |expMFS|.
% E = \frac{1}{2 \beta^2}\sqrt{ \frac{\sum  ||(x-b)A||^2 - \beta^2}{N} }$$
%
N = length(mag);
magCorrected = (mag - b) * A;
r = sum(magCorrected.^2,2) - expmfs.^2;
E = sqrt(r.'*r./N)./(2*expmfs.^2);
fprintf('Residual error in corrected data : %.2f\n\n',E);
end
