%Zoe Berg
%MENG 471 Final project 
%Yale University, Fall 2023

%graphs regular function
%computes a fast Fourier transform and graphs
%finds Euclidean distance between functions and graphs as a scatter plot
%outputs a table with average Euclidean distance

%% setup functions and increment values 
clc
clear 
close all

maximum = 65; %program will run for 6.5 seconds
increments = 0.1; %graph increments by 0.1

Ts = increments; 
t = 0:Ts:maximum;

func1 = (20 .* sin(t/2));
func2 = (exp(-t.^2)) .* (20 .* sin(3 .* t ));
func3 = 20 .* (sin(3 .* sin(3 .* sin(sin(t/3)))));
func4 = 1 + (cos(t/10) + (cos(0.2 .* t)/2)) .* exp(abs(t/50)); 

%% begin by plotting normally 
figure; 
plot(t, func1, '--', 'Color', [0, 0.4470, 0.741], 'LineWidth', 2) 
hold on 
plot(t, func2, '-', 'color', [0.466, 0.674, 0.188], 'LineWidth', 2)
hold on
plot(t, func3, ':', 'color', [0.75, 0.75, 0], 'LineWidth', 2)
hold on
plot(t, func4, '-.', 'color', [0.25, 0.25, 0.25], 'LineWidth', 2)
hold on

%add labels and improve graph appearance 
xticks('auto');
yticks('auto');
xlabel('time in microseconds [µs]');
ylabel('duty cycle');
title('Functions plotted')
legend('function 1', 'function 2', 'function 3', 'function 4');

%% fast fourier transform and plot 

%sampling frequency is Ts which is increments 
fs = 1/Ts; %sampling period

y1 = fft(func1); %computes a discrete Fourier transform of func1
f1 = (0:length(y1)-1)*fs/length(y1);
y2 = fft(func2); %computes a discrete Fourier transform of func2
f2 = (0:length(y2)-1)*fs/length(y2);
y3 = fft(func3); %computes a discrete Fourier transform of func3
f3 = (0:length(y3)-1)*fs/length(y3);
y4 = fft(func4); %computes a discrete Fourier transform of func4
f4 = (0:length(y4)-1)*fs/length(y4);

%plot figure 
figure;
plot(f1, abs(y1), '--', 'Color', [0, 0.4470, 0.741], 'LineWidth', 2)
hold on 
plot (f2, abs(y2), '-', 'color', [0.466, 0.674, 0.188], 'LineWidth', 2)
hold on 
plot (f3, abs(y3), ':', 'color', [0.75, 0.75, 0], 'LineWidth', 2)
hold on 
plot (f4, abs(y4), '-.', 'color', [0.25, 0.25, 0.25], 'LineWidth', 2)
hold on 

%add labels and improve graph appearance 
yticks("auto");
xticks("auto");
xlabel('Frequency (f) [Hz]');
ylabel('|fft(duty)|');
title('Complex Magnitude of Fast Fourier Transform Spectrum');
legend('function 1', 'function 2', 'function 3', 'function 4');

%% Euclidean distance and scatter plots 

%define maximum points 
%the amount of points calculated based on earlier maximum and increments
maximumPoints = 650; 

%put functions into a matrix 
functionMatrix = [func1', func2', func3', func4'];

%define colors to use in graph 
colorone = [0 0.4470 0.741];
colortwo = [0.466 0.674 0.188];
colorthree = [0.75 0.75 0];
colorfour = [0.25 0.25 0.25];

%% function one 
figure;

totalTwoOne = 0;
totalThreeOne = 0;
totalFourOne = 0;

for k=1:1:maximumPoints 
    %this gets the point at a particular location increasing by one 
    %time
    pointOne = [functionMatrix(k,1)]; 
    pointTwo = [functionMatrix(k,2)]; 
    pointThree = [functionMatrix(k,3)]; 
    pointFour = [functionMatrix(k,4)]; 

    %get distances 
    distanceOneTwo = norm(pointOne - pointTwo); 
    distanceOneThree = norm(pointOne - pointThree);
    distanceOneFour = norm(pointOne - pointFour); 
    distanceOneOne = 0; 
  
    %plot the values
    scatter(k, distanceOneOne, 10, colorone, 'filled'); 
    hold on
    scatter(k, distanceOneTwo, 10, colortwo, 'filled')
    hold on 
    scatter(k, distanceOneThree, 10, colorthree, 'filled')
    hold on
    scatter(k, distanceOneFour, 10, colorfour, 'filled')
    hold on

    totalTwoOne = distanceOneTwo + totalTwoOne;
    totalThreeOne = distanceOneThree + totalThreeOne; 
    totalFourOne = distanceOneFour + totalFourOne;
end

%add labels 
title('Euclidean distance from function one to the other functions')
legend('function 1', 'function 2', 'function 3', 'function 4');
xlabel('Time in deciseconds [ds]');
ylabel('Difference in duty');

%calculate average Euclidean Distances
averageOneOne = 0;
averageTwoOne = totalTwoOne / maximumPoints; 
averageThreeOne = totalThreeOne / maximumPoints; 
averageFourOne = totalFourOne / maximumPoints; 

%% function two 
figure;

totalOneTwo = 0; 
totalThreeTwo = 0; 
totalFourTwo = 0; 

for k=1:1:maximumPoints 
    pointOne = [functionMatrix(k,1)]; 
    pointTwo = [functionMatrix(k,2)]; 
    pointThree = [functionMatrix(k,3)]; 
    pointFour = [functionMatrix(k,4)]; 

    %euclidean distances 
    distanceOneTwo = norm(pointOne - pointTwo);
    distanceTwoThree = norm(pointTwo - pointThree);
    distanceTwoFour = norm(pointTwo - pointFour);
    distanceTwoTwo = 0;

    %plot the values
    scatter(k, distanceOneTwo, 10, colorone, 'filled')
    hold on 
    scatter(k, distanceTwoTwo, 10, colortwo, 'filled')
    hold on 
    scatter(k, distanceTwoThree, 10, colorthree, 'filled')
    hold on
    scatter(k, distanceTwoFour, 10, colorfour, 'filled')
    hold on 

    totalOneTwo = distanceOneTwo + totalOneTwo;
    totalThreeTwo = distanceTwoThree + totalThreeTwo; 
    totalFourTwo = distanceTwoFour + totalFourTwo;
end

%add labels 
title('Euclidean distance from function two to the other functions')
legend('function 1', 'function 2', 'function 3', 'function 4');
xlabel('Time in deciseconds [ds]');
ylabel('Difference in duty');

%calculate average Euclidean Distances
averageOneTwo = totalOneTwo / maximumPoints; 
averageTwoTwo = 0; 
averageThreeTwo = totalThreeTwo / maximumPoints; 
averageFourTwo = totalFourTwo / maximumPoints; 

%% function three 
figure;

totalOneThree = 0; 
totalTwoThree = 0; 
totalFourThree = 0; 

for k=1:1:maximumPoints 
    pointOne = [functionMatrix(k,1)]; 
    pointTwo = [functionMatrix(k,2)]; 
    pointThree = [functionMatrix(k,3)]; 
    pointFour = [functionMatrix(k,4)]; 

    %euclidean distances 
    distanceOneThree = norm(pointOne - pointThree);
    distanceTwoThree = norm(pointTwo - pointThree);
    distanceThreeFour = norm(pointThree - pointFour);
    distanceThreeThree = 0;
    
    %plot the values
    scatter(k, distanceOneThree, 10, colorone, 'filled')
    hold on 
    scatter(k, distanceTwoThree, 10, colortwo, 'filled')
    hold on
    scatter(k, distanceThreeThree, 10, colorthree, 'filled')
    hold on 
    scatter(k, distanceThreeFour, 10, colorfour, 'filled')
    hold on

    totalOneThree = distanceOneThree + totalOneThree;
    totalTwoThree = distanceTwoThree + totalTwoThree; 
    totalFourThree = distanceThreeFour + totalFourThree;
end

%add labels 
title('Euclidean distance from function three to the other functions') 
legend('function 1', 'function 2', 'function 3', 'function 4');
xlabel('Time in deciseconds [ds]');
ylabel('Difference in duty');

%calculate average Euclidean Distances
averageOneThree = totalOneThree / maximumPoints; 
averageTwoThree = totalTwoThree / maximumPoints; 
averageThreeThree = 0; 
averageFourThree = totalFourThree / maximumPoints; 

%% fourth function 
figure;

totalOneFour = 0; 
totalTwoFour = 0; 
totalThreeFour = 0; 

for k=1:1:maximumPoints 
    pointOne = [functionMatrix(k,1)]; 
    pointTwo = [functionMatrix(k,2)]; 
    pointThree = [functionMatrix(k,3)]; 
    pointFour = [functionMatrix(k,4)]; 

    %euclidean distances 
    distanceOneFour = norm(pointOne - pointFour);
    distanceTwoFour = norm(pointTwo - pointFour);
    distanceThreeFour = norm(pointThree - pointFour);
    distanceFourFour = 0; 

    %plot the values 
    scatter(k, distanceOneFour, 10, colorone, 'filled')
    hold on 
    scatter(k, distanceTwoFour, 10, colortwo, 'filled')
    hold on
    scatter(k, distanceThreeFour, 10, colorthree, 'filled')
    hold on
    scatter(k, distanceFourFour, 10, colorfour, 'filled')

    totalOneFour = distanceOneFour + totalOneFour;
    totalTwoFour = distanceTwoFour + totalTwoFour; 
    totalThreeFour = distanceThreeFour + totalThreeFour; 
end

%add labels 
title('Euclidean distance from function four to the other functions')
legend('function 1', 'function 2', 'function 3', 'function 4');
xlabel('Time in deciseconds [ds]');
ylabel('Difference in duty');

%% calculate average Euclidean Distances
averageOneFour = totalOneFour / maximumPoints; 
averageTwoFour = totalTwoFour / maximumPoints; 
averageThreeFour = totalThreeFour / maximumPoints; 
averageFourFour = 0; 

%output to a table 
functionTitle = ["One"; "Two"; "Three"; "Four"];
averageEuclideanOne = [averageOneOne; averageOneTwo; averageOneThree; averageOneFour];
averageEuclideanTwo = [averageTwoOne; averageTwoTwo; averageTwoThree; averageTwoFour];
averageEuclideanThree = [averageThreeOne; averageThreeTwo; averageThreeThree; averageThreeFour];
averageEuclideanFour = [averageFourOne; averageFourTwo; averageFourThree; averageFourFour];
averageEuclideanDistances = table(functionTitle, averageEuclideanOne, averageEuclideanTwo, averageEuclideanThree, averageEuclideanFour)
