% This is a quick test, you must remove the "clear all" of the main to use
% it
windowSizeMSE = [];
for testIndex = 0.1:0.1:10
    disp(num2str(testIndex));
    timeInterval = testIndex;
    main
    windowSizeMSE = [findBestWindow;
        timeInterval MSE];
end