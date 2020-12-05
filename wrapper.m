%%
clear all;
clc;
close all;
for k = 1:20
    clearvars -except k xstarDiffs xstarErrs rrtPathDists xstarDists elapsed_times numPoints
    close all;

    try
        MainScript3D
    catch
        fprintf('Mainscript has failed \n')
        xstarDiffs(k) = NaN;
        xstarErrs(k) = NaN;
        rrtPathDists(k)=NaN;
        xstarDists(k)=NaN;
        elapsed_times(k)=NaN;
        numPoints(k)=NaN;
        continue
    end
    xstarDiffs(k) = xstarDiff;
    xstarErrs(k) = xstarErr;
    rrtPathDists(k)=rrtPathDist;
    xstarDists(k)=xstarDist;
    elapsed_times(k)=elapsed_time;
    numPoints(k)=length(xstar);
end

filename = 'statistics.mat';
save(filename, 'xstarDiffs','xstarErrs','rrtPathDists','xstarDists','elapsed_times','numPoints')
