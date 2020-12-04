%%
clear all;
clc;
close all;
for k = 1:100
    clearvars -except k xstarDiffs xstarErrs rrtPathDists xstarDists elapsed_times
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
        continue
    end
    xstarDiffs(k) = xstarDiff;
    xstarErrs(k) = xstarErr;
    rrtPathDists(k)=rrtPathDist;
    xstarDists(k)=xstarDist;
    elapsed_times(k)=elapsed_time;
end

filename = 'statistics.mat';
save(filename, 'xstarDiffs','xstarErrs','rrtPathDists','xstarDists','elapsed_times')
