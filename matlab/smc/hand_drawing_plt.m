clc; clear; close all;

% User Settings
% - Keep this script simple and explicit.
% - Input folder: data/hand_drawing
% - Input files: one .mat file per subject/shape drawing.
% - Output: one figure at a time for visual inspection.

dataFolder = fullfile('data', 'hand_drawing');
files = dir(fullfile(dataFolder, '*.mat'));

fprintf('Found %d hand drawing files in: %s\n', length(files), dataFolder);

for k = 1:length(files)
    filePath = fullfile(files(k).folder, files(k).name);

    fprintf('\nLoading %d/%d: %s\n', k, length(files), files(k).name);

    fileData = load(filePath);
    variableNames = fieldnames(fileData);

    fprintf('Variables in file:\n');
    disp(variableNames);

    figure(1);
    clf;
    hold on;
    grid on;
    axis equal;

    plottedSomething = false;

    for i = 1:length(variableNames)
        variableName = variableNames{i};
        value = fileData.(variableName);

        if isnumeric(value) && ismatrix(value)
            [numRows, numCols] = size(value);

            if numCols >= 2
                % Assumption: first column is x position, second column is y position.
                % Units: same units saved in the .mat file.
                plot(value(:, 1), value(:, 2), 'LineWidth', 1.5);
                plottedSomething = true;
                fprintf('Plotted %s as x-y data: %d rows, %d columns\n', ...
                    variableName, numRows, numCols);
            elseif numCols == 1
                % Assumption: vector data is plotted against sample index.
                plot(value, 'LineWidth', 1.5);
                plottedSomething = true;
                fprintf('Plotted %s as vector data: %d samples\n', ...
                    variableName, numRows);
            end
        end
    end

    title(files(k).name, 'Interpreter', 'none');
    xlabel('x or sample index');
    ylabel('y or value');

    if ~plottedSomething
        fprintf('No numeric vector or x-y matrix was found to plot.\n');
    end

    hold off;

    input('Press Enter to continue to next file...', 's');
end
