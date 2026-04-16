function maze_INK(bgFile)

if nargin < 1
    bgFile = './data/maze_img.png';  % background image
end

% -----------------------------
% Data
% -----------------------------
points = zeros(50000,3);
pID = 0;
drawing = false;
sketchID = 4;  % TODO; ID of recorded data

% -----------------------------
% Figure
% -----------------------------
fig = figure('Name','INK', ...
    'NumberTitle','off', ...
    'MenuBar','none', ...
    'ToolBar','none', ...
    'Color','w');

ax = axes('Parent',fig, ...
    'Units','normalized', ...
    'Position',[0.05 0.12 0.9 0.83]);

hold(ax,'on');
axis(ax,[0 1 0 1]);
set(ax,'XTick',[],'YTick',[],'Box','on');
set(ax,'YDir','normal');

% -----------------------------
% Background image
% -----------------------------
if exist(bgFile,'file')
    bg = imread(bgFile);
    image(ax, [0 1], [0 1], flipud(bg));
else
    warning('Image file not found.');
end

axis(ax,[0 1 0 1]);
set(ax,'XTick',[],'YTick',[],'Box','on');
set(ax,'YDir','normal');

% -----------------------------
% Drawing line
% -----------------------------
hLine = plot(ax, nan, nan, 'r', 'LineWidth', 1.5);

% -----------------------------
% Buttons
% -----------------------------
uicontrol('Style','pushbutton', ...
    'String','Clear', ...
    'Units','normalized', ...
    'Position',[0.25 0.02 0.18 0.06], ...
    'Callback',@clear_callback);

uicontrol('Style','pushbutton', ...
    'String','Save', ...
    'Units','normalized', ...
    'Position',[0.57 0.02 0.18 0.06], ...
    'Callback',@save_callback);

% -----------------------------
% Mouse callbacks
% -----------------------------
set(fig,'WindowButtonDownFcn',@mouseDown);
set(fig,'WindowButtonMotionFcn',@mouseMove);
set(fig,'WindowButtonUpFcn',@mouseUp);

tic

% =========================================================
% Nested functions
% =========================================================

    function mouseDown(~,~)
        cp = get(ax,'CurrentPoint');
        x = cp(1,1);
        y = cp(1,2);

        if x >= 0 && x <= 1 && y >= 0 && y <= 1
            drawing = true;
        end
    end

    function mouseMove(~,~)
        if drawing
            cp = get(ax,'CurrentPoint');
            x = cp(1,1);
            y = cp(1,2);

            if x < 0 || x > 1 || y < 0 || y > 1
                return
            end

            pID = pID + 1;

            if pID > size(points,1)
                points = [points; zeros(50000,3)];
            end

            points(pID,1) = x;
            points(pID,2) = y;
            points(pID,3) = toc;

            set(hLine,'XData',points(1:pID,1), ...
                      'YData',points(1:pID,2));

            drawnow limitrate
        end
    end

    function mouseUp(~,~)
        if drawing
            drawing = false;
            sketchID = sketchID + 1;

            pID = pID + 1;
            if pID > size(points,1)
                points = [points; zeros(50000,3)];
            end
            points(pID,:) = NaN;
        end
    end

    function clear_callback(~,~)
        points = zeros(50000,3);
        pID = 0;
        sketchID = 0;
        set(hLine,'XData',nan,'YData',nan);
    end

    function save_callback(~,~)
        if pID == 0
            disp('No points to save.');
            return
        end

        pts = points(1:pID,:);
        filename = ['maze_INK_data_exp_' num2str(sketchID) '.mat'];
        save(['./data/' filename],'pts');
        fprintf('Saved to %s\n', filename);
    end

end