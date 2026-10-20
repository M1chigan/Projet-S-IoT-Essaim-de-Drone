function mainMenu()
% Main entry point script/function
clear; close all;

mainFig = uifigure('Name', 'Menu Principal', 'Position', [500, 400, 400, 250], 'Color', [0.15, 0.15, 0.15]);
mainLayout = uigridlayout(mainFig, [3, 1]);
mainLayout.RowHeight = {'1x', 50, 50};
mainLayout.BackgroundColor = [0.15, 0.15, 0.15];

titleLabel = uilabel(mainLayout, 'Text', 'Drone Monitoring System', ...
    'FontSize', 18, 'FontColor', 'white', 'HorizontalAlignment', 'center');
titleLabel.Layout.Row = 1;

ObserverButton = uibutton(mainLayout, 'Text', 'Data Observer', ...
    'ButtonPushedFcn', @(~,~) StartObserver(mainFig));
ObserverButton.Layout.Row = 2;
ObserverButton.BackgroundColor = [0.2, 0.6, 0.3];
ObserverButton.FontColor = 'white';
ObserverButton.FontSize = 14;


launchBtn = uibutton(mainLayout, 'Text', '3D Simulation', ...
    'ButtonPushedFcn', @(~,~) startSimulation(mainFig));
launchBtn.Layout.Row = 3;
launchBtn.BackgroundColor = [0.2, 0.6, 0.3];
launchBtn.FontColor = 'white';
launchBtn.FontSize = 14;

end

function startSimulation(parentFig)
close(parentFig);
runDroneSimulation();
end

function StartObserver(parentFig)
close(parentFig);
runObserver();
end