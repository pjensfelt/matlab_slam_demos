%
% Author Patric Jensfelt
%
function create_ui

clf
p = get(gcf, 'Position');
p(3) = 800;
p(4) = 600;
set(gcf, 'Position', p);

if nargin < 1
    filterUI = 0;
end

global tspeed
global run
global reset
global setUniform
global addDisturbance
global dispGaussApprox
global zRhoStd
global zPhiStd
global lMask
global forceUpdate
global injectNoise
global superGPS

% Create a figure and axes
ax = axes('Units','pixels');

% Create push button
btn = uicontrol('Style', 'pushbutton', 'String', 'Quit',...
    'Position', [5 5 50 20],...
    'Callback', @clb_quit);

% Create push button
btn = uicontrol('Style', 'pushbutton', 'String', 'Reset',...
    'Position', [60 5 50 20],...
    'Callback', @clb_reset);

% Create push button
btn = uicontrol('Style', 'pushbutton', 'String', 'Uniform',...
    'Position', [5 45 50 20],...
    'Callback', @clb_uniform);

% Create push button
btn = uicontrol('Style', 'pushbutton', 'String', 'Disturb',...
    'Position', [5 80 50 20],...
    'Callback', @clb_disturbance);

% Create push button
btn = uicontrol('Style', 'pushbutton', 'String', 'Update',...
    'Position', [5 115 50 20],...
    'Callback', @clb_forceupdate);

btn = uicontrol('Style', 'pushbutton', 'String', 'Inject',...
    'Position', [5 150 50 20],...
    'Callback', @clb_injectNoise);

btn = uicontrol('Style', 'pushbutton', 'String', 'superGPS',...
    'Position', [5 185 50 20],...
    'Callback', @clb_superGPS);

tbtn = uicontrol('Style', 'togglebutton', 'String', 'DispGauss',...
    'Position', [115 5 70 20],...
    'Callback', @clb_dispgauss);
tbtn.Value = dispGaussApprox;

% Create slider
sld = uicontrol('Style', 'slider',...
    'Min',-1,'Max',15,'Value',-1,'SliderStep',[0.1 1],...
    'Position', [360 0 80 20],...
    'Callback', @clb_zrhostd);

% Add a text uicontrol to label the slider.
zrhostd_txt = uicontrol('Style','text',...
    'Position',[360 20 80 20],...
    'String','No range');
sld.Value = 15;
clb_zrhostd(sld,0);

% Create slider
sld = uicontrol('Style', 'slider',...
    'Min',-1,'Max',15,'Value',-1,'SliderStep',[0.1 1],...
    'Position', [445 0 80 20],...
    'Callback', @clb_zphistd);

% Add a text uicontrol to label the slider.
zphistd_txt = uicontrol('Style','text',...
    'Position',[445 20 80 20],...
    'String','No bearning');
sld.Value = 15;
clb_zphistd(sld,0);

% Create push button
btn = uicontrol('Style', 'pushbutton', 'String', 'Stop',...
    'Position', [540 5 50 20],...
    'Callback', @clb_stop);

% Create slider
tsld = uicontrol('Style', 'slider',...
    'Min',-1,'Max',1,'Value',0,...
    'Position', [595 0 80 20],...
    'Callback', @clb_tspeed);

% Add a text uicontrol to label the slider.
vtxt = uicontrol('Style','text',...
    'Position',[595 20 80 20],...
    'String','v=0m/s');

tbtn = uicontrol('Style', 'togglebutton', 'String', 'L1',...
    'Position', [755 75 40 20],...
    'Callback', @clb_L1);
tbtn.Value = lMask(1);

tbtn = uicontrol('Style', 'togglebutton', 'String', 'L2',...
    'Position', [755 110 40 20],...
    'Callback', @clb_L2);
tbtn.Value = lMask(2);

tbtn = uicontrol('Style', 'togglebutton', 'String', 'L3',...
    'Position', [755 145 40 20],...
    'Callback', @clb_L3);
tbtn.Value = lMask(3);

tbtn = uicontrol('Style', 'togglebutton', 'String', 'L4',...
    'Position', [755 180 40 20],...
    'Callback', @clb_L4);
tbtn.Value = lMask(4);

    function clb_tspeed(source,callbackdata)
        tspeed = 2.0*source.Value;
        vtxt.String = sprintf('v=%.3fm/s',tspeed);
    end

    function clb_zrhostd(source,callbackdata)
        val = source.Value;
        if val <= 0
            zRhoStd = -1;
        elseif val <= 5
            zRhoStd = 0.01 * val;
        else
            zRhoStd = 0.1 * (val-5);
        end
        if val < 0
            zrhostd_txt.String = 'No range';
        else
            zrhostd_txt.String = sprintf('zRhoStd=%.3fm',zRhoStd);
        end
    end

    function clb_zphistd(source,callbackdata)
        val = source.Value;
        if val <= 0
            zPhiStd = -1;
        elseif val <= 5
            zPhiStd = 0.1*pi/180 * val;
        else
            zPhiStd = 1*pi/180 * (val-5);
        end
        if val < 0
            zphistd_txt.String = 'No bearning';
        else
            zphistd_txt.String = sprintf('zPhiStd=%.3fdeg',zPhiStd*180/pi);
        end
    end

    function clb_stop(source,callbackdata)
        tspeed = 0;
        tsld.set('Value',0);
        vtxt.String = 'v=0m/s';
        disp('Stop pressed')
    end

    function clb_reset(source,callbackdata)
        reset = 1;
        disp('Reset pressed')
    end

    function clb_uniform(source,callbackdata)
        setUniform = 1;
        disp('setUniform pressed')
    end

    function clb_disturbance(source,callbackdata)
        addDisturbance = 1;
        disp('addDisturbance pressed')
    end

    function clb_quit(source,callbackdata)
        run = 0;
        disp('Reset pressed')
    end

    function clb_dispgauss(source,callbackdata)
        dispGaussApprox = source.Value;
        disp('Pressed Disp Gauss')
    end

    function clb_L1(source,callbackdata)
        lMask(1) = source.Value;
        disp(sprintf('Pressed L1, used = %d', source.Value))
    end

    function clb_L2(source,callbackdata)
        lMask(2) = source.Value;
        disp(sprintf('Pressed L2, used = %d', source.Value))
    end

    function clb_L3(source,callbackdata)
        lMask(3) = source.Value;
        disp(sprintf('Pressed L3, used = %d', source.Value))
    end

    function clb_L4(source,callbackdata)
        lMask(4) = source.Value;
        disp(sprintf('Pressed L4, used = %d', source.Value))
    end

    function clb_forceupdate(source,callbackdata)
        forceUpdate = 1;
        disp(sprintf('Forcing an update'))
    end

    function clb_injectNoise(source,callbackdata)
        injectNoise = 1;
        disp(sprintf('Injecting noise'))
    end

    function clb_superGPS(source,callbackdata)
        superGPS = 1;
        disp(sprintf('Acquired super GPS measurement'))
    end
end