%**************************************************************************
%                        IMPORTANT NOTICE
%**************************************************************************
% THIS SOFTWARE IS PROVIDED "AS IS". NO WARRANTIES, WHETHER EXPRESS,
% IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES
% OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS
% SOFTWARE. INFINEON SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL,
% INCIDENTAL, OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
%**************************************************************************

clc
clearvars
close all

addpath(genpath(cd));

%% Demo Settings
showWebcam        =  true; % Display the webcam image
showSignal        =  true; % Display the range-angle intensity data
showDetections    =  true; % Display the annotated detection boxes
showTicToc        =  true; % Display profiling Tic-Toc info for the main loop
thresholdClose    =    60; % Used for scaling of the intensity data for near targets
thresholdFar      =    50; % Used for scaling of the intensity data for far targets
thresholdPeakFind =    85; % Threshold used for the FastPeakFind
cameraFov         =    72; % Camera horizonal FOV for angle-pixel correlation
cameraVpixels     =   448; % Camera verical pixels for angle-pixel correlation
cameraHpixels     =   800; % Camera horizontal pixels for angle-pixel correlation
filtPeakFind      =   fspecial('gaussian', 20, 1); % Need to play with this - see FastPeakFind comments
colormapBlueRed   =   [0 0.7 1;0 0.655 0.933;0 0.608 0.866;0 0.561 0.800;0 0.514 0.733;0 0.468 0.666;0 0.421 0.600;0 0.374 0.533;0 0.327 0.466;0 0.281 0.400;0 0.234 0.333;0 0.187 0.266;0 0.140 0.200;0 0.093 0.133;0 0.047 0.066;0 0 0;0 0 0;0 0 0;0 0 0;0 0 0;0 0 0;0 0 0;0 0 0;0 0 0;0 0 0;0 0 0;0 0 0;0 0 0;0 0 0;0 0 0;0 0 0;0 0 0;0 0 0;0 0 0;0 0 0;0 0 0;0 0 0;0 0 0;0 0 0;0 0 0;0 0 0;0 0 0;0 0 0;0 0 0;0 0 0;0 0 0;0 0 0;0 0 0;0.062 0 0;0.125 0 0;0.188 0 0;0.25 0 0;0.312 0 0;0.375 0 0;0.437 0 0;0.5 0 0;0.562 0 0;0.625 0 0;0.687 0 0;0.75 0 0;0.812 0 0;0.875 0 0;0.937 0 0;1 0 0];
staticVelFilter   =     2; % number of doppler bins around zero to consider as static
NFFTAnt           =   256; % number of angle bins - higher number looks better but lower framerate
min_range         =   0.5;  % minimum displayed range in meters
max_range         =    10;  % maximum displayed range in meters
min_vel           =    -5;  % minimum displayed velocity in m/s
max_vel           =     5;  % maximum displayed velocity in m/s
% Note that the ramp scenario is setup to exceed each of these display limits

% Webcam device and resolution selection
if (showWebcam)
    %webcamlist
    %cam = webcam('Logitech QuickCam S5500');
    %cam = webcam('Logitech BRIO');
    cam = webcam('Logitech Webcam C930e');
    cam.Resolution = '800x448';
    %preview(cam)
end

%% Board initialization
% connect to board
board = strata.utils.connectToBoard();
boardRadar = board.getIBoardRadar();

% get config structs
mmicConfig = strata.config.CES_config();
sequence = strata.sequence.CES_sequence();
stages = strata.stages.CES_stages();

% configure all settings
boardRadar.setConfiguration(mmicConfig);
boardRadar.setSequence(sequence);
boardRadar.setProcessingStages(stages);
boardRadar.setup();

% Extra configuration
radarRxs = boardRadar.getIRadarRxs(0);
radarRxs.enableDividerOutput(true); % enable divider output for test

% Get values for visualization
radarInputInfo = boardRadar.getInputInfo();
samples = radarInputInfo.samples;
ramps = double(radarInputInfo.ramps);
NoTX = double(radarInputInfo.txChannels);
NoRX = double(radarInputInfo.rxChannels);
NoRange = 128;

%% Derived parameters from config
c0=3e8;
%sweep duration
tsw=(sequence.tRamp-sequence.tRampStartDelay);
%effective RF bandwidth
bsw=(sequence.ramps(1).fDelta/sequence.tRamp*(tsw));
%pulse repetition time
PRT=sequence.tRamp+sequence.tJump+sequence.tWait;
if sequence.modulation == 0
    PRT=PRT/NoTX;
    ramps=ramps/NoTX;
end
%maximum velocity
f_sin = sequence.ramps(1).fStart+sequence.ramps(1).fDelta/2;
vmax=c0/2/(f_sin)/PRT/2;
rmax=mmicConfig.sampleRate*tsw/bsw*c0/2/2;

r_vect=linspace(0,0.5-1/NoRange,NoRange)*rmax*2;
v_vect=linspace(-0.5,0.5-1/ramps,ramps)*2*vmax;

FftBinSz = 8; % bytes;
nciBinSz = 4; % bytes;
Fft2Sz = NoRange*ramps*NoRX*NoTX*FftBinSz;
NciSz = NoRange*ramps*nciBinSz;

%% AoA parameters
NrMIMO              =   8; % Number of MIMO channels 
[~, min_range_idx]  =   min(abs(r_vect - min_range));
[~, max_range_idx]  =   min(abs(r_vect - max_range));
range_view          =   r_vect(min_range_idx : max_range_idx);
[~, min_vel_idx]    =   min(abs(v_vect - min_vel));
[~, max_vel_idx]    =   min(abs(v_vect - max_vel));
zero_vel_ind        =   find(v_vect == 0);
vel_view_ind        =   [min_vel_idx:zero_vel_ind-staticVelFilter zero_vel_ind+staticVelFilter:max_vel_idx];
vel_view            =   v_vect(vel_view_ind);

% Window function for  NrMIMO virtual receive channels
winAnt              =   hanning(NrMIMO);
scale_winAnt        =   sum(winAnt);
winAnt3D            =   permute(repmat(winAnt,1,numel(range_view),numel(ramps)),[3 2 1]);

% Positions for angle vs range plot
vU                  =   linspace(-1, 1, NFFTAnt);
blackImage          =   zeros(length(range_view),NFFTAnt);

%% Main loop to trigger ramps and plot data
figure(1);clf;
if showWebcam
    figure(2);
end
boardRadar.startMeasurements();
idx = 1;
while true

    if showTicToc tic; end
    
    % Transfer data via Ethernet from radar memory on Aurix
    [measurement, timeStamp] = board.getFrameBuffer(20000);
    if showWebcam
        img = snapshot(cam); % campture a snapshot of the webcam around the same time as the radar data
    end
    
    if showTicToc disp([num2str(toc) '  Get Frame Buffer']);tic; end
    
    % Data to extract is Tx1 and Tx3 from the linear measurement memory
    % Reshape to get rid of unwanted half of FFT and remap the Rx channels
    RD_data = typecast(measurement, 'int32');
    RD_data = double(reshape(complex(RD_data(1:2:end), RD_data(2:2:end)), ramps*NoRange, []));
    RD_data = RD_data(1:ramps*NoRange,[1 2 3 4 5 6 7 8]); % remapping already done on Aurix
    
    % Reshape to doppler x range x rx channels
    RD_data = reshape(RD_data,ramps,NoRange,8);
    
    if showTicToc disp([num2str(toc) '  Typcast and Reshape']);tic; end
    
    % Take the angle FFT
    range_profile = fftshift(RD_data,1); %shift the doppler dimension
    range_profile_FOV = range_profile(:, min_range_idx:max_range_idx, :);
    spec = fftshift(fft(range_profile_FOV.*winAnt3D,NFFTAnt,3)/scale_winAnt,3);
    
    if showTicToc disp([num2str(toc) '  Angle FFT']);tic; end
    
    % extract the doppler signatures in two regemes: "toward" or "away"
    [dopMax_neg, dopMaxInd_neg] = max(spec(min_vel_idx:zero_vel_ind-staticVelFilter,:,:),[],1);
    [dopMax_pos, dopMaxInd_pos] = max(spec(zero_vel_ind+staticVelFilter:max_vel_idx,:,:),[],1);
    
    % linear interpolation of "near" and "far" threshold values
    JMax = repmat(linspace(thresholdClose,thresholdFar,length(range_view))',1,NFFTAnt);

    % normalize the data so that a strong target is near 0 dB and noise is around -10 dB
    JdB_pos = 10.*log10(abs(squeeze(dopMax_pos)));
    JNorm_pos = JdB_pos - JMax;
    JNorm_pos(JNorm_pos < -10) = -10; % Clamp lower end to -10
    
    JdB_neg = 10.*log10(abs(squeeze(dopMax_neg)));
    JNorm_neg = JdB_neg - JMax;
    JNorm_neg(JNorm_neg < -10) = -10; % Clamp lower end to -10
    
    % limit the FOV to match the camera before detecting targets
    %JNorm_neg(:,vU < sind(cameraFov/2)*-1 | vU > sind(cameraFov/2)) = -10;
    %JNorm_pos(:,vU < sind(cameraFov/2)*-1 | vU > sind(cameraFov/2)) = -10;

    if showTicToc disp([num2str(toc) '  RDM Collapse']);tic; end
    
    % find potential targets in cartesian space using FastPeakFind
    [cent_pos, centInd_pos] = FastPeakFind(JNorm_pos*10+100, thresholdPeakFind, filtPeakFind);
    [cent_neg, centInd_neg] = FastPeakFind(JNorm_neg*10+100, thresholdPeakFind, filtPeakFind);

    if showTicToc disp([num2str(toc) '  Find Peaks']);tic; end
    
    % Plot the range-angle plot and the camera image
    if (idx == 1)
        % Initial figure setup
        figure(1);
        ax1 = gca;
        if (showSignal)
            % Superimposing the "toward" and "away" plots onto a single image
            h1 = imagesc(vU, range_view, (JNorm_neg*-1-10)+(JNorm_pos+10)); hold on;
        else
            h1 = imagesc(vU, range_view, blackImage); hold on;
        end
        set(ax1,'Color',[0 0 0]);
        set(ax1,'YDir','normal');
        colormap(colormapBlueRed);
        xlabel('sin(\theta)');ylabel('Range (m)');
        caxis([-10 10]);xlim([-1 1]);ylim([0 10]);%axis square;
        tightfig;
        if showWebcam
            figure(2);
            ax2 = gca;
            h2 = imshow(img);
            tightfig;
        end
    else
        % Fast updates of the existing axes
        if ~ishandle(h1)
            break;
        end
        if showSignal
            set(h1,'CData',(JNorm_neg*-1-10)+(JNorm_pos+10));
        else
            set(h1,'CData',blackImage);
        end
        if showWebcam
            if ~ishandle(h2)
                break;
            end
            set(h2,'CData',img);
        end
    end
    
    % Remove all existing annotations
    %delete(findall([figure(1) figure(2)],'type','rectangle')) % Causes flickering
    if exist('r1','var') delete(r1); end
    if exist('r2','var') delete(r2); end
    if exist('t1','var') delete(t1); end
    if exist('t2','var') delete(t2); end
    
    if showDetections
        % Draw rectangles around the detections in the angle-range plot and webcam image
        % Targets moving "toward" are drawn in blue and "away" are drawn in red (red-shift!)
        for jdx = 1:2:length(cent_pos)
            r1(1, jdx) = rectangle(ax1, 'Position',[vU(cent_pos(jdx))-0.125,range_view(cent_pos(jdx+1))-0.25,0.25,0.5],'LineWidth',2,'LineStyle','-','EdgeColor',[1 0 0],'Curvature',1);
            if showWebcam
                angle = asind(vU(cent_pos(jdx)))*-1;
                hPixel = (tand(angle)*((cameraHpixels/2)/tand(cameraFov/2))) + (cameraHpixels/2);
                r1(2, jdx) = rectangle(ax2, 'Position',[hPixel-30,(cameraVpixels/2)-100,60,200],'LineWidth',2,'LineStyle','-','EdgeColor',[1 0 0],'Curvature',1);
                t1(jdx) = text(ax2,hPixel,(cameraVpixels/2)-145,sprintf('Rng: %0.1f meters\nAng: %0.1f',range_view(cent_pos(jdx+1)),angle),'Color',[1 1 1],'HorizontalAlignment','center');
            end
        end
        for jdx = 1:2:length(cent_neg)
            r2(1, jdx) = rectangle(ax1, 'Position',[vU(cent_neg(jdx))-0.125,range_view(cent_neg(jdx+1))-0.25,0.25,0.5],'LineWidth',2,'LineStyle','-','EdgeColor',[0 0.7 1],'Curvature',1);
            if showWebcam
                angle = asind(vU(cent_neg(jdx)))*-1;
                hPixel = (tand(angle)*((cameraHpixels/2)/tand(cameraFov/2))) + (cameraHpixels/2);
                r2(2, jdx) = rectangle(ax2, 'Position',[hPixel-30,(cameraVpixels/2)-100,60,200],'LineWidth',2,'LineStyle','-','EdgeColor',[0 0.7 1],'Curvature',1);
                t2(jdx) = text(ax2,hPixel,(cameraVpixels/2)-145,sprintf('Rng: %0.1f meters\nAng: %0.1f',range_view(cent_neg(jdx+1)),angle),'Color',[1 1 1],'HorizontalAlignment','center');
            end
        end
    end
    drawnow;
    if showTicToc disp([num2str(toc) '  Update Plots']); end
    idx = idx + 1;
end

%% Clean up the connections
boardRadar.stopMeasurements();
board.delete();

% Clear the wrapper dll to close the UDP connection
clear wrapper_matlab

% Close the webcam connection
if showWebcam
    clear cam
end
