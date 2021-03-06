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

%% connect to board
board = strata.utils.connectToBoard();
boardRadar = board.getIBoardRadar();

%% get config structs
mmicConfig = strata.config.rangeDoppler();
sequence = strata.sequence.range3Tx_DoA();
stages = strata.stages.rangeData();

%% configure all settings
boardRadar.setConfiguration(mmicConfig);
boardRadar.setSequence(sequence);
boardRadar.setProcessingStages(stages);
boardRadar.setup();

%% Extra configuration
radarRxs = boardRadar.getIRadarRxs(0);
radarRxs.enableDividerOutput(true); % enable divider output for test

%% Get values for visualization
radarInputInfo = boardRadar.getInputInfo();
samples = radarInputInfo.samples;
ramps = double(radarInputInfo.ramps);
NoTX = double(radarInputInfo.txChannels);
NoRX = double(radarInputInfo.rxChannels);
%NoRange = 2^nextpow2(double(samples))/2;
NoRange = 512;

%% scaling for plot

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
NFFTAnt             =  256;
NrMIMO              =  8;       % Number of MIMO channels 
min_range           =   0.5;  % minimum calculated range in meters
max_range           =   5;  % maximum calculated range in meters

[~, min_range_idx]  =   min(abs(r_vect - min_range));
[~, max_range_idx]  =   min(abs(r_vect - max_range));
range_view          =   r_vect(min_range_idx : max_range_idx);

% Window function for  NrMIMO virtual receive channels
winAnt              =   hanning(NrMIMO);
scale_winAnt        =   sum(winAnt);
winAnt2D            =   repmat(winAnt.',numel(range_view),1);
winAnt3D            =   permute(repmat(winAnt,1,numel(range_view),ramps),[2 3 1]);
angle_deg           =   (-NFFTAnt/2 : NFFTAnt/2-1)'./NFFTAnt.*180;

% Positions for polar plot of cost function
vU                  =   linspace(-1, 1, NFFTAnt);
[mRange , mU]       =   ndgrid(range_view, vU);
mX                  =   mRange.*mU;
mY                  =   mRange.*cos(asin(mU));

%% RD demo
figure(1);clf;
box on;

boardRadar.startMeasurements();

for idx=1:1:2000000
    idx
    
    [measurement, timeStamp] = board.getFrameBuffer(20000);

    % Data to extract is Tx1 and Tx3 from the linear measurement memory
    % Reshape to get rid of unwanted half of FFT and remap the Rx channels
    RD_data = typecast(measurement, 'int32');
    %RD_data = double(complex(RD_data(1:2:end), RD_data(2:2:end)));
    %plot(abs(RD_data))
    RD_data = double(reshape(complex(RD_data(1:2:end), RD_data(2:2:end)), NoRange*2, []));
    %RD_data = RD_data(1:NoRange,[3 4 1 2 11 12 9 10]);
    RD_data = RD_data(1:NoRange,[1 2 3 4 9 10 11 12]);
    
    % Look at range slices at zero doppler only to compare
    range_profile = RD_data;
    range_profile_FOV = range_profile(min_range_idx : max_range_idx, :);  
    %plot(unwrap(angle(range_profile)));
    spec = fftshift(fft(range_profile_FOV.*winAnt2D,NFFTAnt,2)/scale_winAnt,2);
        
    % normalize cost function
    JdB              =   20.*log10(abs(spec));
    %[JMax, idx_JMax] =   max(JdB(:));
    JMax = 130;
    JNorm            =   JdB - JMax;
    JNorm(JNorm < -20)  =   -20;

    if idx==1
        %polar plot
        figure(1);
        h = surf(mX,mY,JNorm);
        shading flat;
        view(0,90);
        xlim([-5 5]);
        ylim([0 5]);
        caxis([-20 -8]);
        axis equal; xlabel('x (m)'); ylabel('y (m)'); colormap('jet');
    end
    
    if ~ishandle(h)
        break;
    end
    set(h,'ZData',JNorm);

    drawnow;
end

boardRadar.stopMeasurements();


%%%%%%%%% Destroy the board object %%%%%%%%%
board.delete();

%for debugging, unload the wrapper dll to be able to update it
clear wrapper_matlab
