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
sequence = strata.sequence.rangeDoppler();
stages = strata.stages.rangeDoppler();

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
NoRange = 2^nextpow2(double(samples))/2;

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

if (stages.nciSetting.groups == 0)
    RD_offset = 0;
    RD_size = Fft2Sz/NoRX/NoTX;
else
    RD_offset = Fft2Sz;
    RD_size = NciSz;
end

%% RD demo

figure(1);clf;
box on;

boardRadar.startMeasurements();

for idx=1:1:15000
    idx
    
    [measurement, timeStamp] = board.getFrameBuffer();

    if (stages.nciSetting.groups == 0)
        RD_data = measurement(RD_offset+(1:RD_size));
        RD_data = typecast(RD_data, 'int32');
        RD_data = complex(RD_data(1:2:end), RD_data(2:2:end));
        RD_data = reshape(RD_data,ramps,NoRange);
        RD_plot = double(RD_data).*conj(double(RD_data)); % signal power
    else
        NCI_data = measurement(RD_offset+(1:RD_size));
        NCI_data = typecast(NCI_data, 'uint32');
        NCI_data = reshape(NCI_data,ramps,NoRange);
        RD_plot = double(NCI_data)*2^31; % raise to the same level as RD_data power
    end

    RD_plot = fftshift(RD_plot.',2);
    RD_plot = 10*log10(RD_plot);
    RD_plot(RD_plot<0) = 0;

    if idx==1
        h=surf(v_vect,r_vect,RD_plot);
        colormap 'jet';
        caxis([60 130]);
        shading interp;
        %set(gca,'ztick',[]) 
        view([0 90])
        ylim([0 max(r_vect)]);
        xlim([-vmax max(v_vect)]);
        %zlim([40 140])
        ylabel('Range (m)');
        xlabel('Velocity (m/s)');
        zlabel('Magnitude (dB)');
        %set(gca,'FontSize',18);
    end
    
    if ~ishandle(h)
        break;
    end
    set(h,'ZData',RD_plot);
    drawnow;
end

boardRadar.stopMeasurements();


%%%%%%%%% Destroy the board object %%%%%%%%%
board.delete();

%for debugging, unload the wrapper dll to be able to update it
clear wrapper_matlab
