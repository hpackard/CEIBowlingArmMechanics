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

%webcamlist
%cam = webcam('Logitech QuickCam S5500');
%cam.Resolution = '960x720';
%preview(cam)

%% connect to board
board = strata.utils.connectToBoard();
boardRadar = board.getIBoardRadar();

%% get config structs
mmicConfig = strata.config.rangeDoppler();
sequence = strata.sequence.rangeDoppler3Tx_DoA();
stages = strata.stages.rangeDoppler_DoA();

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
NoRange = 64;

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
max_range           =   10;  % maximum calculated range in meters
min_vel             =   -10;  % minimum calculated velocity in m/s
static_vel_filter   =   3; % number of doppler bins around zero to consider as static
max_vel             =   10;  % maximum calculated velocity in m/s

[~, min_range_idx]  =   min(abs(r_vect - min_range));
[~, max_range_idx]  =   min(abs(r_vect - max_range));
range_view          =   r_vect(min_range_idx : max_range_idx);

[~, min_vel_idx]  =   min(abs(v_vect - min_vel));
[~, max_vel_idx]  =   min(abs(v_vect - max_vel));
zero_vel_ind      =   find(v_vect == 0);
vel_view_ind      =   [min_vel_idx:zero_vel_ind-static_vel_filter zero_vel_ind+static_vel_filter:max_vel_idx];
vel_view          =   v_vect(vel_view_ind);

% Window function for  NrMIMO virtual receive channels
winAnt              =   hanning(NrMIMO);
scale_winAnt        =   sum(winAnt);
winAnt2D            =   repmat(winAnt.',numel(range_view),1);
winAnt3D            =   permute(repmat(winAnt,1,numel(range_view),numel(ramps)),[3 2 1]);
%winAnt3D            =   permute(repmat(winAnt,1,numel(range_view),numel(vel_view)),[3 2 1]);

% Positions for polar plot of cost function
vU                  =   linspace(-1, 1, NFFTAnt);
[mRange , mU]       =   ndgrid(range_view, vU);
mX                  =   mRange.*mU;
mY                  =   mRange.*cos(asin(mU));

% Color maps
cmap_blue = [0 0 0;0 0.0277777779847383 0.0277777779847383;0 0.0555555559694767 0.0555555559694767;0 0.0833333358168602 0.0833333358168602;0 0.111111111938953 0.111111111938953;0 0.138888895511627 0.138888895511627;0 0.16666667163372 0.16666667163372;0 0.194444447755814 0.194444447755814;0 0.222222223877907 0.222222223877907;0 0.25 0.25;0 0.277777791023254 0.277777791023254;0 0.305555552244186 0.305555552244186;0 0.333333343267441 0.333333343267441;0 0.361111104488373 0.361111104488373;0 0.388888895511627 0.388888895511627;0 0.416666656732559 0.416666656732559;0 0.444444447755814 0.444444447755814;0 0.472222208976746 0.472222208976746;0 0.5 0.5;0 0.527777791023254 0.527777791023254;0 0.555555582046509 0.555555582046509;0 0.583333313465118 0.583333313465118;0 0.611111104488373 0.611111104488373;0 0.638888895511627 0.638888895511627;0 0.666666686534882 0.666666686534882;0 0.694444417953491 0.694444417953491;0 0.722222208976746 0.722222208976746;0 0.75 0.75;0 0.777777791023254 0.777777791023254;0 0.805555582046509 0.805555582046509;0 0.833333313465118 0.833333313465118;0 0.861111104488373 0.861111104488373;0 0.888888895511627 0.888888895511627;0 0.916666686534882 0.916666686534882;0 0.944444417953491 0.944444417953491;0 0.972222208976746 0.972222208976746;0 1 1;0 1 1;0 1 1;0 1 1;0 1 1;0 1 1;0 1 1;0 1 1;0 1 1;0 1 1;0 1 1;0 1 1;0 1 1;0 1 1;0 1 1;0 1 1;0 1 1;0 1 1;0 1 1;0 1 1;0 1 1;0 1 1;0 1 1;0 1 1;0 1 1;0 1 1;0 1 1;0 1 1];
cmap_red = [0 0 0;0.0277777779847383 0.00838780030608177 0.0194989107549191;0.0555555559694767 0.0167756006121635 0.0389978215098381;0.0833333358168602 0.0251633990556002 0.0584967322647572;0.111111111938953 0.0335512012243271 0.0779956430196762;0.138888895511627 0.0419389978051186 0.0974945574998856;0.16666667163372 0.0503267981112003 0.116993464529514;0.194444447755814 0.0587145984172821 0.136492371559143;0.222222223877907 0.0671024024486542 0.155991286039352;0.25 0.0754901990294456 0.175490200519562;0.277777791023254 0.0838779956102371 0.194989114999771;0.305555552244186 0.0922657996416092 0.21448802947998;0.333333343267441 0.100653596222401 0.233986929059029;0.361111104488373 0.109041400253773 0.253485858440399;0.388888895511627 0.117429196834564 0.272984743118286;0.416666656732559 0.125817000865936 0.292483657598495;0.444444447755814 0.134204804897308 0.311982572078705;0.472222208976746 0.142592594027519 0.331481486558914;0.5 0.150980398058891 0.350980401039124;0.527777791023254 0.159368202090263 0.370479315519333;0.555555582046509 0.167755991220474 0.389978229999542;0.583333313465118 0.176143795251846 0.409477144479752;0.611111104488373 0.184531599283218 0.428976058959961;0.638888895511627 0.19291940331459 0.448474943637848;0.666666686534882 0.201307192444801 0.467973858118057;0.694444417953491 0.209694996476173 0.487472772598267;0.722222208976746 0.218082800507545 0.506971716880798;0.75 0.226470589637756 0.526470601558685;0.777777791023254 0.234858393669128 0.545969486236572;0.805555582046509 0.2432461977005 0.565468430519104;0.833333313465118 0.251634001731873 0.584967315196991;0.861111104488373 0.260021805763245 0.604466259479523;0.888888895511627 0.268409609794617 0.62396514415741;0.916666686534882 0.276797384023666 0.643464088439941;0.944444417953491 0.285185188055038 0.662962973117828;0.972222208976746 0.293572992086411 0.68246191740036;1 0.301960796117783 0.701960802078247;1 0.301960796117783 0.701960802078247;1 0.301960796117783 0.701960802078247;1 0.301960796117783 0.701960802078247;1 0.301960796117783 0.701960802078247;1 0.301960796117783 0.701960802078247;1 0.301960796117783 0.701960802078247;1 0.301960796117783 0.701960802078247;1 0.301960796117783 0.701960802078247;1 0.301960796117783 0.701960802078247;1 0.301960796117783 0.701960802078247;1 0.301960796117783 0.701960802078247;1 0.301960796117783 0.701960802078247;1 0.301960796117783 0.701960802078247;1 0.301960796117783 0.701960802078247;1 0.301960796117783 0.701960802078247;1 0.301960796117783 0.701960802078247;1 0.301960796117783 0.701960802078247;1 0.301960796117783 0.701960802078247;1 0.301960796117783 0.701960802078247;1 0.301960796117783 0.701960802078247;1 0.301960796117783 0.701960802078247;1 0.301960796117783 0.701960802078247;1 0.301960796117783 0.701960802078247;1 0.301960796117783 0.701960802078247;1 0.301960796117783 0.701960802078247;1 0.301960796117783 0.701960802078247;1 0.301960796117783 0.701960802078247];

%% RD demo
figure(1);clf;
box on;

boardRadar.startMeasurements();

for idx=1:1:2000
    idx
    
    [measurement, timeStamp] = board.getFrameBuffer(20000);
    
    % Data to extract is Tx1 and Tx3 from the linear measurement memory
    % Reshape to get rid of unwanted half of FFT and remap the Rx channels
    RD_data = typecast(measurement, 'int32');
    %RD_data = double(complex(RD_data(1:2:end), RD_data(2:2:end)));
    %plot(abs(RD_data))
    
    RD_data = double(reshape(complex(RD_data(1:2:end), RD_data(2:2:end)), ramps*NoRange, []));
    %RD_data = RD_data(1:ramps*NoRange,[3 4 1 2 7 8 5 6]);
    RD_data = RD_data(1:ramps*NoRange,[1 2 3 4 5 6 7 8]);
    %plot(abs(RD_data(:,1)))
    
    % Reshape to doppler x range x rx
    RD_data = reshape(RD_data,ramps,NoRange,8);
    
    % Take maximums over doppler but ignore zero doppler and edge bins
    %range_profile = squeeze(RD_data(:,1,:));
    %range_profile = squeeze(max(RD_data(3:ramps-1,:,:),[],1));
    range_profile = fftshift(RD_data,1); %shift the doppler dimension
    %range_profile_FOV    =   range_profile(min_range_idx : max_range_idx, :);  
    %range_profile_FOV = range_profile([min_vel_idx:zero_vel_ind-static_vel_filter zero_vel_ind+static_vel_filter:max_vel_idx], min_range_idx:max_range_idx, :);
    range_profile_FOV = range_profile(:, min_range_idx:max_range_idx, :);
    %plot(abs(range_profile))
    %plot(unwrap(angle(range_profile)))
    %spec = fftshift(fft(range_profile_FOV.*winAnt2D,NFFTAnt,2)/scale_winAnt,2);
    spec = fftshift(fft(range_profile_FOV.*winAnt3D,NFFTAnt,3)/scale_winAnt,3);
    
    % collapse down to maximum signal across desired doppler bins
    [dopMax, dopMaxInd] = max(spec(vel_view_ind,:,:),[],1);
    %[dopMax, dopMaxInd] = max(spec,[],1);
    spec_dop_max = squeeze(dopMax);
    JdB              =   10.*log10(abs(spec_dop_max));

    % remove anything outside of the desired FOV
    % angle is not actually linear at this stage
    %JdB(:, [1:round((NFFTAnt-1)/2*(sind(-60) + 1)+1) round((NFFTAnt-1)/2*(sind(60) + 1)+1):end]) = -10;
    
    %[JMax, idx_JMax] =   max(JdB(:));
    JMax = 55;
    JNorm            =   JdB - JMax;
    JNorm(JNorm < -10)  =   -10;
    
    % find potential targets
    [cent, centInd] = FastPeakFind(JdB, JMax*0.85);
    % convert results to cartesian coordinates and lookup max doppler
    centX = mX(centInd==1);
    centY = mY(centInd==1);
    centVel = vel_view(dopMaxInd(:,centInd==1));
    
    % also get the static targets for display
    [dopMax, ~] = max(spec(zero_vel_ind-static_vel_filter+1:zero_vel_ind+static_vel_filter-1,:,:),[],1);
    spec_dop_max = squeeze(dopMax);
    JdB              =   10.*log10(abs(spec_dop_max));
    JMax = 60; % less sensitive to static targets
    JNormStatic            =   JdB - JMax;
    JNormStatic(JNormStatic < -10)  =   -10;
    
    % Extract a single RDM
%     RD_plot = RD_data.*conj(RD_data); % signal power
%     RD_plot = 10*log10(RD_plot(:,:,1));
%     RD_plot(RD_plot<0) = 0;
% 
%     h1 = surf(v_vect,r_vect,RD_plot.');
%     colormap 'jet';
%     caxis([60 130]);
%     shading interp;
%     %set(gca,'ztick',[]) 
%     view([0 90])
%     ylim([0 max(r_vect)]);
%     xlim([-vmax max(v_vect)]);
%     %zlim([40 140])
%     ylabel('Range (m)');
%     xlabel('Velocity (m/s)');
%     zlabel('Magnitude (dB)');
%     %set(gca,'FontSize',18);

    %polar plot
    h2 = surf(mX,mY,JNorm);hold on;
    surf(mX,mY,JNormStatic);hold off;
    set(gca,'Color',[0.2 0.2 0.2])
    shading flat;
    view(0,90);
    xlim([-5 5]);
    ylim([0 5]);
    caxis([-10 0]);
    axis image; xlabel('x (m)'); ylabel('y (m)'); colormap('bone'); %jet
    
    for jdx = 1:1:length(centX)
        if centVel(jdx) < 0
            rectangle('Position',[centX(jdx)-0.5,centY(jdx)-0.5,1,1],'LineWidth',2,'LineStyle','-','EdgeColor','c')
        elseif centVel(jdx) > 0
            rectangle('Position',[centX(jdx)-0.5,centY(jdx)-0.5,1,1],'LineWidth',2,'LineStyle','-','EdgeColor','r')
        else
            rectangle('Position',[centX(jdx)-0.5,centY(jdx)-0.5,1,1],'LineWidth',2,'LineStyle','-','EdgeColor','y')
        end
    end
    
    drawnow;
end

boardRadar.stopMeasurements();


%%%%%%%%% Destroy the board object %%%%%%%%%
board.delete();

%for debugging, unload the wrapper dll to be able to update it
clear wrapper_matlab
