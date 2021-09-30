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
clearvars;format compact;
close all

%webcamlist
%cam = webcam('Logitech QuickCam S5500');
%cam.Resolution = '960x720';
%preview(cam)

% Add sub folders to the project include paths
addpath(genpath(cd))

%% connect to board
board = strata.utils.connectToBoard();
boardRadar = board.getIBoardRadar();
radarRxs = boardRadar.getIRadarRxs(0);
radarRegisters = radarRxs.getIRegisters();
radarProcessing = boardRadar.getIProcessingRadar();
radarData = board.getIAccessData();
accessMemory = board.getIAccessMemory();

%% create ramper object and get config struct
[ramper, cfg] = cfgDBF_from_TimeData(radarRxs);

%% ES2 RAM update
radarRxs.initialize(); % Initialize RXS (FW, CRC settings, etc.)
%fw_vrxs_es2_ram(radarRegisters);

%% send Ramp config which is created in the cfg function
ramper.sendRampMemory();

%% configure MMIC
radarRxs.Static_frequency(cfg.rmpCfg.f_sin);

radarRxs.Enable_TX(cfg.txCfg.TX1_enable, cfg.txCfg.TX2_enable, cfg.txCfg.TX3_enable, cfg.txCfg.operationMode);
radarRegisters.writeRegister('0x0846', '0x0000');   % BBD_EN_REG

radarRxs.Set_TX_power(cfg.tx1_dac, cfg.tx2_dac, cfg.tx3_dac);

radarRxs.Enable_RX(cfg.rxCfg.RX1_enable, cfg.rxCfg.RX2_enable, cfg.rxCfg.RX3_enable, cfg.rxCfg.RX4_enable, cfg.rxCfg.LP_gain, cfg.rxCfg.HP_gain, cfg.rxCfg.DCOC_shift, cfg.rxCfg.DCOC_enable, cfg.rxCfg.RX_settings);
radarRegisters.writeRegister('0x0846', '0x0000');   % BBD_EN_REG

radarRxs.Set_lvds_frame_delay(cfg.Time_rise_delay, cfg.Time_fall_delay);

%% calibration of MMIC
status = radarRxs.LVDS_calib_start();
status = radarRxs.LVDS_calib_stop();

radarRxs.One_time_calib();
radarRxs.Calibration(cfg.f_kvco_min, cfg.f_kvco_max, cfg.df_kvco, 0);
radarRxs.Static_frequency(cfg.rmpCfg.f_start);

radarRxs.enableDataCrc(false); % disable LVDS CRC
radarRxs.enableDividerOutput(true); % enable divider output for test

radarRegisters.setRegisterBits('0x042C', '0x0001'); % DMUX1 as output
radarRegisters.writeRegister('0x0434', '0x0020'); % DMUX1 map DMUX_A

% number of enabled Transmitters
NoTX = cfg.txCfg.TX1_enable + cfg.txCfg.TX2_enable + cfg.txCfg.TX3_enable; 

% Number of enabled RIF channels
% (if rif1_ch2 is enabled rif1_ch1 has to be enabled aswell even if corresponding RX1 chain in RXS is disabled)
EnabledRX = [cfg.rxCfg.RX1_enable, cfg.rxCfg.RX2_enable, cfg.rxCfg.RX3_enable, cfg.rxCfg.RX4_enable];
NoRX = max(find(EnabledRX));

%% configure RIF

% board specific data index of used RIF
[vid, pid] = board.getIds();
if pid == hex2dec('5A')
    dataIndex = 1; % Board5a
else
    dataIndex = 0; % other boards
end

% general LVDS data config: bit order, CRC
cfgData.channelSwapping = 0;
cfgData.lsbFirst = false;
cfgData.crcEnabled = false;
cfgData.crcMswFirst = true;
radarData.initializeData(dataIndex, cfgData);

% configuration of radar data alignment: ramps, number of samples, ...
cfgRadarData.rxChannels = max(find([cfg.rxCfg.RX1_enable, cfg.rxCfg.RX2_enable, cfg.rxCfg.RX3_enable, cfg.rxCfg.RX4_enable]));
cfgRadarData.ramps = cfg.rmpCfg.NoRamps*3;
cfgRadarData.samples = getNoSamples(cfg.rmpCfg.t_ramp, getDecimation(cfg.rxCfg.RX_settings), cfg.Time_rise_delay, cfg.Time_fall_delay);
cfgRadarData.bitWidth = getBitWidth(cfg.rxCfg.RX_settings);
cfgRadarData.txChannels = 1;
cfgRadarData.modulation = 0;
radarData.configureData(dataIndex, cfgRadarData);

%% config SPU
processingDescriptor(1).dataSource = dataIndex;
processingDescriptor(1).radarDataInput = cfgRadarData;
processingDescriptor(1).stages =  cfg.cfgFft;
radarProcessing.configure(processingDescriptor, 1);

radarData.startData(dataIndex);

%%
settings_DoA;
sIF = zeros(cfgRadarData.samples, NrMIMO);
%% ramp scenario
figure(1);clf;
for idx=1:1:2000
    radarProcessing.run();
    
    radarRxs.Start_rmp_scenario(hex2dec('4000'), 0, 0);
    
    idx
    while radarProcessing.isBusy()
        pause(0.01);
    end
    
    okPin = radarRxs.getStatus();
    crcStatus = radarData.wasCrcErrorDetected(dataIndex);
    
    % start from begin of EMEM and read NoSamples*NoRamps*NoRx Samples (4=>32bit, 2=>real+imag)
    ememData = accessMemory.readMem('0xB9000000', cfgRadarData.samples*3*cfgRadarData.ramps*NoRX*4*2);
    ememData = bitshift(ememData, -(32-cfgRadarData.bitWidth));
    ememData = twoComp(ememData(1:2:end-1), cfgRadarData.bitWidth);
    ememData = reshape(ememData, cfgRadarData.samples, NoRX, cfgRadarData.ramps*3);
    
    % Measurements of Transmitter 1
    tt = double(ememData(:, :, 1)) - 8192;
    tt = tt - repvec(mean(tt)', cfgRadarData.samples)';
    tt = tt(:, [3, 4, 1, 2]);               % reordering according CEI antenna arrangement
    sIF(:, 1:4)  = tt(:,1:4);
    
    % Measurements of Transmitter 2
    tt = double(ememData(:, :, 2)) - 8192;
    tt = tt - repvec(mean(tt)', cfgRadarData.samples)';
    sIF(:, 5:8)  = tt(:, [3, 4, 1, 2]);     % reordering according CEI antenna arrangement
    
    % Measurements of Transmitter 3
    tt = double(ememData(:, :, 3)) - 8192;
    tt = tt - repvec(mean(tt)', cfgRadarData.samples)';
    sIF(:, 9:12)  = tt(:, [3, 4, 1, 2]);     % reordering according CEI antenna arrangement
    
    % Plot Raw Data
    figure(23);clf
    subplot(3,1,1);hold on;box on;grid on;
    plot(sIF(:,1))
    plot(sIF(:,2))
    plot(sIF(:,3))
    plot(sIF(:,4))
    title('Tx1 = ON, RX1-4');
    xlim([0 cfgRadarData.samples])    
    ylim([-500 500]);
    
    subplot(3,1,2);hold on;box on;grid on;
    plot(sIF(:,5))
    plot(sIF(:,6))
    plot(sIF(:,7))
    plot(sIF(:,8))
    ylim([-500 500]);
    xlim([0 cfgRadarData.samples])
    title('Tx2 = ON, RX1-4')

    subplot(3,1,3);hold on;box on;grid on;
    plot(sIF(:,9))
    plot(sIF(:,10))
    plot(sIF(:,11))
    plot(sIF(:,12))
    ylim([-500 500]);
    xlim([0 cfgRadarData.samples])
    title('Tx3 = ON, RX1-4')
drawnow;

    % Calculate range profile including calibration
    range_profile       =   fft(sIF.*win2D.*mCalib, NFFT,1).*full_scale/scale_win;
    range_profile_FOV    =   range_profile(min_range_idx : max_range_idx, :);  
    
    %plot(unwrap(angle(range_profile.')));
    
    % calculate fourier transform over receive channels
    spec        =   fftshift(fft(range_profile_FOV.*winAnt2D,NFFTAnt,2)/scale_winAnt,2);
    
    % normalize cost function
    JdB              =   20.*log10(abs(spec));
    [JMax, idx_JMax] =   max(JdB(:));
    JNorm            =   JdB - JMax;
    JNorm(JNorm < -20)  =   -20;
    
    
    %--------------
    idx_max_range  = mod(idx_JMax , (max_range_idx - min_range_idx + 1));  % gets index in range of max peak 
    idx_angle_max =  floor(idx_JMax / (max_range_idx - min_range_idx + 1));% gets index in angle of max peak 
    
%     h=figure(31); clf; hold on;box on; grid on;
%     plot(angle_deg, JNorm(idx_max_range,:));
%     h=line([angle_deg(idx_angle_max) angle_deg(idx_angle_max)],[-20 -18]);
%     set(h,'Color',[1, 0, 0],'Linewidth',2);
%     ylim([-20, 0])
%     drawnow;

        
%     figure(3)
%     imagesc(angle_deg, range_view, JNorm);
%     xlabel('Ang (°)');
%     ylabel('R (m)');
%     colormap('jet');
    
    
    %    generate polar plot
    if idx==1
        h=surf(mX,mY, JNorm); 
        shading flat;
        view(0,90);
        axis equal; xlabel('x (m)'); ylabel('y (m)'); colormap('jet');
    end
    
    if ~ishandle(h)
        break;
    end
    set(h,'ZData',JNorm);
    drawnow;
   
end

radarData.stopData(dataIndex);

radarRxs.softReset();

%%%%%%%%% Destroy the board object %%%%%%%%%
board.delete();
%for debugging, unload the wrapper dll to be able to update it
clear wrapper_matlab