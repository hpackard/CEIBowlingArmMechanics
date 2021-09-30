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
[ramper, cfg] = cfgRangeDoppler(radarRxs);

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

radarRxs.enableDataCrc(false); % disable LVDS CRC
radarRxs.enableDividerOutput(true); % enable divider output for test

% number of enabled Transmitters
NoTX = cfg.txCfg.TX1_enable + cfg.txCfg.TX2_enable + cfg.txCfg.TX3_enable; 

% Number of enabled RIF channels
% (if rif1_ch2 is enabled rif1_ch1 has to be enabled aswell even if corresponding RX1 chain in RXS is disabled)
EnabledRX = [cfg.rxCfg.RX1_enable, cfg.rxCfg.RX2_enable, cfg.rxCfg.RX3_enable, cfg.rxCfg.RX4_enable];
NoRX = max(find(EnabledRX));

%% configure RIF

% board specific data index of used RIF
[vid, pid] = board.getIds();
% 0 for all boards, except 1 for Board_5A
if pid == hex2dec('5A')
    dataIndex = 1;
else
    dataIndex = 0;
end

% general LVDS data config: bit order, CRC
cfgData.channelSwapping = 0;
cfgData.lsbFirst = false;
cfgData.crcEnabled = false;
cfgData.crcMswFirst = true;
radarData.initializeData(dataIndex, cfgData);

% configuration of radar data alignment: ramps, number of samples, ...
cfgRadarData.rxChannels = max(find([cfg.rxCfg.RX1_enable, cfg.rxCfg.RX2_enable, cfg.rxCfg.RX3_enable, cfg.rxCfg.RX4_enable]));
cfgRadarData.ramps = cfg.rmpCfg.NoRamps;
cfgRadarData.samples = getNoSamples(cfg.rmpCfg.t_ramp, getDecimation(cfg.rxCfg.RX_settings), cfg.Time_rise_delay, cfg.Time_fall_delay);
cfgRadarData.bitWidth = getBitWidth(cfg.rxCfg.RX_settings);
cfgRadarData.txChannels = 1;
cfgRadarData.modulation = 0;
radarData.configureData(dataIndex, cfgRadarData);

%% configure SPU
processingDescriptor(1).dataSource = dataIndex;
processingDescriptor(1).radarDataInput = cfgRadarData;
processingDescriptor(1).stages =  cfg.fftCfg;
radarProcessing.configure(processingDescriptor, 1);

%Alternatively, we could use the 'writeConfigRam' function
%The whole Spu configuration that can be downloaded into RAM
%This is only for illlustration purposes, changes in the above
%configuration requires the '.mat' file to be generated again.
%load('config/cfgRangeDoppler_SpuRamCfg.mat','SpuRamCfg');
%radarProcessing.writeConfigRam(SpuRamCfg.id, SpuRamCfg.offset, SpuRamCfg.values)

%% scaling for plot
c0=3e8;
%sweep duration
tsw=(cfg.rmpCfg.t_ramp-cfg.Time_rise_delay);
%effective RF bandwidth
bsw=(cfg.rmpCfg.f_delta/cfg.rmpCfg.t_ramp*(tsw));
%pulse repetition time
PRT=cfg.rmpCfg.t_ramp+cfg.rmpCfg.t_jump+cfg.rmpCfg.t_wait;
%maximum velocity
vmax=c0/2/(cfg.rmpCfg.f_sin)/PRT/2;

r_vect=linspace(0,0.5,256)*(50e6/8)*tsw/bsw*c0/2;
v_vect=linspace(-0.5,0.5-1/16,16)*2*vmax;


RD_offset = 0;
RD_size = cfgRadarData.samples*cfgRadarData.ramps*4;

figure(1);clf;
box on;

radarData.startData(dataIndex);

%% RD demo
for idx=1:1:10000
    idx
    
    radarProcessing.run();

    radarRxs.Start_rmp_scenario(hex2dec('4000'), 0, 0);
    
    while radarProcessing.isBusy()
        pause(0.01);
    end

    measurement = accessMemory.readMem(hex2dec('B9000000')+RD_offset, RD_size);
    ememData = typecast(measurement(1:RD_size/4), 'int32');

    RD_data = complex(ememData(1:2:end), ememData(2:2:end));

    RD_plot = abs(double(RD_data));
    RD_plot = reshape(RD_plot,cfgRadarData.ramps,cfgRadarData.samples/2).';
    RD_plot = fftshift(RD_plot,2);
    RD_plot = 20*log10(RD_plot);

    if idx==1
        h=surf(v_vect,r_vect,RD_plot);
        colormap 'jet';
        caxis([60 130]);
        shading interp;
        set(gca,'ztick',[]) 
        view([30 120])
        ylim([0 10]);
        xlim([-2.5 2.5]);
        zlim([40 140])
        ylabel('Range (m)');
        xlabel('Velocity (m/s)');
        zlabel('Magnitude (dB)');
        set(gca,'FontSize',18);                 
    end
    
    if ~ishandle(h)
        break;
    end    
    set(h,'ZData',RD_plot);
    drawnow;    
end;

radarData.stopData(dataIndex);

radarRxs.softReset();

%%%%%%%%% Destroy the board object %%%%%%%%%
board.delete();
%for debugging, unload the wrapper dll to be able to update it
clear wrapper_matlab