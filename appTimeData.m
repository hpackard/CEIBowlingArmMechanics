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
[ramper, cfg] = cfgTimeData(radarRxs);

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
cfgRadarData.ramps = cfg.rmpCfg.NoRamps;
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

%% ramp scenario
for idx=1:1:1
    radarProcessing.run();
    
    %IRPLI.rxs.BeginLvdsTransfer('NumWords', cfgRadarData.samples*cfgRadarData.ramps-1);

    radarRxs.Start_rmp_scenario(hex2dec('4000'), 0, 0);
    
    %data = IRPLI.rxs.EndLvdsTransfer();
    %radarData.stopData(dataIndex);
    
    while radarProcessing.isBusy()
        pause(0.01);
    end
    
    okPin = radarRxs.getStatus();
    crcStatus = radarData.wasCrcErrorDetected(dataIndex);
    
    % start from begin of EMEM and read NoSamples*NoRamps*NoRx Samples (4=>32bit, 2=>real+imag)
    ememData = accessMemory.readMem('0xB9000000', cfgRadarData.samples*cfgRadarData.ramps*NoRX*4*2);
    
    ememData = bitshift(ememData, -(32-cfgRadarData.bitWidth));
    ememData = ememData(1:2:end-1);
    ememData = reshape(ememData, cfgRadarData.samples, NoRX, cfgRadarData.ramps);
    
    ifData = [];
    for idx=1:size(ememData,3)
        ifData = [ifData; ememData(:,:,idx)];
    end
    ifData = twoComp(ifData, cfgRadarData.bitWidth);
    
    cnt = 1;
    for idx_rx=1:1:NoRX
        sif(idx_rx,:,:,:) = reshape(ifData(:,idx_rx),size(ifData,1)/(cfgRadarData.ramps*NoTX),NoTX,cfgRadarData.ramps);
        if EnabledRX(idx_rx)
            %****************************************************
            %    sif(TX-idx,RX-idx,NumberOfSamples,NumberOfRamps)                 
            %    scaled to fullscale +-1
            %****************************************************
            for idx_tx=1:1:NoTX
                figure(cnt);
                clf;
                hold on;
                plot(mean(squeeze(sif(idx_rx, :,idx_tx,:)),2),'r');
                plot(squeeze(sif(idx_rx,:,idx_tx,1)),'b');
                plot(squeeze(sif(idx_rx,:,idx_tx,2:1:end)),'k');
                plot(mean(squeeze(sif(idx_rx, :,idx_tx,:)),2),'r');%plotting again; for visibility in figure
                hold off;
                grid on;
                box on;
                legend('Mean value','First ramp','Other ramps');
                title(['RX' num2str(idx_rx) ' / TX' num2str(idx_tx)]);
                xlabel('Samples (1)');
                ylabel('Amplitude (FS)');
                %ylim([-1 1]);
                cnt=cnt+1;
                drawnow;
            end
        end
    end
    
end

radarData.stopData(dataIndex);

radarRxs.softReset();

%%%%%%%%% Destroy the board object %%%%%%%%%
board.delete();
%for debugging, unload the wrapper dll to be able to update it
clear wrapper_matlab