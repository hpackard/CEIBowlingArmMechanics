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

%% create ramper object and get config struct
[ramper, cfg] = cfgRampLinearityES2_part(radarRxs);

%% ES2 RAM update
radarRxs.initialize(); % Initialize RXS (FW, CRC settings, etc.)
%fw_vrxs_es2_ram(radarRegisterss);

%% send Ramp config which is created in the cfg function
ramper.sendRampMemory();

%% configure MMIC
radarRxs.Static_frequency(cfg.rmpCfg.f_sin);

radarRxs.Enable_TX(cfg.txCfg.TX1_enable, cfg.txCfg.TX2_enable, cfg.txCfg.TX3_enable, cfg.txCfg.operationMode);

radarRxs.Set_TX_power(cfg.tx1_dac, cfg.tx2_dac, cfg.tx3_dac);

radarRxs.Enable_RX(cfg.rxCfg.RX1_enable, cfg.rxCfg.RX2_enable, cfg.rxCfg.RX3_enable, cfg.rxCfg.RX4_enable, cfg.rxCfg.LP_gain, cfg.rxCfg.HP_gain, cfg.rxCfg.DCOC_shift, cfg.rxCfg.DCOC_enable, cfg.rxCfg.RX_settings);

radarRxs.Set_lvds_frame_delay(cfg.Time_rise_delay, cfg.Time_fall_delay);

%% calibration of MMIC
status = radarRxs.LVDS_calib_start();
status = radarRxs.LVDS_calib_stop();

radarRxs.One_time_calib();
radarRxs.Calibration(cfg.f_kvco_min, cfg.f_kvco_max, cfg.df_kvco, 0);

%radarRxs.enableDataCrc(false); % disable LVDS CRC
radarRxs.enableDividerOutput(true); % enable divider output for test

radarRegisters.setRegisterBits('0x206', '0x0400'); % inf loops allowed

% These settings are supposed to be treated as default settings for fast
% ramping
radarRegisters.writeRegister('0x020E', '0x4A52');
radarRegisters.writeRegister('0x020C', '0x4210');
radarRegisters.writeRegister('0x0214', '0x0294');
radarRegisters.writeRegister('0x0880', '0x36F1');
val = radarRegisters.readRegister('0x0882');
radarRegisters.writeRegister('0x0884', bitor(val, 4096));


%% ramp scenario
  
radarRxs.Start_rmp_scenario(hex2dec('4000'), 0, 0);
%*****************************
%*   Do Measurement here!    *
%*****************************
pause();    

radarRxs.softReset();    

%%%%%%%%% Destroy the board object %%%%%%%%%
board.delete();
%for debugging, unload the wrapper dll to be able to update it
clear wrapper_matlab
