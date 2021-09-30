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
[ramper, cfg] = cfgConstantFrequencyES2(radarRxs);

%% ES2 RAM update
radarRxs.initialize(); % Initialize RXS (FW, CRC settings, etc.)
%fw_vrxs_es2_ram(radarRegisterss);

%% set RF frequency range for KVCO Calibration
fmin=77e9;
fmax=78e9;

cfg.f_kvco_min = fmin;
cfg.f_kvco_max = fmax;
cfg.rmpCfg.f_delta=fmax-fmin;
cfg.rmpCfg.f_sin=(fmax+fmin)/2;
cfg.df_kvco = abs(cfg.rmpCfg.f_delta);

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


%Define RF frequency vector; Example here: 3 RF frequencies 
%Note: RF frequencies have to be within: fmin and fmax
f_vect=linspace(fmin,fmax,3);
%Enable TX channel if required
TX1_ON=0;
TX2_ON=1;
TX3_ON=0;
%enable DIV32 Output if required
DIV32_enable=1;

if DIV32_enable==1
    radarRegisters.setRegisterBits('0x022E', '0x2000'); % enable divider output for test
else
    radarRegisters.clearRegisterBits('0x022E', '0x2000');
end;

TX_enable=TX1_ON+2*TX2_ON+4*TX3_ON;

for idx_f=1:1:length(f_vect)

fPLL=f_vect(idx_f);
radarRxs.Calibration(cfg.f_kvco_min, cfg.f_kvco_max, cfg.df_kvco, 0);
radarRxs.Static_frequency(fPLL);
radarRegisters.writeRegister('0x0208', TX_enable) % RMP_DEF_CONF_LW_REG
%*****************************
%*   Do Measurement here!    *
%*****************************
pause()

end;

radarRxs.softReset();    

%%%%%%%%% Destroy the board object %%%%%%%%%
board.delete();
%for debugging, unload the wrapper dll to be able to update it
clear wrapper_matlab
