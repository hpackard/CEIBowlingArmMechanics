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

%% Extra configuration
radarRxs = boardRadar.getIRadarRxs(0);
radarRegisters = radarRxs.getIRegisters();
radarRxs.enableDividerOutput(true); % enable divider output for test
radarRegisters.setRegisterBits('0x042C', '0x0001'); % DMUX1 as output
radarRegisters.writeRegister('0x0434', '0x0020'); % DMUX1 map DMUX_A

%%
fcw_tones = [76 76.5 77 77.5 78 78.5 79 79.5 80]*10^9;

radarRxs.Enable_TX(0,0,1, 0)
radarRxs.Set_TX_power(0,0,32)
radarRegisters.setRegisterBits('0x0208', '0x0004'); % 

%radarRxs.Calibration(fcw_tones(1), fcw_tones(1) + 100e6, 100e6, 1)

for cw = 1: 1 : length(fcw_tones)%: -1 : 1
%cw=3; 
radarRxs.Calibration(fcw_tones(cw)-0e6, fcw_tones(cw) + 200e6, 200e6, 0)
    pause(1)
    radarRxs.Static_frequency(fcw_tones(cw))
    disp([num2str(fcw_tones(cw)/10^9),' GHz CW tone set, hit a key to proceed'])
    pause 
end  


%%
radarRxs.softReset();


%%%%%%%%% Destroy the board object %%%%%%%%%
board.delete();

%for debugging, unload the wrapper dll to be able to update it
clear wrapper_matlab
