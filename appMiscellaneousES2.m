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

%% ES2 RAM update
radarRxs.initialize(); % Initialize RXS (FW, CRC settings, etc.)
%fw_vrxs_es2_ram(radarRegisterss);

%% Check version of AURIX democode
version = board.getVersion();
fprintf('Chipkit evaluation suite: %s\n', version);

%% Check version of MMIC firmware 
[chipVersion, revision] = radarRxs.Get_firmware_version();
fprintf('RXS chip version: %d\n', chipVersion);
fprintf('RXS firmware version: %d\n', revision);

%% MMIC register access
% read single registers
value = radarRegisters.getRegister('DMUX1_MAP_REG');         % registers can be read via the name 
value = radarRegisters.readRegister('0x0434');                  % or via their offset address

% set single registers
radarRegisters.setRegister('DMUX1_MAP_REG','0x0001');  % registers can be set via the their name 
radarRegisters.writeRegister('0x0434','0x0002');           % or via their offset address

% a warning might be switched on to check written and read back value, e.g.
radarRegisters.setRegister('DMUX1_MAP_REG','0x0001','on'); % no warning, written value matches read back value

% reads in the registers list of the RXS8160PL ES2
regList = radarRegisters.getRegisterList(); % reads back all registers from MMIC and returns a table with the register list
radarRegisters.showRegList();      % displays lates version of read back registers

% reads in the registers list of the RXS8160PL ES2 and stores it to *.mat file 
% e.g. P2S.RadarModule.Mmic.saveRegList(); % reads and stores register list + date, Name is optional but not required (will be generated autom.)
radarRegisters.saveRegList('RegisterList1');        % read back register status from MMIC and save it
radarRegisters.setRegister('DMUX1_MAP_REG','0x0004')  % registers can be set via the their name 
radarRegisters.saveRegList('RegisterList2');        % read back register status from MMIC and save it
diffTable = compareRegLists('RegisterList1', 'RegisterList2', 'Content')   % shows differences between 2 register lists

% Do not read from cu_call_fifo_reg and cu_status_fifo_reg.
% OK pin will signal FIFO underflow event otherwise.
skip = ['0x0426';      % cu_call_fifo_reg
        '0x042A'];     % cu_status_fifo_reg
    
map_0400_043C = radarRegisters.getMemoryMap('0x0400', '0x043C', skip);
map_0200_021A = radarRegisters.getMemoryMap('0x0200', '0x021A', []);

%% MMIC Memory access
% with the functions getRegister()/setRegister() only defined registers of
% the MMIC van be addressed. To access any memory are of the MMIC use the
% functions readRegister()/writeRegister()/set/clear/modifyRegisterBits().

radarRegisters.writeRegister('0x4000', 4);     % write value 4 to address 0x4000

radarRegisters.setRegisterBits('0x4000', '0x0003');    % set bit 0 and 1 at address 0x4000
radarRegisters.clearRegisterBits('0x4000', '0x0001');    % reset bit 0 at address 0x4000

value = radarRegisters.readRegister('0x4000');   % read value from address 0x4000

  
radarRxs.softReset();    

%%%%%%%%% Destroy the board object %%%%%%%%%
board.delete();
%for debugging, unload the wrapper dll to be able to update it
clear wrapper_matlab
