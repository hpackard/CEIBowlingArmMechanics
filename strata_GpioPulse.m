%% connect to board
board = strata.utils.connectToBoard();
[vid, pid] = board.getIds()
boardRadar = board.getIBoardRadar();

gpioPulse = strata.custom.GpioPulse(board);

%%%%%%%%% Configure a finite amount of GPIO pulses %%%%%%%%%
gpioPulse.stop()   % stop any previously started GPIO pulse generation

count = 10;        % amount of GPIO pulses to be generated
duration = 0.0001; % duration of a single pulse (GPIO logic high state)
delay = 0.0003;    % delay between the synchronized start of both GPIO pulses
period = 0.001;    % time interval between the start of consecutive pulses

x601_dmux5 = strata.GpioDefines.GPIO_ID(10, 3);
x602_dmux5 = strata.GpioDefines.GPIO_ID(10, 8);

gpioPulse.configure(x601_dmux5, count, duration, 0, period)
gpioPulse.configure(x602_dmux5, count, duration, delay, period)
gpioPulse.start()

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Note: you must call stop() and configure(), before another start()
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%% Destroy the board object %%%%%%%%%
board.delete();
%for debugging, unload the wrapper dll to be able to update it
clear wrapper_matlab
