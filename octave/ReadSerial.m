pkg load instrument-control
supportedinterfaces = instrhwinfo().SupportedInterfaces;

if !isempty(strfind (supportedinterfaces , "serial"))
    disp("instrument control, Serial: Supported")
else
    disp("instrument control, Serial: Unsupported")
endif

% Opens serial port /dev/tty* with baudrate of 115200 (config defaults to 8-N-1)
s1 = serial("/dev/ttyACM0", 115200);

% initialize the data structures
LineInterval = 0.005; % seconds
CaptureTime = 60;   % seconds
LineCount = CaptureTime / LineInterval;
ErrorCount = 0;

LineLen = 100; % characters
ParamCount = 9;
Line = uint8(20 + zeros(LineCount, LineLen)); % fill with ascii code for blank
Data = zeros(LineCount, ParamCount);
% Format: Time(usec), Vt, Vb, Vwt, Vwb, Rwt, Rwb, Rt, azimuth
LineFormat = '%f, %f, %f, %f, %f, %f, %f, %f, %f';

% Flush input and output buffers
srl_flush(s1);
% initial alignment
while srl_read(s1, 1) ~= 13 % scan character stream for return
end
srl_read(s1, 1); % consume the next character (linefeed)

%% read and discard 3 lines
% somehow there is a glitch in time between 3rd and 4th lines
for LineIdx = 1:3   % for 3 lines at startup
  for CharIdx = 1:LineLen     % for each char on line
    chr = srl_read(s1, 1);  % read one character
    if chr == 13            % looking for \r, return
      srl_read(s1, 1);      % consume the \n, linefeed
      break;
    end
  end % for each char on line
end % for three lines

% read lines from serial port, assemble into text strings, convert to numbers
LineIdx = 1;
LineData = zeros(1, ParamCount);
while LineIdx <= LineCount % for each line
  % read a line terminated in return
  LineChr = zeros(1, LineLen);
  LineStr = zeros(1, LineLen);
  for CharIdx = 1:LineLen          % for each char on line
    chr = srl_read(s1, 1);    % read one character
    if chr == 13              % looking for \r, return
      srl_read(s1, 1);        % consume the \n, linefeed
      break;                  % end of line found, exit the character reading loop
    end
    LineChr(CharIdx) = chr; % assemble the line as character numbers
  end
  LineStr = char(LineChr);
  DataStr(LineIdx, :) = LineStr;

  % discard bad lines read from serial port
  try % to convert to floating point numbers
    LineData = sscanf(LineStr, LineFormat); % assemble floating point array
  catch
    ErrorCount = ErrorCount + 1;
    fprintf(1, 'Error %d, ReadSerial: sscanf failed with "%s" \n', ErrorCount, LineStr);
    continue; % skip to next line
  end

  if length(LineData) ~= ParamCount
    ErrorCount = ErrorCount + 1;
    fprintf(1, 'Error %d, ReadSerial: line has %d elemennts, not %d\n', length(LineData), ParamCount);
    continue;
  end

  % handle first Line
  if LineIdx == 1
    TimeNow = LineData(1) / 1e6; % sec
    TimePrev = TimeNow - LineInterval; % artificial TimePrev
  end

  % check if glitch in time
  TimeNow = LineData(1) / 1e6; % sec
  if abs(TimeNow - TimePrev) > 10 * LineInterval % sec, 10 samples to allow several glitches in a row
    ErrorCount = ErrorCount + 1;
    fprintf(1, 'Error %d, ReadSerial: glitch in time\n', TimePrev);
    continue; % skip to next line
  end

  Data(LineIdx, :) = LineData; % record the data
  LineIdx = LineIdx + 1;
  TimePrev = TimeNow;
end % while SampleIdx < SampleCount

fprintf(1, 'stop rotation and close input')
srl_write(s1, 'A'); % stop azimuth rotation
srl_write(s1, char(13)); % Line feed
srl_write(s1, char(10)); % Carriage return
fclose(s1);

% end of input reading,

%% process data and plot
ColIdx = 0;
ColIdx += 1; Tusec   = Data(:, ColIdx); % Time in microseconds since power up
ColIdx += 1; Vtop    = Data(:, ColIdx); % voltage on top of pot
ColIdx += 1; Vbot    = Data(:, ColIdx); % voltage on bottom of pot
ColIdx += 1; Vwipert = Data(:, ColIdx); % voltage on top of pot
ColIdx += 1; Vwiperb = Data(:, ColIdx); % voltage on bottom of pot
ColIdx += 1; Rwipert = Data(:, ColIdx); % voltage on top of pot
ColIdx += 1; Rwiperb = Data(:, ColIdx); % voltage on bottom of pot
ColIdx += 1; Rtop    = Data(:, ColIdx); % Resistor value on bottom of pot
ColIdx += 1; azimuth = Data(:, ColIdx); % mapped through calibration

Tsec       = Tusec / 1e6;
TimeStart  = Tsec(1);
TimeOffset = (Tsec - TimeStart);
TimeMin    = 0; % min(TimeOffset);
TimeMax    = max(TimeOffset);

%% Plots
figure(1); clf;
SubplotCnt = 3;
SubplotIdx = 0; % index is preincremented

% Voltage Plot
SubplotIdx = SubplotIdx + 1;
sx1 = subplot(SubplotCnt, 1, SubplotIdx); hold on; grid on;
title('Voltage reading top and bottom');
xlabel('time (sec)');
ylabel('Voltage, (Volts)');
xlim([TimeMin TimeMax]);
plot(TimeOffset, Vtop, 'b.');
plot(TimeOffset, Vbot, 'r.');
legend('Top', 'Bottom');

% Resistance Plot
SubplotIdx = SubplotIdx + 1;
sx2 = subplot(SubplotCnt, 1, SubplotIdx); hold on; grid on;
% next plottitle('resistor top');
xlabel('time (sec)');
ylabel('Resistor (Ohms)');
xlim([TimeMin TimeMax]);
ylim([0 600]);
plot(TimeOffset, Rtop, '.b');
legend('Rtop');

% Azimuth plot
SubplotIdx = SubplotIdx + 1;
sx3 = subplot(SubplotCnt, 1, SubplotIdx); hold on; grid on;
xlim([TimeMin TimeMax]);
ylim([0 400]);
TitleStr = sprintf('Az Top, Map');
title('Azimuth top and mapped');
xlabel('time (sec)');
ylabel('azimuth (deg)');
plot(TimeOffset, 360 * Rtop / 500, '.b'); % convert Rtop to azimuth before calibration
plot(TimeOffset, azimuth,          '.r');
legend('Az OEM', 'Az HCO');
linkaxes([sx1, sx2, sx3], 'x');

figure(2);
clf; hold on; grid on;
title('Rtop');
xlabel('Azimuth (deg)');
ylabel('Resistance (ohms)');
plot(azimuth, Rtop, 'b.');

figure(3);
clf; hold on; grid on;
title('Wiper votages estimate');
xlabel('time (sec)');
ylabel('Voltage');
plot(TimeOffset, Vwipert, 'b.');
plot(TimeOffset, Vwiperb, 'r.');

figure(4);
clf; hold on; grid on;
title('Wiper resistance estimate');
xlabel('Azimuth (deg)');
ylabel('Resistance (ohms)');
plot(TimeOffset, Rwipert, 'b.');
plot(TimeOffset, Rwiperb, 'r.');

