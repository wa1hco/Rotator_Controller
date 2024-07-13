% voltage calculations
Vs = 3.3;
Rp = 500;
Rs = 330;
ADCmax = 1023;
Az = (0:2:360); 
A  = Az / 360;
Rw = 0;
% 
% Vt = Vs * ((Rp *    A)  ./ (Rp *    A  + Rb));
% Vb = Vs * ((Rp * (1-A)) ./ (Rp * (1-A) + Rb));

% worksheet 4/30/22



%% Top formulas
Rt = Rp * Az / 360 ; % Resistor top from azimuth
Vt = Vs .* Rt ./ (Rs + Rt); % voltage reading by resistor top and bottom divider rule
%Vt = Vs * (Rp * Az / 360)  / (Rs + (Rp * Az / 360));  % plug in Rt = F(az), resistor from azimuth
%Vt * (Rs + (Rp * Az / 360)) = Vs * (Rp * Az / 360);   % multiply both sides by denominator
%Vt * Rs + Vt * Rp * Az / 360 = Vs * Rp * Az / 360;    % multiply through by Vt
%Vt * Rs = Vs * Rp * Az / 360 - Vt * Rp * Az / 360;    % Bring Az to one side
%Vs * Rp * Az / 360 - Vt * Rp * Az / 360 = Vt * Rs;    % reverse sides 
%Az * (Vs * Rp / 360 - Vt * Rp / 360) = Vt * Rs;       % factor out Az
%Az * Rp / 360 * (Vs - Vt) = Vt * Rs;                  % factor out Rp/360
%Az * Rp / 360 = Vt * Rs / (Vs - Vt);                  % move (Vs - Vt) to right
%Az = (360 / Rp) * Vt * Rs / (Vs - Vt);                % move Rp / 360 to right
AzTop = 360 / Rp * Rs .* Vt ./ (Vs - Vt);                  % rearrange

%% Bottom formulas, assuming Vwiper = 0
Rb = Rp * (1 - Az / 360);
Vb  = Vs .* Rb                  ./ (Rs + Rb); % V bottom assuming Vwiper = 0
% Vb = Vs * (Rp * (1 - Az / 360)) / (Rs + (Rp * (1 - Az / 360)));           % plug in resistor from azimuth
% Vb * (Rs + (Rp * (1 - Az / 360))) = Vs * (Rp * (1 - Az / 360));           % multiply both sides by denominator
% Vb * (Rs + (Rp - Rp * Az / 360))  = Vs * (Rp - Rp * Az / 360);            % multiply through by Rp
% Vb * Rs + Vb * Rp - Vb * Rp * Az / 360 = Vs * Rp - Vs * Rp * Az / 360));  % multiply through by Vb and Vs
% Vb * Rs + Vb * Rp = Vs * Rp - Vs * Rp * Az / 360 + Vb * Rp * Az / 360;    % move Az to right
% Vb * Rs + Vb * Rp - Vs * Rp = Vb * Rp * Az / 360 - Vs * Rp * Az / 360 ;   % isolate Az on right
% Vb * Rp * Az / 360 - Vs * Rp * Az / 360 = Vb * Rs + Vb * Rp - Vs * Rp;    % swap sides
% Az * (Vb * Rp / 360 - Vs * Rp / 360)    = Vb * Rs + Vb * Rp - Vs * Rp;    % factor out Az
% Az * Rp / 360 * (Vb - Vs)               = Vb * Rs + Vb * Rp - Vs * Rp;    % factor out Rp/360
% Az * (Vb - Vs)              = 360 / Rp * (Vb * Rs + Vb * Rp - Vs * Rp);        % move Rp / 360 to right
% Az * (Vb - Vs)                   = 360 * (Vb * Rs + Vb * Rp - Vs * Rp) / Rp;   % move Rp to end
Az = 360 * (Vb * Rs + Vb .* Rp - Vs * Rp) / (Rp * (Vb - Vs));                    % move (Vb - Vs) to right

ADCt = Vt / Vs * ADCmax;
ADCb = Vb / Vs * ADCmax;

It = (Vs - Vt) / Rs;
Ib = (Vs - Vb) / Rs;

SubPlotCnt = 4;
SubPlotIdx = 1;
figure(1); clf;
subplot(SubPlotCnt, 1, SubPlotIdx); hold on; grid on;

plot(Az, Vt);
plot(Az, Vb);
xlim([ min(Az) max(Az)]);
title('ADC Voltage vs Azimuth');
xlabel('angle (deg)');
ylabel('ADC Voltage');

SubPlotIdx = SubPlotIdx + 1; 
subplot(SubPlotCnt, 1, 2); hold on; grid on;
plot(Az, ADCt);
plot(Az, ADCb);
xlim([ min(Az) max(Az)]);
title('ADC reading vs Azimuth');
xlabel('angle (deg)');
ylabel('ADC reading');

SubPlotIdx = SubPlotIdx + 1; 
subplot(SubPlotCnt, 1, SubPlotIdx); hold on; grid on;
plot(Az, AzTop);
plot(Az, AzBot);
xlim([ min(Az) max(Az)]);
title('ADC calculated vs Azimuth input');
xlabel('angle (deg)');
ylabel('Angle calculated(deg)');

SubPlotIdx = SubPlotIdx + 1; 
subplot(SubPlotCnt, 1, SubPlotIdx); hold on; grid on;
plot(Az, It);
plot(Az, Ib);
plot(Az, It + Ib);
xlim([ min(Az) max(Az)]);
title('Wiper current vs Azimuth input');
xlabel('Wiper current (Amps)');
ylabel('Angle calculated(deg)');


%% Earlier attempts, some successful
%% solve for A interms of Vt and resistor constants
% Vt = Vs * ((Rp * A)  ./ (Rp * A + Rb));
% Vt * (Rp * A + Rb) = Vs * ((Rp * A));  
% Vt * (Rp * A + Rb) = Vs * Rp * A;
% (Rp * A + Rb) = Vs / Vt * Rp * A;
% Rp * A = ((Vs / Vt) * Rp * A) - Rb;
% Rp * A - ((Vs / Vt) * Rp * A) = -Rb;
% ((Vs / Vt) * Rp * A) - (Rp * A) = Rb;
% A * ((Vs / Vt) * Rp) - Rp = Rb;
% A = Rb / (((Vs / Vt) * Rp) - Rp);

%% solve for A in terms of Vb and resistors constants
% Vb * (Rp * (1-A) + Rb) = Vs * ((Rp * (1-A)));
% Vb * (Rp * (1-A) + Rb) = Vs * Rp * (1-A);
% (Rp * (1-A) + Rb) = Vs / Vb * Rp * (1-A);
% (Rp * (1-A)) - ((Vs / Vb) * Rp * (1-A)) = -Rb;
% ((Vs / Vb) * Rp * B) - (Rp * B) = Rb;
%  B * ((Vs / Vb) * Rp) - Rp = Rb;
%  B = Rb / (((Vs / Vb) * Rp) - Rp);
%  1-A = Rb / (((Vs / Vb) * Rp) - Rp);
%  -A = Rb / (((Vs / Vb) * Rp) - Rp)-1;
%  A = 1 - (Rb / (((Vs / Vb) * Rp) - Rp));

%% Include effects of wiper contact resistance
% 
% Vt = Vs - (It .* (Rb + Rp .*      A  + Rw));
% Vb = Vs - (Ib .* (Rb + Rb .* (1 - A) + Rw));

% Vw = Iw .* Rw;
% Iw = Vw ./ Rw;

% It = (Vs - Vw) ./ (Rb + Rp .*      A);
% Ib = (Vs - Vw) ./ (Rb + Rp .* (1 - A));

% Vt, Vb  is Vs less voltage drop across Rb
% Vt = Vs - (It * Rb);
% Vb = Vs - (Ib * Rb);

% plugging It, Ib into voltage equation
% Vt = Vs - ((Rb * (Vs - Vw)) / (Rb + Rp *      A));
% Vb = Vs - ((Rb * (Vs - Vw)) / (Rb + Rp * (1 - A)));

% 
% Vt = Vs - (Rb * Vs - Rb * Vw) / (Rb + Rp *      A);
%Vb = Vs - (Rb * Vs - Rb * Vw) / (Rb + Rp * (1 - A));

% right hand side is same for both
% Vt * (Rb + Rp *      A)  = Vs - (Rb * Vs - Rb * Vw);
% Vb * (Rb + Rp * (1 - A)) = Vs - (Rb * Vs - Rb * Vw);

% Vt * (Rb + Rp * A) = Vb * (Rb + Rp * (1 - A));
% Vt * Rb + Vt * Rp * A = Vb * Rb + Vb * Rp * (1 - A);
% Vt * Rb = Vb * Rb + Vb * Rp * (1 - A) - (Vt * Rp * A);
% Vt * Rb - (Vb * Rb) = Vb * Rp * (1 - A) - (Vt * Rp * A);
% Rb * (Vt - Vb) = Vb * Rp * (1 - A) - (Vt * Rp * A);
% Rb * (Vt - Vb) = Rp * (Vb * (1 - A)) - (Vt * A);
% Rb / Rp * (Vt - Vb) = (Vb * (1 - A)) - (Vt * A);
% Rb / Rp * (Vt - Vb) = Vb  - Vb * A - Vt * A;
% Rb / Rp * (Vt - Vb) = Vb  - A * (Vb - Vt);
% Rb / Rp * (Vt - Vb) = Vb  + A * (Vt - Vb);
% (Rb / Rp * (Vt - Vb)) - Vb = A * (Vt - Vb);
% ((Rb / Rp * (Vt - Vb)) - Vb) / (Vt - Vb) = A;
%A = ((Rb / Rp * (Vt - Vb)) - Vb) / (Vt - Vb);
%A = (Rb / Rp  - Vb) ./ (Vt - Vb);

%% per Kichoff example
% Vs = Vb + Vp(  A) + Vw
% Vs = Vb + Vp(1-A) + Vw
% Vs = It * Rb + It * Rp *     A ) + Iw * Rw;
% Vs = Ib * Rb + Ib * Rp * (1 -A)) + Iw * Rw;

% Vs = It * Rb + It * Rp *     A ) + (It + Ib) * Rw;
% Vs = Ib * Rb + Ib * Rp * (1 -A)) + (It + Ib) * Rw;


%  It * Rb  +  It * Rp * A  + (It + Ib) * Rw        =  Ib * Rb  + (Ib * Rp * (1 - A))     + (It + Ib) * Rw;
% (It * Rb) + (It * Rp * A) + (It * Rw) + (Ib * Rw) = (Ib * Rb) +  Ib * Rp - Ib * Rp * A  + (It * Rw) + (Ib * Rw);
% (It * Rb) + (It * Rp * A)                         = (Ib * Rb) +  Ib * Rp - Ib * Rp * A                         ;
