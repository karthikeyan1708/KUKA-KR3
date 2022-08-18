
function kr3Teach(q)

global kr3 Qd


if ~exist('q','var')
    q=Qd;
end

qrad = q*pi/180;


L = kr3.links;
if (qrad(1) < L(1).qlim(1)) || (qrad(1) > L(1).qlim(2))
    error('acute angle of joint 1 exceeds the limits..');
elseif (qrad(2) < L(2).qlim(1)) || (qrad(2) > L(2).qlim(2))
    error('acute angle of joint 2 exceeds the limits.');
elseif (qrad(3) < L(3).qlim(1)) || (qrad(3) > L(3).qlim(2))
    error('acute angle of joint 3 exceeds the limits.');
elseif (qrad(4) < L(4).qlim(1)) || (qrad(4) > L(4).qlim(2))
    error('acute angle of joint 4 exceeds the limits.');
elseif (qrad(5) < L(5).qlim(1)) || (qrad(5) > L(5).qlim(2))
    error('acute angle of joint 5 exceeds the limits.');
elseif (qrad(6) < L(6).qlim(1)) || (qrad(6) > L(6).qlim(2))
    error('acute angle of joint 6 exceeds the limits.');
end

warning('off','RTB:SerialLink:plot');


kr3.teach(qrad, 'notiles', 'floorlevel', 1, 'lightpos', [0 0 -20])

%
set(gca, 'ZDir', 'reverse', 'YDir', 'reverse', ...
    'ZLim', [-1400 1], 'YLim', [-700 700], 'XLim', [-700 700], ...
    'ZLimMode', 'manual', 'YLimMode', 'manual', 'XLimMode', 'manual');