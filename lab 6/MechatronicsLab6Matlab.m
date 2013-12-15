clc;
clear all;
close all;
a = [5 25381;
    10 19897;
    15 15711;
    20 12493;
    25 10000;
    30 8056;
    35 6529.7;
    40 5323.9;
    45 4365.3;
    50 3598.7;
    55 2982.3];

regression = polyfit (a (:, 2), a (:, 1), 3)

for i = 1 : 11
    regressionLineX (i) = a (i, 2);
    regressionLineY (i) = (regression (1) .* (a (i, 2) .^ 3)) + ...
                            (regression (2) .* (a (i, 2) .^ 2)) + ...
                            (regression (3) .* (a (i, 2) .^ 1)) + ...
                            (regression (4));% .* (a (i, 2) .^ 3)) + ...
%                             (regression (5) .* (a (i, 2) .^ 2)) + ...
%                             (regression (6) .* a (i, 2)) + ...
%                             regression (7);
end

plot (a (:, 2), a (:, 1), regressionLineX, regressionLineY);
