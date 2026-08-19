function names = motorTypeNames()
%MOTORTYPENAMES  Motor type number -> physical servo model name.
%
%   Mirrors ../ServoDAQ/MOTOR_TYPES.md's inventory table -- that file is
%   the authoritative record of the actual 8-servo study inventory;
%   update both if it ever changes. This is just the MATLAB-side lookup
%   setup.m (and anything else that wants it) uses to turn "type1" into
%   something a reader would actually recognize.
%
%   A type number with no entry here simply has no name -- callers fall
%   back to "type<N>", they don't error.

    names = containers.Map('KeyType', 'double', 'ValueType', 'char');
    names(1) = 'Miuzei 25kg Servo';
    names(2) = 'Knockoff MG996R';
    names(3) = 'MG90D';
end
