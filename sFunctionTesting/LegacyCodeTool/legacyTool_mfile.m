%% info
% shows how an s-function can be created with LegacyCodeTool
% https://www.youtube.com/watch?v=5o5R0VnptZw

%% clear
clc; clear

%% setup compiler
mex -setup
%setenv('','')

%% check compiler
copyfile(fullfile(matlabroot,'extern','examples','mex','yprime.c'),'.','f');
mex yprime.c % this creates mex file
delete yprime.c
delete yprime.mexa64

%% create s-function
def = legacy_code('initialize');        % initialize Legacy Code Tool
def.SourceFiles = {'main.c'};           % source file
def.HeaderFiles = {'add_lib.h'};        % header file
def.SFunctionName = 'my_s_fun';         % name for S-function
def.OutputFcnSpec = 'double y1 = add_2(double u1, double u2)';  % output declaration
legacy_code('sfcn_cmex_generate', def); % creates c-code for the s-fuction (with additional headers)
legacy_code('compile', def);            % compile the new c-file

%% test with simulink
% create a simulink model
% add the sfunction by creating an s-func block and add the name of the s-function
% check if it works