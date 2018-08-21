function varargout = start_stop_button(varargin)
% START_STOP_BUTTON MATLAB code for start_stop_button.fig
%      START_STOP_BUTTON, by itself, creates a new START_STOP_BUTTON or raises the existing
%      singleton*.
%
%      H = START_STOP_BUTTON returns the handle to a new START_STOP_BUTTON or the handle to
%      the existing singleton*.
%
%      START_STOP_BUTTON('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in START_STOP_BUTTON.M with the given input arguments.
%
%      START_STOP_BUTTON('Property','Value',...) creates a new START_STOP_BUTTON or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before start_stop_button_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to start_stop_button_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help start_stop_button

% Last Modified by GUIDE v2.5 20-Aug-2018 21:50:21

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @start_stop_button_OpeningFcn, ...
                   'gui_OutputFcn',  @start_stop_button_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before start_stop_button is made visible.
function start_stop_button_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to start_stop_button (see VARARGIN)

% Choose default command line output for start_stop_button
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);
global start_stop_state close_state
start_stop_state = 1;
close_state = 0;
% UIWAIT makes start_stop_button wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = start_stop_button_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in start_stop.
function start_stop_Callback(hObject, eventdata, handles)
% hObject    handle to start_stop (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global start_stop_state
if start_stop_state == 1
    start_stop_state = 0;
    set(hObject,'String','START');
    set(hObject,'BackgroundColor','green');
else
    start_stop_state = 1;
    set(hObject,'String','STOP');
    set(hObject,'BackgroundColor','red');
end

% --- Executes on button press in close.
function close_Callback(hObject, eventdata, handles)
% hObject    handle to close (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
global start_stop_state close_state serial_state
close_state = 1;
if start_stop_state == 0
    % close serial
    fclose(serial_state);
end
