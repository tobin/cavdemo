    function varargout = cavdemo(varargin)
% cavdemo Interactive demo of resonant cavity reflection coefficient
%
% This little demo plots the complex reflection coefficient of a
% fabry-perot cavity, the intra-cavity power buildup, and the reflected
% phase versus detuning.  Use the sliders to adjust the amplitude
% reflectivity coefficients of the two mirrors between values of 0 and 1.
% Two check-boxes allow you to hold either the product r1*r2 or the
% quotient r1/r2 constant, corresponding to fixed finesse or fixed
% impedance matching.
%
% This software is not at all polished - just a quick and dirty demo. 
%
% Tobin Fricke <tfricke@ligo.caltech.edu> August 26, 2010

% Edit the above text to modify the response to help cavdemo

% Last Modified by GUIDE v2.5 30-Aug-2010 14:35:04

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @cavdemo_OpeningFcn, ...
                   'gui_OutputFcn',  @cavdemo_OutputFcn, ...
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

% --- Executes just before cavdemo is made visible.
function cavdemo_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to cavdemo (see VARARGIN)

% Choose default command line output for cavdemo
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes cavdemo wait for user response (see UIRESUME)
% uiwait(handles.figure1);
r1slider_Callback(hObject, eventdata, handles)

% --- Outputs from this function are returned to the command line.
function varargout = cavdemo_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on slider movement.
function r1slider_Callback(hObject, eventdata, handles)
% hObject    handle to r1slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of
%        slider
r1 = get(handles.r1slider, 'Value');
r2 = get(handles.r2slider, 'Value');
constantF = get(handles.FinesseCheckbox, 'Value');
if constantF,    
    if handles.r1 ~= r1,
        r2 = (handles.r1 * handles.r2) / r1;
    else
        r1 = (handles.r1 * handles.r2) / r2;
    end
elseif get(handles.CriticalCheckbox, 'Value'),
   if handles.r1 ~= r1,
        r2 = (handles.r1 / handles.r2) * r1;
    else
        r1 = (handles.r1 / handles.r2) * r2;
    end   
end

handles.r1 = r1;
handles.r2 = r2;    
set(handles.r1slider, 'Value', handles.r1);
set(handles.r2slider, 'Value', handles.r2);
guidata(hObject, handles)
replot();

% --- Executes during object creation, after setting all properties.
function r1slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to r1slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function r2slider_Callback(hObject, eventdata, handles)
% hObject    handle to r2slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
r1slider_Callback(hObject, eventdata, handles);

% --- Executes during object creation, after setting all properties.
function r2slider_CreateFcn(hObject, eventdata, handles)
% hObject    handle to r2slider (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function r1valueedit_Callback(hObject, eventdata, handles)
% hObject    handle to r1valueedit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of r1valueedit as text
%        str2double(get(hObject,'String')) returns contents of r1valueedit as a double


% --- Executes during object creation, after setting all properties.
function r1valueedit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to r1valueedit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function r2valueedit_Callback(hObject, eventdata, handles)
% hObject    handle to r2valueedit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of r2valueedit as text
%        str2double(get(hObject,'String')) returns contents of r2valueedit as a double


% --- Executes during object creation, after setting all properties.
function r2valueedit_CreateFcn(hObject, eventdata, handles)
% hObject    handle to r2valueedit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function replot()

h_r1 = findobj(gcf, 'Tag', 'r1slider');
h_r2 = findobj(gcf, 'Tag', 'r2slider');

r1 = get(h_r1, 'Value');
r2 = get(h_r2, 'Value');

%t1 = sqrt(1 - r1^2);
%t2 = sqrt(1 - r2^2);
phi = linspace(-pi/2, pi/2, 101);

%t = -t1*t2*exp(1i*phi)      ./ (1 - r1*r2*exp(1i*2*phi));
r = (r1 - r2*exp(1i*2*phi)) ./ (1 - r1*r2*exp(1i*2*phi));

morephi = linspace(-pi/2, pi/2, 101);
radius = r2 * (r1^2 - 1) ./ (1 - (r1*r2)^2);
center = r1 * (1 - r2^2) ./ (1 - (r1*r2)^2);
circle = radius * exp(1i*2*morephi) + center;

h_axes = findobj(gcf, 'Tag', 'MyAxes');
if isempty(h_axes),
    error('Could not find the axes )-:');
end
plot(h_axes, real(circle), imag(circle), 'k-', ...
             real(r), imag(r), 'ro');
%plot(real(t), imag(t),'x');
axis(h_axes, 'equal');
set(h_axes, 'Xlim', [-1 1], 'Ylim', [-1 1]);
grid(h_axes, 'on');
% line(h_axes,[0 0], [-1 1], 'color', 'k');
% line(h_axes,[-1 1], [0 0], 'color', 'k');

h_axes2 = findobj(gcf, 'Tag', 'MyAxes2');
g = sqrt(1 - r1^2)/(1 - r1*r2);
F = 4 * r1*r2 / (1 - r1*r2)^2;
plot(h_axes2, 2*phi, g^2./(1 + F * sin(phi).^2));
set(h_axes2, 'XLim', [-1 1]*pi, 'YLim', [0 10]);

h_axes3 = findobj(gcf, 'Tag', 'MyAxes3');
plot(h_axes3, 2*phi, (angle(r))*180/pi,'o');
set(h_axes3,'XLim', [-1 1]*pi, 'YLim', [-180 180]);



% --- Executes on button press in FinesseCheckbox.
function FinesseCheckbox_Callback(hObject, eventdata, handles)
% hObject    handle to FinesseCheckbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of FinesseCheckbox
if get(hObject, 'Value')
    set(handles.CriticalCheckbox, 'Value', 0);
end

% --- Executes on button press in CriticalCheckbox.
function CriticalCheckbox_Callback(hObject, eventdata, handles)
% hObject    handle to CriticalCheckbox (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of CriticalCheckbox
if get(hObject, 'Value')
    set(handles.FinesseCheckbox, 'Value', 0);
end


% --- Executes during object creation, after setting all properties.
function MyAxes_CreateFcn(hObject, eventdata, handles)
% hObject    handle to MyAxes (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
axis(hObject, 'equal');
set(hObject, 'Xlim', [-1 1], 'Ylim', [-1 1]);
grid(hObject, 'on');
line([0 0], [-1 1], 'color', 'k');
line([-1 1], [0 0], 'color', 'k');
% Hint: place code in OpeningFcn to populate MyAxes
