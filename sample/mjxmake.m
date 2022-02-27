% clear

if exist('mjx')==3
	mjx('exit');
end
clear mjx;


% compile

if isunix
	mex -v -output ../bin/mjx -D_NIX -I../include/ mjx.cpp -L../bin/ -lmujoco200 -lGL -lglew libglfw.so.3 
else
	mex -v -output ..\bin\mjx -DWIN32 -D_WIN32 -I..\include\ mjx.cpp -L..\bin\ -lmujoco200 -lglfw3
end
