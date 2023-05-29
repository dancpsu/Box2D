@echo off
echo Unzip http://download.savannah.gnu.org/releases/tinycc/tcc-0.9.27-win64-bin.zip to C:\rawdraw\tcc
echo Also, if compiling with OpenGL, download http://download.savannah.nongnu.org/releases/tinycc/winapi-full-for-0.9.27.zip and overwrite the include, etc. folders in C:\tcc.
echo Don't worry.  It includes the i386 compiler in the win64 build.

set CFLAGS= -v -DHIDAPI -DWINDOWS -DWIN32 -DTCC -DRUNTIME_SYMNUM -O2 -Itccinc -DINCLUDING_EMBEDDED -rdynamic -g 
set INCLUDES=-Irawdraw -Iinclude
set LDFLAGS=-lkernel32 -lole32 -lgdi32 -luser32 -lsetupapi -ldbghelp -lws2_32 -lAvrt -lopengl32

set SOURCES=main.c
  
set ARCH_SPECIFIC=-L32 C:\windows\system32\winmm.dll -DWIN32_LEAN_AND_MEAN 
set CC=C:\tcc\i386-win32-tcc.exe

@echo on
%CC% %CFLAGS% %INCLUDES% %ARCH_SPECIFIC% %SOURCES% %LDFLAGS% -o out.exe
@echo off
cmd /c if exist out.exe out.exe