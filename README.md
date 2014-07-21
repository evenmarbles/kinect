kinect
======

Extraction of motion capture data from the Kinect


setup (Visual Studio)
=====================

Download and unzip the MSVC freeglut binaries from
http://www.transmissionzero.co.uk/software/freeglut-devel/

Copy freeglut.lib in the lib folder of the archive to
C:\Program Files (x86)\Microsoft Visual Studio 12.0\VC\lib

Copy the GL folder in the include folder of the archive to
C:\Program Files (x86)\Microsoft Visual Studio 12.0\VC\include

Copy the freeglut.dll in the bin folder of the archive to
C:\Windows\SysWOW64 (C:\Windows\System32 for 32-bit Windows)


Download and unzip the GLEW binaries from
http://glew.sourceforge.net/

Copy glew32.lib in the lib/release/win32 folder of the archive to
C:\Program Files (x86)\Microsoft Visual Studio 12.0\VC\lib

Copy all files in the include folder of the archive to
C:\Program Files (x86)\Microsoft Visual Studio 12.0\VC\include

Copy the glew32.dll in the bin\release\win32 folder of the archive to
C:\Windows\SysWOW64 (C:\Windows\System32 for 32-bit Windows)

note
====

If you are setting up your own Visual Studio project, in the project's properties:

Under C/C++ > Additional Include Directories, add "$(KINECTSDK10_DIR)\include".
Under Linker > Additional Library Directories, add "$(KINECTSDK10_DIR)\lib\x86".
Under Linker > Input > Additional Dependencies, add "kinect10.lib".
Under Linker > System > Subsystem, add "Windows (/SUBSYTEM:WINDOWS)".
Under Linker > Advanced > Entry Point, add "mainCRTStartup".
