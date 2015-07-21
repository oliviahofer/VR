

What I did:

- create Project from tutorials-Project (http://www.nm.ifi.lmu.de/teaching/Vorlesungen/2015ss/vr/ --> Übungsmaterial)
Visual Studio 2012 Express (https://www.microsoft.com/de-de/download/details.aspx?id=34673)
CMake 3.2 oder neuer (http://www.cmake.org/files/v3.2/cmake-3.2.2-win32-x86.exe)
Das Archiv "libraries.zip" bitte ins Stammverzeichnis von C: entpacken.
Das Archiv "tutorials.zip" in C:\Users\Olivia\Documents\VR\tutorials -> kopie angelegt C:\Users\Olivia\Documents\VR\tutorials (opensg_vpnr)
Rename Project (CMakeLists : 3 project(bubbles) 9 add_subdirectory(Bubbles), Bubbles/CMakeLists : 7 project(bubbles) 10 add_project(Bubbles ...), vs11_32_osg_invrs.cmake : bubbles.cmake)
Die Datei "bubbles.cmake" auf "run_cmake.bat" ziehen (Drag'n'drop). (Cmake erstellt nun unter "build_bubbles" die Projekt-Dateien für Visual Studio. Danach wird ein Shortcut "start_vs_bubbles" erstellt.)
Mit dem erstellten Shortcut (Verknüpfung auf "build_bubbles/bubbles.sln") starten sie die Entwicklungsumgebung mit den einzelnen Projekten (Release-Version).

- build PortAudio 
downlaod PortAudio (http://www.portaudio.com/download.html, Version: pa_stable_v19_20140130.tgz)
download ASIO SDK (http://www.steinberg.net/en/company/developers.html)
put 3 directories to the same level: /ASIO / portaudio /Bubbles 
set PORTAUDIO_PATH : C:\Users\Olivia\Documents\VR\Bubbles\portaudio
cmake-gui (source code: C:/Users/Olivia/Documents/VR/Bubbles/portaudio, binaries: C:/Users/Olivia/Documents/VR/Bubbles/portaudio/bin) --> Configure : Visual Studio 11 2012 --> Configure --> Generate
open C:/Users/Olivia/Documents/VR/Bubbles/portaudio/bin/portaudio.sln
In Visual Studio ( Projekt --> Eigenschaften (Konfiguration:Aktiv(Debug), Plattform : Aktiv(Win32))
	-> Konfigurationseigenschaften -> C/C++ --> Präprozessor --> Präprozessordefinitionen : PA_WDMKS_NO_KSGUID_LIB
Projekt erstellen (in C:\Users\Olivia\Documents\VR\Bubbles\portaudio\bin\Debug sind jetzt portaudio_x86.dll und portaudio_86.lib)

- add PortAudio
In Visual Studio Bubbles ( Projekt --> BubblesEigenschaften (Konfiguration:Aktiv(Debug), Plattform : Aktiv(Win32))
	-> Konfigurationseigenschaften -> C/C++ --> Allgemein --> Zusätzliche Includeverzeichnisse : ..\..\..\portaudio\include
					-> Linker --> Allgemein --> Zusätzliche Bibliothekverzeichnisse : ..\..\..\\portaudio\bin\Debug
					-> Linker --> Eingabe --> Zusätzliche Abhängigkeiten : ..\..\..\\portaudio\bin\Debug\portaudio_x86.lib
copy portaudio_x86.dll to C:\Users\Olivia\Documents\VR\Bubbles\build_bubbles\Bubbles\Debug (Where Bubbles executable is)

nr
