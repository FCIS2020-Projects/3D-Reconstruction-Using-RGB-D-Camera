Since libpxcutils uses STL, it is necessary generate compiler specific 
versions. This is done using a common OutputDirectory ($OutDir) definition
(lib/$(PlatformName)/$(PlatformToolset)/) within the various project files.

libpxcutils_vs2013.vcxproj -> lib/$(PlatformName)/v120
libpxcutils_vs2012.vcxproj -> lib/$(PlatformName)/v110
libpxcutils_vs2010.vcxproj -> lib/$(PlatformName)/v100
