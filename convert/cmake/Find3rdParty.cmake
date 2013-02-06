set(ThirdParty_DIR ${PROJECT_SOURCE_DIR}/3rd-party)

set(GCSS3DLib_INCLUDE_DIR ${ThirdParty_DIR}/GCSS3DLib/include)
set(GCSS3DLib_LIBRARY_DIRS ${ThirdParty_DIR}/GCSS3DLib/lib)
set(GCSS3DLib_LIBRARY optimized GCSS3DLib.lib debug GCSS3DLib.lib)

include_directories(${GCSS3DLib_INCLUDE_DIR})
link_directories(${GCSS3DLib_LIBRARY_DIRS})
set(ThirdParty_LIBS ${ThirdParty_LIBS} ${GCSS3DLib_LIBRARY})

#set(MFC42_LIBRARY_DIRS ${ThirdParty_DIR}/MFC42/lib)
#set(MFC42_LIBRARY optimized MFC42.lib debug MFC42D.lib)

#link_directories(${MFC42_LIBRARY_DIRS})
#set(ThirdParty_LIBS ${ThirdParty_LIBS} ${MFC42_LIBRARY})

