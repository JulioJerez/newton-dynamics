/* Copyright (c) <2003-2022> <Newton Game Dynamics>
* 
* This software is provided 'as-is', without any express or implied
* warranty. In no event will the authors be held liable for any damages
* arising from the use of this software.
* 
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely
*/

#if (defined(WIN32) || defined(_WIN32))
#include <windows.h>
#include <commdlg.h>
#endif

#include "ndFileBrowser.h"

//static void strtolwr(char* const string)
//{
//	for (char * cp = string; *cp; ++cp)
//	{
//		if ((*cp >= 'A') && (*cp <= 'Z'))
//		{
//			*cp += 'a' - 'A';
//		}
//	}
//}

bool ndGetSaveFileName(char* const fileName, int maxSize)
{
#if (defined(WIN32) || defined(_WIN32))
	OPENFILENAME ofn;
	// open a file name
	char appPath[256];
	GetModuleFileNameA(nullptr, appPath, sizeof(appPath));
	//strtolwr(appPath);

	char* const end = strstr(appPath, "applications");
	end[0] = 0;
	strcat(appPath, "applications\\ndSandbox");

	ZeroMemory(&ofn, sizeof(ofn));
	ofn.lStructSize = sizeof(ofn);
	ofn.hwndOwner = nullptr;
	ofn.lpstrFile = fileName;
	ofn.lpstrFile[0] = '\0';
	ofn.nMaxFile = DWORD(maxSize);
	ofn.lpstrFilter = const_cast<LPSTR>("newton save file *.nd\0*.nd\0");
	ofn.nFilterIndex = 1;
	ofn.lpstrFileTitle = const_cast<LPSTR>("Newton Dynamics 4.0 demos");
	ofn.nMaxFileTitle = 0;
	ofn.lpstrInitialDir = appPath;
	ofn.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST;

	bool state = GetSaveFileName(&ofn) ? true : false;
	if (state) 
	{
		char* const ext = strrchr(fileName, '.');
		if (!ext) 
		{
			strcat(fileName, ".nd");
		}
	}
	return state;
#else
	return false;
#endif
}

bool ndGetLoadFileName(char* const fileName, int maxSize)
{
#if (defined(WIN32) || defined(_WIN32))
	OPENFILENAME ofn;
	// open a file name
	char appPath[1024];
	GetModuleFileNameA(nullptr, appPath, sizeof(appPath));
	//strtolwr(appPath);

	char* const end = strstr(appPath, "applications");
	end[0] = 0;
	strcat(appPath, "applications\\ndSandbox");

	ZeroMemory(&ofn, sizeof(ofn));
	ofn.lStructSize = sizeof(ofn);
	ofn.hwndOwner = nullptr;
	ofn.lpstrFile = fileName;
	ofn.nMaxFile = DWORD(maxSize);
	ofn.lpstrFilter = const_cast<LPSTR>("ndMesh file *.nd\0*.nd\0native xml *.xml\0*.xml\0All Files (*.*)\0*.*\0");
	ofn.nFilterIndex = 1;
	ofn.lpstrFileTitle = const_cast<LPSTR>("Newton Dynamics 4.0 demos");
	ofn.nMaxFileTitle = 0;
	ofn.lpstrInitialDir = appPath;
	ofn.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST;

	bool state = GetOpenFileName(&ofn) ? true : false;
	if (state)
	{
		char* const ext = strrchr(fileName, '.');
		if (!ext)
		{
			strcat(fileName, ".nd");
		}
	}
	return state;
#else
	return false;
#endif
}
