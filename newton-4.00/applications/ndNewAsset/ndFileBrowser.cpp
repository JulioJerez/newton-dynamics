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
#include <ndNewton.h>

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
	ofn.lpstrFileTitle = const_cast<LPSTR>("Newton asset editor");
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
	ofn.lpstrFilter = const_cast<LPSTR>("newton load file *.nd\0*.nd\0All Files (*.*)\0*.*\0");
	ofn.nFilterIndex = 1;
	ofn.lpstrFileTitle = const_cast<LPSTR>("Newton asset editor");
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

bool ndGetImportFbxFileName(char* const fileName, int maxSize)
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
	ofn.lpstrFilter = const_cast<LPSTR>("newton load file *.fbx\0*.fbx\0");
	ofn.nFilterIndex = 1;
	ofn.lpstrFileTitle = const_cast<LPSTR>("Newton asset editor");
	ofn.nMaxFileTitle = 0;
	ofn.lpstrInitialDir = appPath;
	ofn.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST;

	bool state = GetOpenFileName(&ofn) ? true : false;
	if (state)
	{
		char* const ext = strrchr(fileName, '.');
		if (!ext)
		{
			strcat(fileName, ".fbx");
		}
	}
	return state;
#else
	return false;
#endif
}

bool ndGetImportUrdfFileName(char* const fileName, int maxSize)
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
	ofn.lpstrFilter = const_cast<LPSTR>("newton load file *.urdf\0*.urdf\0");
	ofn.nFilterIndex = 1;
	ofn.lpstrFileTitle = const_cast<LPSTR>("Newton asset editor");
	ofn.nMaxFileTitle = 0;
	ofn.lpstrInitialDir = appPath;
	ofn.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST;

	bool state = GetOpenFileName(&ofn) ? true : false;
	if (state)
	{
		char* const ext = strrchr(fileName, '.');
		if (!ext)
		{
			strcat(fileName, ".urdf");
		}
	}
	return state;
#else
	return false;
#endif
}

bool ndGetWorkingFileName(const char* const basePath, char* const name, ndInt32 maxSize)
{
#if (defined(WIN32) || defined(_WIN32))

	ndFixSizeArray<ndString, 32> stack;
	ndString path(basePath);
	path.ToLower();
	stack.PushBack(path);

	ndString fileName(name);
	fileName.ToLower();
	while (stack.GetCount())
	{
		const ndString baseDir(stack.Pop());
		const ndString directory(baseDir + "*");

		WIN32_FIND_DATA file;
		HANDLE search_handle = FindFirstFile(directory.GetStr(), &file);
		if (search_handle != INVALID_HANDLE_VALUE)
		{
			do
			{
				ndString thisName(file.cFileName);
				thisName.ToLower();
				if ((thisName != ".") && (thisName != ".."))
				{
					if (file.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY)
					{
						ndString subDirectory(baseDir);
						subDirectory += thisName;
						subDirectory += "/";
						stack.PushBack(subDirectory);
					}
					else
					{
						if (fileName == thisName)
						{
							const ndString filePath(baseDir + fileName);
							snprintf(name, size_t(maxSize), filePath.GetStr());
							FindClose(search_handle);
							return true;
						}
					}
				}
			} while (FindNextFile(search_handle, &file));
		}
		FindClose(search_handle);
	}
	return false;
#else
	return false;
#endif
}