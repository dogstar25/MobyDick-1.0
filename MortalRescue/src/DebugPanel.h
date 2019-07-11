#pragma once
#include <stdio.h>
#include<map>
#include <string>

#include <SDL_rect.h>

using namespace std;

class DebugPanel
{
public:
	DebugPanel();
	~DebugPanel();

	void addItem(string, string);

	SDL_Point location;
	int itemCount;
	//map <string,string>debugItems;


};

