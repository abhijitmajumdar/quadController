#ifndef _CONFIGLOADER_H
#define _CONFIGLOADER_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>

using namespace std;

struct lConst
{
	string name;
	long value;
};

struct fConst
{
	string name;
	float value;
};

struct bConst
{
	string name;
	bool value;
};

class qConfig{
	public:
		static void readConfigFile(std::string configFileName,lConst* lC,int lCn,fConst* fC,int fCn,bConst* bC,int bCn);
	private:
		static int store_line(std::string key,std::string val,lConst* lC,int lCn,fConst* fC,int fCn,bConst* bC,int bCn);
};

#endif //_CONFIGLOADER_H
