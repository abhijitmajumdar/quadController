#ifndef _CONFIGLOADER_H
#define _CONFIGLOADER_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <map>

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
		static void readConfigFile(std::string configFileName, std::map<int,lConst> &lC, int nlC, std::map<int,fConst> &fC, int nfC, std::map<int,bConst> &bC, int nbC);
	private:
		static int store_line(std::string key, std::string val, std::map<int,lConst> &lC, int nlC, std::map<int,fConst> &fC, int nfC, std::map<int,bConst> &bC, int nbC);
};

#endif //_CONFIGLOADER_H
