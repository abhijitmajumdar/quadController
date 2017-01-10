#include "configLoader.h"

//Make sure to compile with "-std=c++11"
int qConfig::store_line(std::string key,std::string val,lConst* tC,int tCn,lConst* idC,int idCn,fConst* fC,int fCn,bConst* bC,int bCn)
{
	int i;
	for(i=0;i<tCn;i++)
	{
		if(tC[i].name == key)
		{
			tC[i].value = stol(val);
			return 0;
		}
	}
	for(i=0;i<idCn;i++)
	{
		if(idC[i].name == key)
		{
			idC[i].value = stol(val);
			return 0;
		}
	}
	for(i=0;i<fCn;i++)
	{
		if(fC[i].name == key)
		{
			fC[i].value = stof(val);
			return 0;
		}
	}
	for(i=0;i<bCn;i++)
	{
		if(bC[i].name == key)
		{
			if(val == "true")
				bC[i].value = true;
			else
				bC[i].value = false;
			return 0;
		}
	}
	return 1;
}

void qConfig::readConfigFile(std::string configFileName,lConst* tC,int tCn,lConst* idC,int idCn,fConst* fC,int fCn,bConst* bC,int bCn)
{
	fstream configFile;
	configFile.open(configFileName,ios::out | ios::in);
	if(configFile.is_open())
	{
		//Load the configurations
		string line;
		while(getline(configFile,line))
		{
			istringstream is_line(line);
			string key;
			if( getline(is_line, key, '=') )
			{
				string value;
				if(getline(is_line, value) ) 
				  if(store_line(key,value, tC, tCn, idC, idCn, fC, fCn, bC, bCn)==1)
					cout<<"Could not find "<<key<<"\n";
			}
		}
		
		//Close file at the end
		configFile.close();
	}
	else
	{
		cout<<"Error reading config file. Loading deafult values";
	}
}
