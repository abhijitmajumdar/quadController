#include "configLoader.h"

//Make sure to compile with "-std=c++11"
int qConfig::store_line(std::string key, std::string val, std::map<int,lConst> &lC, int nlC, std::map<int,fConst> &fC, int nfC, std::map<int,bConst> &bC, int nbC)
{
	int i;
	for(i=0;i<nlC;i++)
	{
		if(lC[i].name == key)
		{
			lC[i].value = stol(val);
			return 0;
		}
	}
	for(i=0;i<nfC;i++)
	{
		if(fC[i].name == key)
		{
			fC[i].value = stof(val);
			return 0;
		}
	}
	for(i=0;i<nbC;i++)
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

void qConfig::readConfigFile(std::string configFileName, std::map<int,lConst> &lC, int nlC, std::map<int,fConst> &fC, int nfC, std::map<int,bConst> &bC, int nbC)
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
				  if(store_line(key,value,lC,nlC,fC,nfC,bC,nbC)==1)
					cout<<"Could not find "<<key<<"\n";
			}
		}
		
		//Close file at the end
		configFile.close();
	}
	else
	{
		cout<<"Error reading config file. Loading deafult values\n";
	}
}
