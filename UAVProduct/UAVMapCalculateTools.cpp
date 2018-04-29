//
// Created by wuwei on 18-4-20.
// Map title tools
//
//
//
//
//
#include "UAVMapCalculateTools.h"
#include <dirent.h>



//standarded directory structure
//level-rows-cols
void GetMapUnitsFromDirectory(string cate_dir,std::vector<MAPUNIT> &mapUnits)
{
#ifdef WIN32
    _finddata_t file;
    long lf;
    if ((lf=_findfirst(cate_dir.c_str(), &file)) == -1) {
        cout<<cate_dir<<" not found!!!"<<endl;
    } else {
        while(_findnext(lf, &file) == 0) {
            if (strcmp(file.name, ".") == 0 || strcmp(file.name, "..") == 0)
                continue;
            //
            if((fileinfo.attrib &  _A_SUBDIR))
            {

            }
            files.push_back(file.name);
        }
    }
    _findclose(lf);
#endif

#ifdef linux
    DIR *dir;
    struct dirent *ptr;
    char base[1000];

    if ((dir=opendir(cate_dir.c_str())) == NULL)
    {
        perror("Open dir error...");
        exit(1);
    }

    int level = 0;
    int rows  = 0;
    int cols  = 0;

    while ((ptr=readdir(dir)) != NULL)
    {

        if(ptr->d_type == 4)    ///dir
        {
            level=atoi(ptr->d_name);
            if(!strcmp(ptr->d_name,"..")||!strcmp(ptr->d_name,"."))
                continue;
            DIR *dirlevel;
            string dirl = cate_dir+"/"+string(ptr->d_name);
            if((dirlevel=opendir(dirl.c_str())) == NULL)
                continue;
            else
            {
                struct dirent *ptrLevel;
                //第二层
                while ((ptrLevel=readdir(dirlevel)) != NULL){
                    if(ptrLevel->d_type == 4)    ///dir
                    {
                        if(!strcmp(ptrLevel->d_name,"..")||!strcmp(ptrLevel->d_name,"."))
                            continue;
                        rows=atoi(ptrLevel->d_name);
                        DIR *dirRows;
                        string dirR = dirl+"/"+string(ptrLevel->d_name);
                        if((dirRows=opendir(dirR.c_str())) == NULL)
                            continue;
                        else{
                            struct dirent *ptrRows;
                            //第三层
                            while ((ptrRows=readdir(dirRows)) != NULL){
                                if(ptrRows->d_type == 8)    ///file
                                {
                                    string name = string(ptrRows->d_name);
                                    string::size_type pos=name.rfind('.');
                                    string ext=name.substr(pos==string::npos?name.length():pos+1);
                                    string strcol=name.substr(0,name.length()-4);
                                    cols = atoi(strcol.c_str());
                                    if(!strcmp("jpg", ext.c_str() ))
                                    {
                                        cols = atoi(ptrRows->d_name);
                                        MAPUNIT unit;
                                        unit.level = level;
                                        unit.row   = rows;
                                        unit.col   = cols;
                                        unit.unit_save = (dirR+"/"+string(ptrRows->d_name));
                                        mapUnits.push_back(unit);
                                    }
                                }
                            }//while level3
                            closedir(dirRows);
                        }

                    }

                }//while level2
                closedir(dirlevel);
            }
        }
    }//wwhile level1
    closedir(dir);
#endif

}